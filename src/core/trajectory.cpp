// SPDX-License-Identifier: Apache-2.0
#include "motionkit/core/trajectory.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <span>
#include <string_view>

#include "motionkit/core/expected.hpp"
#include "motionkit/core/motion_state.hpp"
#include "motionkit/core/types.hpp"

namespace motionkit {
namespace {

/// The positive root of x^2 + b*x - k = 0, for b > 0 and k >= 0.
///
/// The textbook form (-b + sqrt(b*b + 4k)) / 2 subtracts two nearly equal
/// numbers whenever b*b dominates 4k -- which is exactly the regime of a short
/// move against a generous acceleration limit -- and throws away most of the
/// significand doing it. The algebraically identical form below never
/// subtracts, so it is accurate across the whole range instead of only where
/// the move is long.
Scalar positiveQuadraticRoot(Scalar b, Scalar k) noexcept {
  return 2.0 * k / (b + std::sqrt(b * b + 4.0 * k));
}

/// The jerk that drives acceleration from `from` towards `to`, or zero when
/// they already agree. Spelled out rather than written as nested conditionals,
/// which is where a sign error in a braking profile would hide best.
Scalar jerkToward(Scalar from, Scalar to, Scalar j_max) noexcept {
  if (to > from) {
    return j_max;
  }
  if (to < from) {
    return -j_max;
  }
  return 0.0;
}

}  // namespace

std::string_view toString(TrajectoryError error) noexcept {
  switch (error) {
    case TrajectoryError::None:
      return "none";
    case TrajectoryError::NonFiniteInput:
      return "a position or limit was NaN or infinite";
    case TrajectoryError::NonPositiveLimit:
      return "a velocity, acceleration or jerk limit was not positive";
    case TrajectoryError::AxisCountMismatch:
      return "start, goal and limits describe different numbers of axes";
    case TrajectoryError::TooManyAxes:
      return "more axes than kMaxAxes";
    case TrajectoryError::NoAxes:
      return "no axes";
  }
  return "unrecognised TrajectoryError";
}

// ---------------------------------------------------------------------------
// MotionLimits
// ---------------------------------------------------------------------------

TrajectoryError MotionLimits::validate() const noexcept {
  // Finiteness first. A NaN limit compares false against every bound, so the
  // positivity check below would let it through, and every comparison against
  // it downstream would then quietly succeed.
  if (!std::isfinite(max_velocity) || !std::isfinite(max_acceleration) ||
      !std::isfinite(max_jerk)) {
    return TrajectoryError::NonFiniteInput;
  }
  if (max_velocity <= 0.0 || max_acceleration <= 0.0 || max_jerk <= 0.0) {
    return TrajectoryError::NonPositiveLimit;
  }
  return TrajectoryError::None;
}

MotionLimits MotionLimits::scaled(Scalar factor) const noexcept {
  // A factor of zero or less produces limits that validate() rejects, rather
  // than a trajectory that takes forever or runs backwards.
  return MotionLimits{max_velocity * factor, max_acceleration * factor,
                      max_jerk * factor};
}

// ---------------------------------------------------------------------------
// ScurveProfile
// ---------------------------------------------------------------------------

Expected<ScurveProfile, TrajectoryError> ScurveProfile::plan(Scalar start, Scalar goal,
                                                             const MotionLimits& limits) {
  if (!std::isfinite(start) || !std::isfinite(goal)) {
    return {ScurveProfile{}, TrajectoryError::NonFiniteInput};
  }
  if (const TrajectoryError error = limits.validate(); error != TrajectoryError::None) {
    return {ScurveProfile{}, error};
  }

  const Scalar signed_distance = goal - start;
  // Both endpoints can be finite while their difference is not, at the extreme
  // ends of the range. Catch it here rather than propagating an infinity into
  // the profile.
  if (!std::isfinite(signed_distance)) {
    return {ScurveProfile{}, TrajectoryError::NonFiniteInput};
  }

  ScurveProfile profile;
  profile.start_ = start;
  profile.goal_ = goal;
  profile.direction_ = signed_distance < 0.0 ? -1.0 : 1.0;

  const Scalar distance = std::fabs(signed_distance);
  if (distance == 0.0) {
    // A valid profile of zero duration. Refusing this would force every caller
    // to special-case "already there", and one of them would forget.
    return {profile, TrajectoryError::None};
  }

  const Scalar v_max = limits.max_velocity;
  const Scalar a_max = limits.max_acceleration;
  const Scalar j_max = limits.max_jerk;

  // Ramping acceleration up to a_max and back down to zero already takes
  // 2 * (a_max / j_max) seconds and gains this much velocity on its own. Below
  // it there is no room for a constant-acceleration plateau, so the profile
  // never reaches a_max however long the move is.
  const Scalar plateau_velocity_threshold = (a_max / j_max) * a_max;

  // Duration of one ramp from rest to v. Both branches are exact; neither is an
  // approximation of the other.
  const auto rampDuration = [&](Scalar v) noexcept -> Scalar {
    return v >= plateau_velocity_threshold ? a_max / j_max + v / a_max
                                           : 2.0 * std::sqrt(v / j_max);
  };

  // The velocity curve of a ramp is odd-symmetric about its own midpoint, so
  // the distance a ramp to v covers is exactly v * duration / 2 -- no
  // integration needed, and true for both ramp shapes.
  const Scalar distance_to_reach_v_max = v_max * rampDuration(v_max);

  // Whether the move is long enough to reach v_max and hold it. Deciding this
  // once, here, is what keeps the cruise duration below from being computed as
  // the difference of two nearly equal numbers.
  const bool cruises = distance >= distance_to_reach_v_max;

  Scalar peak_velocity = v_max;
  if (!cruises) {
    // Too short to reach v_max: solve distance == v * rampDuration(v) for v.
    // Which branch of rampDuration applies is decided by the distance at which
    // the plateau vanishes, 2 * a_max^3 / j_max^2, grouped below to stay in
    // range for large limits.
    const Scalar plateau_distance_threshold =
        2.0 * (a_max / j_max) * (a_max / j_max) * a_max;
    if (distance >= plateau_distance_threshold) {
      // Plateau survives: v^2 + v * (a_max^2 / j_max) - distance * a_max == 0.
      peak_velocity = positiveQuadraticRoot(plateau_velocity_threshold, distance * a_max);
    } else {
      // No plateau: distance == 2 * v^(3/2) / sqrt(j_max).
      peak_velocity = std::cbrt(0.25 * distance * distance * j_max);
    }
  }

  Scalar jerk_duration = 0.0;
  Scalar plateau_duration = 0.0;
  Scalar peak_acceleration = 0.0;
  if (peak_velocity >= plateau_velocity_threshold) {
    jerk_duration = a_max / j_max;
    plateau_duration = peak_velocity / a_max - jerk_duration;
    peak_acceleration = a_max;
  } else {
    jerk_duration = std::sqrt(peak_velocity / j_max);
    peak_acceleration = j_max * jerk_duration;
  }
  // Right at the threshold the solve above can land a few ulp on the wrong
  // side. A negative duration would run that segment backwards, which is a much
  // worse outcome than a plateau one ulp short.
  plateau_duration = std::max(plateau_duration, 0.0);

  const Scalar ramp_duration = 2.0 * jerk_duration + plateau_duration;
  const Scalar ramp_distance = peak_velocity * ramp_duration * 0.5;

  // A move that never reaches v_max has no cruise, by construction rather than
  // by arithmetic. Computing it from the leftover distance anyway would ask for
  // zero as the difference of two numbers that agree to fifteen digits, and get
  // back something like 1e-17 seconds -- a segment of no duration and no
  // consequence, except that hasCruisePhase() would then say the move cruised.
  // Predicates get used for decisions; this one has to be right.
  const Scalar cruise_duration =
      cruises ? std::max((distance - 2.0 * ramp_distance) / peak_velocity, 0.0) : 0.0;

  const std::array<Scalar, kPhaseCount> durations{
      jerk_duration, plateau_duration, jerk_duration, cruise_duration,
      jerk_duration, plateau_duration, jerk_duration};
  // Jerk is j_max in every non-zero segment, in both profile shapes: with a
  // plateau because jerk_duration was chosen as a_max / j_max, and without one
  // because peak_acceleration was defined as j_max * jerk_duration.
  const std::array<Scalar, kPhaseCount> jerks{j_max,  0.0, -j_max, 0.0,
                                              -j_max, 0.0, j_max};

  profile.segments_.build(durations, jerks, /*v0=*/0.0, /*a0=*/0.0);
  profile.duration_ = profile.segments_.duration();
  profile.peak_velocity_ = peak_velocity;
  profile.peak_acceleration_ = peak_acceleration;
  return {profile, TrajectoryError::None};
}

MotionSample ScurveProfile::sample(Scalar t) const noexcept {
  MotionSample out;
  // Written as !(t > 0) rather than t <= 0 so that a NaN time parks at the
  // start instead of falling through to the segment scan and producing NaN
  // setpoints for the servo.
  if (!(t > 0.0)) {
    out.position = start_;
    return out;
  }
  if (t >= duration_) {
    out.position = goal_;
    return out;
  }

  // The run is built on the magnitude of the displacement; it is mirrored here
  // so there is one construction rather than two.
  const MotionSample relative = segments_.sampleInterior(t);
  out.position = start_ + direction_ * relative.position;
  out.velocity = direction_ * relative.velocity;
  out.acceleration = direction_ * relative.acceleration;
  out.jerk = direction_ * relative.jerk;
  return out;
}

std::array<Scalar, kPhaseCount> ScurveProfile::phaseDurations() const noexcept {
  std::array<Scalar, kPhaseCount> out{};
  for (std::size_t i = 0; i < kPhaseCount; ++i) {
    out[i] = segments_.segmentDuration(i);
  }
  return out;
}

bool ScurveProfile::hasCruisePhase() const noexcept {
  return segments_.segmentDuration(3) > 0.0;
}

bool ScurveProfile::hasAccelerationPlateau() const noexcept {
  return segments_.segmentDuration(1) > 0.0;
}

// ---------------------------------------------------------------------------
// StopProfile
// ---------------------------------------------------------------------------

Expected<StopProfile, TrajectoryError> StopProfile::plan(const MotionState& from,
                                                         const MotionLimits& limits) {
  if (!std::isfinite(from.position) || !std::isfinite(from.velocity) ||
      !std::isfinite(from.acceleration)) {
    return {StopProfile{}, TrajectoryError::NonFiniteInput};
  }
  if (const TrajectoryError error = limits.validate(); error != TrajectoryError::None) {
    return {StopProfile{}, error};
  }

  const Scalar a_max = limits.max_acceleration;
  const Scalar j_max = limits.max_jerk;
  const Scalar v0 = from.velocity;
  const Scalar a0 = from.acceleration;

  StopProfile stop;
  stop.start_ = from;
  stop.started_outside_limit_ = std::fabs(a0) > a_max;

  // The speed the axis will still carry once acceleration has been brought back
  // to zero. Acceleration cannot be changed instantaneously, so this much of
  // the motion is already committed however hard the brake is applied, and its
  // sign decides the whole shape of the stop: positive means the axis is still
  // running forward and has to be braked, negative means the axis is braking so
  // hard that it would reverse, and settling at rest means easing off instead.
  const Scalar committed_velocity = v0 + a0 * std::fabs(a0) / (2.0 * j_max);

  Scalar a_peak = 0.0;
  Scalar hold_duration = 0.0;
  if (committed_velocity > 0.0) {
    // Solving v(T) == 0 with a(T) == 0 for the two-ramp shape gives
    // a_peak^2 == j * v0 + a0^2 / 2. The max() is for rounding only: the
    // argument is positive whenever committed_velocity is.
    const Scalar square = std::max(j_max * v0 + 0.5 * a0 * a0, 0.0);
    a_peak = -std::sqrt(square);
    if (a_peak < -a_max) {
      a_peak = -a_max;
      hold_duration = (square - a_max * a_max) / (j_max * a_max);
    }
  } else if (committed_velocity < 0.0) {
    // The mirror image: the axis would overshoot backwards, so the profile
    // pushes forward to arrive at rest rather than past it.
    const Scalar square = std::max(0.5 * a0 * a0 - j_max * v0, 0.0);
    a_peak = std::sqrt(square);
    if (a_peak > a_max) {
      a_peak = a_max;
      hold_duration = (square - a_max * a_max) / (j_max * a_max);
    }
  }
  // committed_velocity == 0 leaves a_peak at zero: the axis is already on the
  // trajectory that brings it to rest, and all that remains is to unwind the
  // acceleration it has.

  const Scalar first_jerk = jerkToward(a0, a_peak, j_max);
  const Scalar last_jerk = jerkToward(a_peak, 0.0, j_max);
  const std::array<Scalar, kStopPhaseCount> durations{std::fabs(a_peak - a0) / j_max,
                                                      std::max(hold_duration, 0.0),
                                                      std::fabs(a_peak) / j_max};
  const std::array<Scalar, kStopPhaseCount> jerks{first_jerk, 0.0, last_jerk};

  stop.segments_.build(durations, jerks, v0, a0);
  stop.duration_ = stop.segments_.duration();
  stop.rest_position_ = from.position + stop.segments_.terminal().position;
  stop.initial_jerk_ = first_jerk;
  stop.peak_acceleration_ = std::max(std::fabs(a0), std::fabs(a_peak));
  // Velocity is extremal either at the start or where acceleration crosses
  // zero, and the value at that crossing is exactly committed_velocity.
  stop.peak_speed_ = std::max(std::fabs(v0), std::fabs(committed_velocity));
  return {stop, TrajectoryError::None};
}

MotionSample StopProfile::sample(Scalar t) const noexcept {
  // !(t > 0) rather than t <= 0, so a NaN time reports the state handed in
  // instead of falling through and producing NaN setpoints.
  if (!(t > 0.0)) {
    return MotionSample{start_.position, start_.velocity, start_.acceleration,
                        initial_jerk_};
  }
  if (t >= duration_) {
    return MotionSample{rest_position_, 0.0, 0.0, 0.0};
  }
  const MotionSample relative = segments_.sampleInterior(t);
  return MotionSample{start_.position + relative.position, relative.velocity,
                      relative.acceleration, relative.jerk};
}

Expected<Scalar, TrajectoryError> maximumSafeSpeed(Scalar available_distance,
                                                   const MotionLimits& limits) {
  if (!std::isfinite(available_distance)) {
    return {0.0, TrajectoryError::NonFiniteInput};
  }
  if (const TrajectoryError error = limits.validate(); error != TrajectoryError::None) {
    return {0.0, error};
  }
  if (available_distance <= 0.0) {
    // No room to stop in permits no speed. Not an error -- a guard flush
    // against the hazard is a legitimate thing to describe, and zero is the
    // correct answer for it.
    return {0.0, TrajectoryError::None};
  }

  const Scalar a_max = limits.max_acceleration;
  const Scalar j_max = limits.max_jerk;
  // Below this distance the stop is over before the acceleration limit is
  // reached, so the jerk limit alone decides it. Grouped to stay in range.
  const Scalar plateau_distance_threshold = (a_max / j_max) * (a_max / j_max) * a_max;

  Scalar speed = 0.0;
  if (available_distance < plateau_distance_threshold) {
    // distance == v^(3/2) / sqrt(j)
    speed = std::cbrt(available_distance * available_distance * j_max);
  } else {
    // distance == v * a / (2j) + v^2 / (2a), i.e.
    // v^2 + v * (a^2 / j) - 2 * distance * a == 0.
    speed =
        positiveQuadraticRoot((a_max / j_max) * a_max, 2.0 * available_distance * a_max);
  }
  return {std::min(speed, limits.max_velocity), TrajectoryError::None};
}

// ---------------------------------------------------------------------------
// SynchronizedTrajectory
// ---------------------------------------------------------------------------

Expected<SynchronizedTrajectory, TrajectoryError> SynchronizedTrajectory::plan(
    std::span<const Scalar> start, std::span<const Scalar> goal,
    std::span<const MotionLimits> limits) {
  if (start.size() != goal.size() || start.size() != limits.size()) {
    return {SynchronizedTrajectory{}, TrajectoryError::AxisCountMismatch};
  }
  if (start.empty()) {
    return {SynchronizedTrajectory{}, TrajectoryError::NoAxes};
  }
  if (start.size() > kMaxAxes) {
    return {SynchronizedTrajectory{}, TrajectoryError::TooManyAxes};
  }

  SynchronizedTrajectory trajectory;
  trajectory.axis_count_ = start.size();
  // Reads as "no axis binds the pace" until one does, which is also the honest
  // answer for a move in which nothing travels.
  trajectory.velocity_binding_axis_ = start.size();

  MotionLimits path_limits{};
  bool any_axis_moves = false;

  for (std::size_t i = 0; i < start.size(); ++i) {
    if (const TrajectoryError error = limits[i].validate();
        error != TrajectoryError::None) {
      return {SynchronizedTrajectory{}, error};
    }
    if (!std::isfinite(start[i]) || !std::isfinite(goal[i])) {
      return {SynchronizedTrajectory{}, TrajectoryError::NonFiniteInput};
    }
    const Scalar displacement = goal[i] - start[i];
    if (!std::isfinite(displacement)) {
      return {SynchronizedTrajectory{}, TrajectoryError::NonFiniteInput};
    }
    trajectory.start_[i] = start[i];
    trajectory.displacement_[i] = displacement;

    const Scalar travel = std::fabs(displacement);
    if (travel == 0.0) {
      // An axis that stays put constrains nothing, and consulting it would mean
      // dividing its limits by zero -- which would hand back infinities that
      // then bind nothing, or, if the limit were zero too, a NaN that binds
      // everything.
      continue;
    }
    const MotionLimits axis{limits[i].max_velocity / travel,
                            limits[i].max_acceleration / travel,
                            limits[i].max_jerk / travel};
    if (!any_axis_moves) {
      path_limits = axis;
      trajectory.velocity_binding_axis_ = i;
      any_axis_moves = true;
      continue;
    }
    if (axis.max_velocity < path_limits.max_velocity) {
      path_limits.max_velocity = axis.max_velocity;
      trajectory.velocity_binding_axis_ = i;
    }
    path_limits.max_acceleration =
        std::min(path_limits.max_acceleration, axis.max_acceleration);
    path_limits.max_jerk = std::min(path_limits.max_jerk, axis.max_jerk);
  }

  if (!any_axis_moves) {
    // Every axis is already where it should be. The default path profile has
    // zero duration and sits at s = 0, so sampling returns the start positions
    // for all time.
    return {trajectory, TrajectoryError::None};
  }

  const auto path = ScurveProfile::plan(0.0, 1.0, path_limits);
  if (!path) {
    // Reachable when a displacement is small enough that dividing a limit by it
    // overflows to infinity. Better a refusal naming the reason than a profile
    // built on one.
    return {SynchronizedTrajectory{}, path.error};
  }
  trajectory.path_ = path.value;
  return {trajectory, TrajectoryError::None};
}

bool SynchronizedTrajectory::sample(Scalar t,
                                    std::span<MotionSample> out) const noexcept {
  if (out.size() < axis_count_) {
    return false;
  }
  const MotionSample s = path_.sample(t);
  for (std::size_t i = 0; i < axis_count_; ++i) {
    const Scalar displacement = displacement_[i];
    out[i].position = start_[i] + displacement * s.position;
    out[i].velocity = displacement * s.velocity;
    out[i].acceleration = displacement * s.acceleration;
    out[i].jerk = displacement * s.jerk;
  }
  return true;
}

}  // namespace motionkit
