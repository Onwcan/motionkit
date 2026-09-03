// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string_view>

#include "motionkit/core/detail/jerk_segments.hpp"
#include "motionkit/core/expected.hpp"
#include "motionkit/core/motion_state.hpp"
#include "motionkit/core/types.hpp"

namespace motionkit {

/// Number of axes a SynchronizedTrajectory can carry.
///
/// Fixed for the same reason kMaxFrameDepth is fixed: a trajectory is sampled
/// from a control loop, and a control loop cannot call the allocator. Eight
/// covers a six-axis arm with a two-axis positioner, which is the machine this
/// library is aimed at.
inline constexpr std::size_t kMaxAxes = 8;

/// Segments of a jerk-limited profile: three to accelerate, one to cruise,
/// three to decelerate. Segments a particular move does not need are present
/// with zero duration rather than absent, so the shape of the profile never
/// changes and neither does the cost of sampling it.
inline constexpr std::size_t kPhaseCount = 7;

/// Why a trajectory could not be planned.
enum class TrajectoryError : std::uint8_t {
  None = 0,
  /// A position or a limit was NaN or infinite. Refused rather than propagated:
  /// a NaN limit compares false against every bound, so every check downstream
  /// would pass.
  NonFiniteInput,
  /// A velocity, acceleration or jerk limit was zero or negative. Zero is the
  /// interesting case -- see MotionLimits.
  NonPositiveLimit,
  /// start, goal and limits did not describe the same number of axes.
  AxisCountMismatch,
  /// More axes than kMaxAxes.
  TooManyAxes,
  /// Zero axes. There is no such thing as a trajectory for nothing.
  NoAxes,
};

std::string_view toString(TrajectoryError error) noexcept;

/// Per-axis kinematic ceilings.
///
/// All three default to zero, and validate() rejects zero. That is the point:
/// a default-constructed MotionLimits means "nobody has said what this axis can
/// do", and the only safe reading of silence is that the axis may not move. The
/// alternative -- treating an unset limit as unlimited -- makes forgetting to
/// configure an axis indistinguishable from configuring it for full speed, and
/// the difference only shows up on the machine.
struct MotionLimits {
  /// Maximum absolute velocity, in position units per second.
  Scalar max_velocity{0.0};
  /// Maximum absolute acceleration, in position units per second squared.
  Scalar max_acceleration{0.0};
  /// Maximum absolute jerk, in position units per second cubed. This is the
  /// limit that distinguishes an S-curve from a trapezoid, and the one that
  /// decides whether the mechanics ring.
  Scalar max_jerk{0.0};

  [[nodiscard]] TrajectoryError validate() const noexcept;

  /// Every limit scaled by `factor` in (0, 1] -- the usual feed-rate override.
  ///
  /// Note that this is not the same as stretching a planned profile in time: it
  /// re-plans against smaller ceilings, so the shape changes too.
  [[nodiscard]] MotionLimits scaled(Scalar factor) const noexcept;
};

/// A time-optimal rest-to-rest move along one axis, limited in velocity,
/// acceleration and jerk.
///
/// Scope
/// -----
/// **Rest to rest only.** Both endpoints have zero velocity and zero
/// acceleration. Planning from a non-zero initial state is a genuinely harder
/// problem -- the profile stops being symmetric, deceleration can have to begin
/// before acceleration finishes, and there are states from which the goal is
/// only reachable by overshooting it first. Ruckig solves that problem
/// properly; this class solves the one industrial point-to-point motion
/// actually poses, and says so rather than implying the general case is covered.
///
/// Shape
/// -----
/// Seven segments of constant jerk: `+J, 0, -J, 0, -J, 0, +J`. The middle
/// segment is the constant-velocity cruise; the two zero-jerk segments beside
/// it are constant-acceleration plateaus. A short move loses the cruise; a
/// short move against a high acceleration limit loses the plateaus as well and
/// never reaches max_acceleration. All three shapes come out of one
/// construction, with the unused segments at zero duration.
///
/// Sampling
/// --------
/// sample() evaluates the closed form for the segment containing `t`, starting
/// from state stored at that segment's entry. It is O(1), allocation-free and
/// noexcept, so it is callable from the cyclic task.
///
/// **Sample it; do not integrate it.** Feeding velocity into an integrator at
/// the control rate accumulates truncation error that closed-form sampling does
/// not have. At 1 kHz forward Euler lands visibly short of the goal; the figure
/// is measured in TrajectorySampling.EulerIntegrationDriftsWhereSamplingDoesNot.
class ScurveProfile {
 public:
  /// A zero-duration profile parked at position zero.
  constexpr ScurveProfile() noexcept = default;

  /// Plans the fastest move from `start` to `goal` that respects `limits`.
  ///
  /// `goal` may be less than `start`; the profile is mirrored. `goal == start`
  /// plans a valid profile of zero duration.
  static Expected<ScurveProfile, TrajectoryError> plan(Scalar start, Scalar goal,
                                                       const MotionLimits& limits);

  [[nodiscard]] constexpr Scalar duration() const noexcept { return duration_; }
  [[nodiscard]] constexpr Scalar startPosition() const noexcept { return start_; }
  [[nodiscard]] constexpr Scalar goalPosition() const noexcept { return goal_; }

  /// State at time `t`, clamped to [0, duration()].
  ///
  /// Before the start the profile holds the start state; at or after duration()
  /// it holds the goal exactly, with zero velocity, acceleration and jerk. That
  /// last part is a definition rather than an evaluation: the closed form
  /// reaches the goal only to within rounding, and a caller comparing the final
  /// sample against the commanded goal should get the goal.
  [[nodiscard]] MotionSample sample(Scalar t) const noexcept;

  /// Highest absolute velocity the profile reaches. Below max_velocity when the
  /// move is too short to get there.
  [[nodiscard]] constexpr Scalar peakVelocity() const noexcept { return peak_velocity_; }

  /// Highest absolute acceleration the profile reaches. Below max_acceleration
  /// when the move is too short for the plateau to exist.
  [[nodiscard]] constexpr Scalar peakAcceleration() const noexcept {
    return peak_acceleration_;
  }

  /// Durations of the seven segments, in order. Zeros mark the segments this
  /// move does not need, which is how a test names the shape it got.
  [[nodiscard]] std::array<Scalar, kPhaseCount> phaseDurations() const noexcept;

  /// True when the move is long enough to reach max_velocity and hold it.
  [[nodiscard]] bool hasCruisePhase() const noexcept;

  /// True when the move is long enough to reach max_acceleration.
  [[nodiscard]] bool hasAccelerationPlateau() const noexcept;

 private:
  detail::JerkSegments<kPhaseCount> segments_{};
  Scalar duration_{0.0};
  Scalar start_{0.0};
  Scalar goal_{0.0};
  /// +1 or -1. The profile is built on the magnitude of the displacement and
  /// mirrored on the way out, so there is one construction rather than two.
  Scalar direction_{1.0};
  Scalar peak_velocity_{0.0};
  Scalar peak_acceleration_{0.0};
};

/// Segments in a stop: ramp acceleration to the braking level, hold it there,
/// ramp back to zero as the axis arrives at rest.
inline constexpr std::size_t kStopPhaseCount = 3;

/// The fastest stop from an arbitrary state that respects the acceleration and
/// jerk limits.
///
/// Unlike ScurveProfile this takes a moving, accelerating axis: it is the half
/// of the non-zero-initial-state problem that has a closed form. There is no
/// position target -- the axis stops where the limits let it, and where that is
/// is the question worth asking.
///
/// limits.max_velocity is not consulted. A stop cannot be made safer by
/// refusing to start one, and the axis is already travelling at whatever speed
/// it is travelling at; peakSpeed() reports the speed actually reached, which
/// can exceed max_velocity when the axis was still accelerating when the stop
/// was called for.
///
/// A state whose acceleration already exceeds the limit is planned, not
/// refused, and startedOutsideAccelerationLimit() says so. Declining to stop a
/// machine on the grounds that it is already misbehaving has the logic exactly
/// backwards.
class StopProfile {
 public:
  constexpr StopProfile() noexcept = default;

  static Expected<StopProfile, TrajectoryError> plan(const MotionState& from,
                                                     const MotionLimits& limits);

  [[nodiscard]] constexpr Scalar duration() const noexcept { return duration_; }

  /// State at `t`. Before the stop begins this is the state handed in, with the
  /// jerk the stop is about to command; at or after duration() the axis is at
  /// rest, exactly.
  [[nodiscard]] MotionSample sample(Scalar t) const noexcept;

  /// Signed travel between the starting position and rest. Negative for an axis
  /// moving in the negative direction; it can also have the opposite sign to
  /// the initial velocity, when a large opposing acceleration reverses the axis
  /// before it settles.
  [[nodiscard]] constexpr Scalar stoppingDistance() const noexcept {
    return rest_position_ - start_.position;
  }

  [[nodiscard]] constexpr Scalar restPosition() const noexcept { return rest_position_; }

  /// Largest speed reached during the stop. Equal to the initial speed when the
  /// axis was already slowing, and larger when it was not: acceleration cannot
  /// be changed instantaneously, so some further speed is already committed.
  [[nodiscard]] constexpr Scalar peakSpeed() const noexcept { return peak_speed_; }

  /// Largest absolute acceleration during the stop, including the initial value
  /// when that already exceeded the limit.
  [[nodiscard]] constexpr Scalar peakAcceleration() const noexcept {
    return peak_acceleration_;
  }

  [[nodiscard]] constexpr bool startedOutsideAccelerationLimit() const noexcept {
    return started_outside_limit_;
  }

 private:
  detail::JerkSegments<kStopPhaseCount> segments_{};
  MotionState start_{};
  Scalar duration_{0.0};
  Scalar rest_position_{0.0};
  Scalar peak_speed_{0.0};
  Scalar peak_acceleration_{0.0};
  Scalar initial_jerk_{0.0};
  bool started_outside_limit_{false};
};

/// The highest speed at which an axis running at constant velocity may travel
/// and still be brought to rest within `available_distance`.
///
/// This is the number that sizes a guard: how far past a light curtain the tool
/// can still travel, and therefore how far back the curtain has to be. The
/// answer is never more than limits.max_velocity -- a speed the axis cannot
/// reach is not a useful permission -- and is zero when there is no room, which
/// is the honest reading of no room.
///
/// The familiar `v^2 / (2a)` is the answer for a trapezoidal stop, and it is
/// optimistic here by exactly `v * a_max / (2 * j_max)`: the time spent building
/// the braking force is time spent travelling. See
/// StopSafetyEnvelope.TrapezoidalFormulaUnderstatesTheStoppingDistance.
Expected<Scalar, TrajectoryError> maximumSafeSpeed(Scalar available_distance,
                                                   const MotionLimits& limits);

/// A multi-axis move in which every axis starts and stops at the same instant
/// and the machine travels a straight line in joint space.
///
/// How, and why not the obvious way
/// --------------------------------
/// The obvious construction plans each axis independently and stretches the
/// quick ones to match the slowest. It does make them finish together, and the
/// path it produces is still wrong: each axis follows its own profile shape, so
/// the ratios between axis positions vary through the move and the machine bows
/// away from the straight line between its endpoints. On a six-axis arm that is
/// a tool-centre-point excursion nobody asked for.
///
/// This class instead plans a single scalar path parameter `s` running 0 to 1
/// and drives every axis from it:
///
///     q_i(t) = q_i(0) + dq_i * s(t)
///
/// The ratios are then constant by construction, so the joint-space path is
/// exactly straight and the axes are synchronised for free. Each axis
/// constrains the derivatives of `s`:
///
///     ds/dt   <= v_i / |dq_i|
///     d2s/dt2 <= a_i / |dq_i|
///     d3s/dt3 <= j_i / |dq_i|
///
/// so `s` is planned against the tightest of those across the axes. Whichever
/// axis binds each limit then runs exactly at it and the rest run below, which
/// is what time-optimal means once the path is fixed. Axes that do not move
/// impose no constraint and are not consulted -- dividing by their zero
/// displacement would otherwise make them bind everything at once.
class SynchronizedTrajectory {
 public:
  constexpr SynchronizedTrajectory() noexcept = default;

  /// All three spans must have the same size, between 1 and kMaxAxes.
  static Expected<SynchronizedTrajectory, TrajectoryError> plan(
      std::span<const Scalar> start, std::span<const Scalar> goal,
      std::span<const MotionLimits> limits);

  [[nodiscard]] constexpr std::size_t axisCount() const noexcept { return axis_count_; }
  [[nodiscard]] Scalar duration() const noexcept { return path_.duration(); }

  /// Writes axisCount() samples into `out`, one per axis, in the order given to
  /// plan(). Returns false without writing anything when `out` is too small --
  /// the only failure available to a noexcept, allocation-free hot path, and
  /// therefore one that must not be discarded by accident.
  [[nodiscard]] bool sample(Scalar t, std::span<MotionSample> out) const noexcept;

  /// The profile of the path parameter itself, on [0, 1]. Its peak velocity is
  /// in path fractions per second, not in the units of any axis.
  [[nodiscard]] const ScurveProfile& pathProfile() const noexcept { return path_; }

  /// Index of the axis whose velocity limit set the pace, or axisCount() when
  /// nothing moves. Worth logging: on a disappointing cycle time it names the
  /// axis to argue with.
  [[nodiscard]] constexpr std::size_t velocityBindingAxis() const noexcept {
    return velocity_binding_axis_;
  }

 private:
  ScurveProfile path_;
  std::array<Scalar, kMaxAxes> start_{};
  std::array<Scalar, kMaxAxes> displacement_{};
  std::size_t axis_count_{0};
  std::size_t velocity_binding_axis_{0};
};

}  // namespace motionkit
