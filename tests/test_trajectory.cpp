// SPDX-License-Identifier: Apache-2.0
//
// Checks the properties a jerk-limited profile is bought for: that the limits
// are never exceeded anywhere in the move rather than merely at the peaks the
// planner believes in, that the profile arrives, and that the multi-axis form
// holds a straight line in joint space.

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <limits>
#include <span>
#include <vector>

#include "motionkit/core/trajectory.hpp"

namespace motionkit {
namespace {

constexpr MotionLimits kArmAxis{/*max_velocity=*/2.0, /*max_acceleration=*/8.0,
                                /*max_jerk=*/40.0};

/// Worst absolute velocity, acceleration and jerk seen anywhere in a profile.
///
/// The planner reports the peaks it intends to reach. This walks the profile
/// and reports the peaks it actually produces, which is the only version worth
/// asserting against: a planner that miscomputed a phase duration would still
/// report the intended peak quite happily.
struct Extremes {
  Scalar velocity{0.0};
  Scalar acceleration{0.0};
  Scalar jerk{0.0};
};

Extremes scan(const ScurveProfile& profile, std::size_t steps) {
  Extremes worst;
  for (std::size_t i = 0; i <= steps; ++i) {
    const Scalar t =
        profile.duration() * static_cast<Scalar>(i) / static_cast<Scalar>(steps);
    const MotionSample s = profile.sample(t);
    worst.velocity = std::max(worst.velocity, std::fabs(s.velocity));
    worst.acceleration = std::max(worst.acceleration, std::fabs(s.acceleration));
    worst.jerk = std::max(worst.jerk, std::fabs(s.jerk));
  }
  return worst;
}

ScurveProfile mustPlan(Scalar start, Scalar goal, const MotionLimits& limits) {
  const auto planned = ScurveProfile::plan(start, goal, limits);
  EXPECT_EQ(planned.error, TrajectoryError::None) << toString(planned.error);
  return planned.value;
}

}  // namespace

// ---------------------------------------------------------------------------
// Refusals
// ---------------------------------------------------------------------------
namespace {

// The whole reason MotionLimits defaults to zeros. An unconfigured axis must be
// an axis that cannot move, not an axis with no ceiling.
TEST(TrajectoryLimits, DefaultConstructedLimitsAreRefused) {
  const MotionLimits unset{};
  EXPECT_EQ(unset.validate(), TrajectoryError::NonPositiveLimit);

  const auto planned = ScurveProfile::plan(0.0, 1.0, unset);
  EXPECT_FALSE(planned.hasValue());
  EXPECT_EQ(planned.error, TrajectoryError::NonPositiveLimit);
  EXPECT_EQ(planned.value.duration(), 0.0);
}

TEST(TrajectoryLimits, NaNLimitsAreRefusedRatherThanPropagated) {
  const Scalar nan = std::numeric_limits<Scalar>::quiet_NaN();
  MotionLimits limits = kArmAxis;
  limits.max_velocity = nan;
  EXPECT_EQ(limits.validate(), TrajectoryError::NonFiniteInput);

  // Why finiteness is checked before positivity: a NaN limit passes the
  // positivity test, and then passes every downstream bound check too, because
  // a comparison against NaN is false whichever way it is written. It would be
  // a limit that nothing ever violates.
  EXPECT_FALSE(nan <= 0.0);
  EXPECT_FALSE(1e9 > nan);
}

TEST(TrajectoryLimits, InfiniteLimitsAreRefused) {
  MotionLimits limits = kArmAxis;
  limits.max_jerk = std::numeric_limits<Scalar>::infinity();
  EXPECT_EQ(limits.validate(), TrajectoryError::NonFiniteInput);
}

TEST(TrajectoryLimits, ScalingToZeroYieldsLimitsThatAreRefused) {
  // A feed-rate override wound down to zero should stop the machine by refusing
  // to plan, not by planning a move of infinite duration.
  EXPECT_EQ(kArmAxis.scaled(0.0).validate(), TrajectoryError::NonPositiveLimit);
  EXPECT_EQ(kArmAxis.scaled(-0.5).validate(), TrajectoryError::NonPositiveLimit);
  EXPECT_EQ(kArmAxis.scaled(0.5).validate(), TrajectoryError::None);
}

TEST(TrajectoryLimits, NonFinitePositionsAreRefused) {
  const Scalar inf = std::numeric_limits<Scalar>::infinity();
  EXPECT_EQ(ScurveProfile::plan(0.0, inf, kArmAxis).error,
            TrajectoryError::NonFiniteInput);
  EXPECT_EQ(
      ScurveProfile::plan(std::numeric_limits<Scalar>::quiet_NaN(), 1.0, kArmAxis).error,
      TrajectoryError::NonFiniteInput);
  // Finite endpoints whose difference is not.
  const Scalar huge = std::numeric_limits<Scalar>::max();
  EXPECT_EQ(ScurveProfile::plan(-huge, huge, kArmAxis).error,
            TrajectoryError::NonFiniteInput);
}

}  // namespace

// ---------------------------------------------------------------------------
// The three profile shapes
// ---------------------------------------------------------------------------
namespace {

TEST(TrajectoryShape, LongMoveReachesEveryLimitAndCruises) {
  const ScurveProfile profile = mustPlan(0.0, 2.0, kArmAxis);
  EXPECT_TRUE(profile.hasCruisePhase());
  EXPECT_TRUE(profile.hasAccelerationPlateau());
  EXPECT_DOUBLE_EQ(profile.peakVelocity(), kArmAxis.max_velocity);
  EXPECT_DOUBLE_EQ(profile.peakAcceleration(), kArmAxis.max_acceleration);

  // The seven segments, spelled out: jerk up, hold acceleration, jerk down,
  // cruise, and the mirror image of the first three.
  const std::array<Scalar, kPhaseCount> d = profile.phaseDurations();
  EXPECT_NEAR(d[0], 0.2, 1e-12);   // a_max / j_max
  EXPECT_NEAR(d[1], 0.05, 1e-12);  // v_max / a_max - a_max / j_max
  EXPECT_NEAR(d[2], 0.2, 1e-12);
  EXPECT_GT(d[3], 0.0);
  EXPECT_NEAR(d[4], d[2], 1e-12);
  EXPECT_NEAR(d[5], d[1], 1e-12);
  EXPECT_NEAR(d[6], d[0], 1e-12);
}

TEST(TrajectoryShape, MediumMoveLosesTheCruiseButKeepsThePlateau) {
  // Shorter than the 0.9 m it takes to reach v_max and come back to rest, but
  // longer than the 0.64 m below which the acceleration plateau vanishes.
  const ScurveProfile profile = mustPlan(0.0, 0.75, kArmAxis);
  EXPECT_FALSE(profile.hasCruisePhase());
  EXPECT_TRUE(profile.hasAccelerationPlateau());
  EXPECT_LT(profile.peakVelocity(), kArmAxis.max_velocity);
  EXPECT_DOUBLE_EQ(profile.peakAcceleration(), kArmAxis.max_acceleration);
}

TEST(TrajectoryShape, ShortMoveLosesThePlateauAsWell) {
  const ScurveProfile profile = mustPlan(0.0, 0.1, kArmAxis);
  EXPECT_FALSE(profile.hasCruisePhase());
  EXPECT_FALSE(profile.hasAccelerationPlateau());
  EXPECT_LT(profile.peakVelocity(), kArmAxis.max_velocity);
  EXPECT_LT(profile.peakAcceleration(), kArmAxis.max_acceleration);
  // The jerk limit is still met exactly -- it is the only one left binding.
  EXPECT_NEAR(scan(profile, 4000).jerk, kArmAxis.max_jerk, 1e-9);
}

TEST(TrajectoryShape, ZeroDistanceIsAValidZeroDurationProfile) {
  const ScurveProfile profile = mustPlan(0.7, 0.7, kArmAxis);
  EXPECT_EQ(profile.duration(), 0.0);
  const MotionSample s = profile.sample(0.0);
  EXPECT_DOUBLE_EQ(s.position, 0.7);
  EXPECT_DOUBLE_EQ(s.velocity, 0.0);
  // Sampling past the end of a zero-duration profile still parks at the goal.
  EXPECT_DOUBLE_EQ(profile.sample(5.0).position, 0.7);
}

}  // namespace

// ---------------------------------------------------------------------------
// Invariants across the whole move
// ---------------------------------------------------------------------------
namespace {

// The claim a jerk-limited planner exists to make. Checking the peaks the
// planner reports would be circular; this walks the profile.
TEST(TrajectoryInvariants, NoLimitIsExceededAnywhereInAnyShape) {
  struct Case {
    const char* name;
    Scalar distance;
    MotionLimits limits;
  };
  const std::array<Case, 6> cases{{
      {"long", 2.0, kArmAxis},
      {"medium", 0.75, kArmAxis},
      {"short", 0.1, kArmAxis},
      {"tiny", 1e-6, kArmAxis},
      {"stiff jerk", 1.0, MotionLimits{2.0, 8.0, 1.0}},
      {"loose jerk", 1.0, MotionLimits{2.0, 8.0, 1e6}},
  }};

  for (const Case& c : cases) {
    const ScurveProfile profile = mustPlan(0.0, c.distance, c.limits);
    const Extremes worst = scan(profile, 20000);
    // A relative slack of 1e-9 is rounding in the closed form, not headroom:
    // the profile is built to touch its limits, so an exact comparison would
    // fail on the last bit.
    EXPECT_LE(worst.velocity, c.limits.max_velocity * (1.0 + 1e-9)) << c.name;
    EXPECT_LE(worst.acceleration, c.limits.max_acceleration * (1.0 + 1e-9)) << c.name;
    EXPECT_LE(worst.jerk, c.limits.max_jerk * (1.0 + 1e-9)) << c.name;

    // And the peaks the planner reports are the peaks it delivers, to the
    // resolution of the scan.
    EXPECT_NEAR(worst.velocity, profile.peakVelocity(), profile.peakVelocity() * 1e-4)
        << c.name;
    EXPECT_NEAR(worst.acceleration, profile.peakAcceleration(),
                profile.peakAcceleration() * 1e-4)
        << c.name;
  }
}

TEST(TrajectoryInvariants, BothEndpointsAreAtRest) {
  const ScurveProfile profile = mustPlan(0.0, 2.0, kArmAxis);

  const MotionSample begin = profile.sample(0.0);
  EXPECT_DOUBLE_EQ(begin.velocity, 0.0);
  EXPECT_DOUBLE_EQ(begin.acceleration, 0.0);

  const MotionSample end = profile.sample(profile.duration());
  EXPECT_DOUBLE_EQ(end.velocity, 0.0);
  EXPECT_DOUBLE_EQ(end.acceleration, 0.0);
  EXPECT_DOUBLE_EQ(end.position, 2.0);

  // Just short of the end the closed form has to agree with that definition,
  // or the last cycle before completion would command a jump.
  const MotionSample almost = profile.sample(profile.duration() - 1e-9);
  EXPECT_NEAR(almost.position, 2.0, 1e-8);
  EXPECT_NEAR(almost.velocity, 0.0, 1e-7);
}

}  // namespace

namespace {

// The residual the class documentation promises to hide behind the definition
// of sample(duration()). Measured rather than assumed: if the phase
// accumulation were wrong, this is where it would show.
TEST(TrajectoryInvariants, ClosedFormReachesTheGoalToNearMachinePrecision) {
  Scalar worst_residual = 0.0;
  for (int i = 1; i <= 60; ++i) {
    const Scalar distance = 0.01 * static_cast<Scalar>(i);
    const ScurveProfile profile = mustPlan(0.0, distance, kArmAxis);
    // One ulp of the duration short of the end, so the closed form is
    // evaluated rather than the terminal definition returned.
    const Scalar t = std::nextafter(profile.duration(), 0.0);
    worst_residual =
        std::max(worst_residual, std::fabs(profile.sample(t).position - distance));
  }
  std::printf("  worst closed-form endpoint residual over 60 moves: %.3e m\n",
              worst_residual);
  EXPECT_LT(worst_residual, 1e-14);
}

TEST(TrajectoryInvariants, PositionIsMonotonicWhenTheMoveIsMonotonic) {
  const ScurveProfile profile = mustPlan(0.0, 2.0, kArmAxis);
  Scalar previous = -1.0;
  for (std::size_t i = 0; i <= 20000; ++i) {
    const Scalar t = profile.duration() * static_cast<Scalar>(i) / 20000.0;
    const Scalar p = profile.sample(t).position;
    ASSERT_GE(p, previous) << "position went backwards at t = " << t;
    previous = p;
  }
}

TEST(TrajectoryInvariants, ReverseMoveIsTheMirrorImage) {
  const ScurveProfile forward = mustPlan(0.0, 1.5, kArmAxis);
  const ScurveProfile backward = mustPlan(1.5, 0.0, kArmAxis);
  EXPECT_DOUBLE_EQ(forward.duration(), backward.duration());

  for (std::size_t i = 0; i <= 500; ++i) {
    const Scalar t = forward.duration() * static_cast<Scalar>(i) / 500.0;
    const MotionSample f = forward.sample(t);
    const MotionSample b = backward.sample(t);
    EXPECT_NEAR(b.position, 1.5 - f.position, 1e-12);
    EXPECT_NEAR(b.velocity, -f.velocity, 1e-12);
    EXPECT_NEAR(b.acceleration, -f.acceleration, 1e-12);
    EXPECT_NEAR(b.jerk, -f.jerk, 1e-12);
  }
}

}  // namespace

// ---------------------------------------------------------------------------
// Internal consistency and how not to consume a profile
// ---------------------------------------------------------------------------
namespace {

TEST(TrajectorySampling, VelocityAndAccelerationAreTheDerivativesTheyClaimToBe) {
  const ScurveProfile profile = mustPlan(0.0, 2.0, kArmAxis);
  constexpr Scalar h = 1e-6;

  // Acceleration is only continuous, not differentiable: jerk steps at every
  // phase boundary. A central difference straddling one of those returns the
  // average of the two sides and would fail for a correct profile, so the
  // boundaries are stepped around rather than tested through.
  std::vector<Scalar> boundaries;
  Scalar edge = 0.0;
  for (const Scalar d : profile.phaseDurations()) {
    edge += d;
    boundaries.push_back(edge);
  }
  const auto nearBoundary = [&](Scalar t) {
    for (const Scalar b : boundaries) {
      if (std::fabs(t - b) < 4.0 * h) {
        return true;
      }
    }
    return false;
  };

  std::size_t checked = 0;
  for (std::size_t i = 1; i < 2000; ++i) {
    const Scalar t = profile.duration() * static_cast<Scalar>(i) / 2000.0;
    if (nearBoundary(t)) {
      continue;
    }
    const MotionSample before = profile.sample(t - h);
    const MotionSample after = profile.sample(t + h);
    const MotionSample here = profile.sample(t);

    EXPECT_NEAR((after.position - before.position) / (2.0 * h), here.velocity, 1e-8)
        << "at t = " << t;
    EXPECT_NEAR((after.velocity - before.velocity) / (2.0 * h), here.acceleration, 1e-6)
        << "at t = " << t;
    ++checked;
  }
  EXPECT_GT(checked, 1500u) << "too many samples skipped to be meaningful";
}

}  // namespace

namespace {

// Why the header says to sample the profile rather than integrate it.
//
// The interesting part is not that forward Euler is inaccurate. It is *where*:
// the per-step error is proportional to acceleration, and a rest-to-rest
// profile accelerates and decelerates by equal amounts, so the errors cancel
// almost exactly by the end. An acceptance test that checks only the final
// position passes. The machine is still in the wrong place for the entire move.
TEST(TrajectorySampling, EulerIntegrationLagsMidMoveThenLandsOnTargetAnyway) {
  const ScurveProfile profile = mustPlan(0.0, 2.0, kArmAxis);
  constexpr Scalar dt = 1e-3;  // a 1 kHz cyclic task
  const auto steps = static_cast<std::size_t>(profile.duration() / dt);

  Scalar integrated = 0.0;
  Scalar worst_lag = 0.0;
  for (std::size_t k = 0; k < steps; ++k) {
    const Scalar t = static_cast<Scalar>(k) * dt;
    const MotionSample s = profile.sample(t);
    worst_lag = std::max(worst_lag, std::fabs(integrated - s.position));
    integrated += s.velocity * dt;
  }
  const Scalar final_error =
      std::fabs(integrated - profile.sample(static_cast<Scalar>(steps) * dt).position);

  // The lag is predictable, not merely present: each step drops half a step of
  // the velocity change, so the deficit tracks (dt / 2) * v(t) and peaks where
  // the profile is fastest.
  const Scalar predicted = 0.5 * dt * profile.peakVelocity();
  std::printf(
      "  1 kHz forward Euler: worst mid-move lag %.3e m (predicted %.3e m),\n"
      "                       error at the end     %.3e m\n",
      worst_lag, predicted, final_error);

  EXPECT_NEAR(worst_lag, predicted, predicted * 0.2);
  // Two orders of magnitude between the two, which is the whole point.
  EXPECT_LT(final_error, worst_lag * 0.01);
}

}  // namespace

// ---------------------------------------------------------------------------
// Optimality, in the only sense the class claims
// ---------------------------------------------------------------------------
namespace {

TEST(TrajectoryOptimality, ExtraDistanceCostsExactlyCruiseTimeAndNothingElse) {
  const ScurveProfile shorter = mustPlan(0.0, 2.0, kArmAxis);
  const ScurveProfile longer = mustPlan(0.0, 3.0, kArmAxis);

  // Both moves reach v_max, so their acceleration and deceleration ramps are
  // identical and the whole difference is spent cruising. Any extra time would
  // be time the planner wasted.
  EXPECT_NEAR(longer.duration() - shorter.duration(), 1.0 / kArmAxis.max_velocity, 1e-12);

  // And no move can beat the velocity limit alone.
  EXPECT_GE(shorter.duration(), 2.0 / kArmAxis.max_velocity);
}

TEST(TrajectoryOptimality, LooserLimitsAreNeverSlower) {
  Scalar previous = std::numeric_limits<Scalar>::infinity();
  for (const Scalar factor : {1.0, 2.0, 4.0, 16.0}) {
    MotionLimits limits = kArmAxis;
    limits.max_jerk *= factor;
    const Scalar duration = mustPlan(0.0, 0.3, limits).duration();
    EXPECT_LE(duration, previous) << "raising the jerk limit made the move slower";
    previous = duration;
  }
}

}  // namespace

// ---------------------------------------------------------------------------
// Multi-axis synchronisation
// ---------------------------------------------------------------------------
namespace {

/// A fast long axis, a slow short one, and one that is already where it belongs
/// and carries limits far too small to move anywhere.
struct Cell {
  std::array<Scalar, 3> start{0.0, 0.0, 0.5};
  std::array<Scalar, 3> goal{1.0, 0.2, 0.5};
  std::array<MotionLimits, 3> limits{MotionLimits{1.0, 4.0, 20.0},
                                     MotionLimits{1.0, 4.0, 20.0},
                                     MotionLimits{0.01, 0.01, 0.01}};
};

SynchronizedTrajectory mustPlanCell(const Cell& cell) {
  const auto planned = SynchronizedTrajectory::plan(cell.start, cell.goal, cell.limits);
  EXPECT_EQ(planned.error, TrajectoryError::None) << toString(planned.error);
  return planned.value;
}

TEST(SynchronizedTrajectory, MismatchedSpansAreRefused) {
  const std::array<Scalar, 2> start{0.0, 0.0};
  const std::array<Scalar, 3> goal{1.0, 1.0, 1.0};
  const std::array<MotionLimits, 2> limits{kArmAxis, kArmAxis};
  EXPECT_EQ(SynchronizedTrajectory::plan(start, goal, limits).error,
            TrajectoryError::AxisCountMismatch);
}

TEST(SynchronizedTrajectory, EmptyAndOversizedAreRefused) {
  EXPECT_EQ(SynchronizedTrajectory::plan({}, {}, {}).error, TrajectoryError::NoAxes);

  std::array<Scalar, kMaxAxes + 1> start{};
  std::array<Scalar, kMaxAxes + 1> goal{};
  std::array<MotionLimits, kMaxAxes + 1> limits{};
  limits.fill(kArmAxis);
  goal.fill(1.0);
  EXPECT_EQ(SynchronizedTrajectory::plan(start, goal, limits).error,
            TrajectoryError::TooManyAxes);
}

TEST(SynchronizedTrajectory, SampleRefusesAnUndersizedBuffer) {
  const SynchronizedTrajectory trajectory = mustPlanCell(Cell{});
  std::array<MotionSample, 2> too_small{};
  EXPECT_FALSE(trajectory.sample(0.1, too_small));
  std::array<MotionSample, 3> big_enough{};
  EXPECT_TRUE(trajectory.sample(0.1, big_enough));
}

}  // namespace

namespace {

TEST(SynchronizedTrajectory, EveryAxisStartsAndStopsTogetherAndArrives) {
  const Cell cell;
  const SynchronizedTrajectory trajectory = mustPlanCell(cell);
  ASSERT_GT(trajectory.duration(), 0.0);

  std::array<MotionSample, 3> at_start{};
  std::array<MotionSample, 3> at_end{};
  ASSERT_TRUE(trajectory.sample(0.0, at_start));
  ASSERT_TRUE(trajectory.sample(trajectory.duration(), at_end));

  for (std::size_t i = 0; i < 3; ++i) {
    EXPECT_DOUBLE_EQ(at_start[i].position, cell.start[i]) << "axis " << i;
    EXPECT_DOUBLE_EQ(at_start[i].velocity, 0.0) << "axis " << i;
    EXPECT_DOUBLE_EQ(at_start[i].acceleration, 0.0) << "axis " << i;
    // The arrival position is start + displacement rather than the goal
    // literally, so it is exact only to a rounding of the larger magnitude.
    EXPECT_NEAR(at_end[i].position, cell.goal[i], 1e-15) << "axis " << i;
    EXPECT_DOUBLE_EQ(at_end[i].velocity, 0.0) << "axis " << i;
    EXPECT_DOUBLE_EQ(at_end[i].acceleration, 0.0) << "axis " << i;
  }
}

TEST(SynchronizedTrajectory, EveryAxisRespectsItsOwnLimitsThroughout) {
  const Cell cell;
  const SynchronizedTrajectory trajectory = mustPlanCell(cell);
  std::array<Scalar, 3> worst_v{};
  std::array<Scalar, 3> worst_a{};
  std::array<Scalar, 3> worst_j{};

  std::array<MotionSample, 3> out{};
  for (std::size_t k = 0; k <= 20000; ++k) {
    const Scalar t = trajectory.duration() * static_cast<Scalar>(k) / 20000.0;
    ASSERT_TRUE(trajectory.sample(t, out));
    for (std::size_t i = 0; i < 3; ++i) {
      worst_v[i] = std::max(worst_v[i], std::fabs(out[i].velocity));
      worst_a[i] = std::max(worst_a[i], std::fabs(out[i].acceleration));
      worst_j[i] = std::max(worst_j[i], std::fabs(out[i].jerk));
    }
  }
  for (std::size_t i = 0; i < 3; ++i) {
    EXPECT_LE(worst_v[i], cell.limits[i].max_velocity * (1.0 + 1e-9)) << "axis " << i;
    EXPECT_LE(worst_a[i], cell.limits[i].max_acceleration * (1.0 + 1e-9)) << "axis " << i;
    EXPECT_LE(worst_j[i], cell.limits[i].max_jerk * (1.0 + 1e-9)) << "axis " << i;
  }

  // The axis the planner names as binding is the one actually running at its
  // ceiling; the others have headroom. That is what time-optimal means once the
  // path is fixed -- something has to be saturated.
  ASSERT_EQ(trajectory.velocityBindingAxis(), 0u);
  EXPECT_NEAR(worst_v[0], cell.limits[0].max_velocity, 1e-6);
  EXPECT_LT(worst_v[1], cell.limits[1].max_velocity * 0.5);
}

}  // namespace

namespace {

// The property the whole path-parameter design exists to provide.
TEST(SynchronizedTrajectory, JointSpacePathIsStraight) {
  const Cell cell;
  const SynchronizedTrajectory trajectory = mustPlanCell(cell);

  std::array<MotionSample, 3> out{};
  Scalar worst_spread = 0.0;
  for (std::size_t k = 0; k <= 5000; ++k) {
    const Scalar t = trajectory.duration() * static_cast<Scalar>(k) / 5000.0;
    ASSERT_TRUE(trajectory.sample(t, out));
    // Fraction of its own journey each moving axis has completed. On a straight
    // line these are the same number.
    const Scalar progress_0 = (out[0].position - cell.start[0]) / 1.0;
    const Scalar progress_1 = (out[1].position - cell.start[1]) / 0.2;
    worst_spread = std::max(worst_spread, std::fabs(progress_0 - progress_1));
  }
  std::printf("  synchronised: worst progress spread between axes %.3e\n", worst_spread);
  // Not a tolerance so much as a rounding budget: both axes are the same scalar
  // multiplied by different constants.
  EXPECT_LT(worst_spread, 1e-14);
}

// The comparison that justifies the previous test, run on equal terms: plan each
// axis on its own limits, then stretch the quicker one in time until the two
// durations match. That does synchronise the endpoints, and every axis stays
// inside its limits -- so it looks right from every check except the shape of
// the path.
TEST(SynchronizedTrajectory, IndependentPlansStayInsideLimitsAndStillBowThePath) {
  const Cell cell;
  const ScurveProfile axis0 = mustPlan(0.0, 1.0, cell.limits[0]);
  const ScurveProfile axis1 = mustPlan(0.0, 0.2, cell.limits[1]);
  ASSERT_LT(axis1.duration(), axis0.duration());

  // Time-scaling a rest-to-rest profile by more than one divides velocity by the
  // factor, acceleration by its square and jerk by its cube, so a stretched
  // profile cannot violate a limit its unstretched form respected.
  const Scalar stretch = axis1.duration() / axis0.duration();

  Scalar worst_spread = 0.0;
  for (std::size_t k = 0; k <= 5000; ++k) {
    const Scalar t = axis0.duration() * static_cast<Scalar>(k) / 5000.0;
    const Scalar progress_0 = axis0.sample(t).position / 1.0;
    const Scalar progress_1 = axis1.sample(t * stretch).position / 0.2;
    worst_spread = std::max(worst_spread, std::fabs(progress_0 - progress_1));
  }
  // Expressed where it would be measured: how far the short axis sits from
  // where the straight line says it should be.
  std::printf(
      "  independent + stretched: worst progress spread %.3e,\n"
      "                           axis-1 excursion off the line %.3e m\n",
      worst_spread, worst_spread * 0.2);

  EXPECT_GT(worst_spread, 1e-3) << "the bow should be obvious, not marginal";
}

}  // namespace

namespace {

// A stationary axis has no scale on which to express its limits, so dividing by
// its displacement would either produce infinities that bind nothing or, if its
// limits were zero too, a NaN that binds everything. It is skipped instead.
TEST(SynchronizedTrajectory, StationaryAxesDoNotSlowTheMoveDown) {
  const Cell with_parked;
  const SynchronizedTrajectory slow_third_axis = mustPlanCell(with_parked);

  Cell without_it;
  // Same two moving axes, and the parked one given limits so small that if it
  // were consulted at all the move would take hours.
  const std::array<Scalar, 2> start{0.0, 0.0};
  const std::array<Scalar, 2> goal{1.0, 0.2};
  const std::array<MotionLimits, 2> limits{without_it.limits[0], without_it.limits[1]};
  const auto two_axis = SynchronizedTrajectory::plan(start, goal, limits);
  ASSERT_TRUE(two_axis.hasValue()) << toString(two_axis.error);

  EXPECT_DOUBLE_EQ(slow_third_axis.duration(), two_axis.value.duration());
}

TEST(SynchronizedTrajectory, NothingToDoIsAZeroDurationTrajectory) {
  const std::array<Scalar, 2> start{0.3, -1.2};
  const std::array<MotionLimits, 2> limits{kArmAxis, kArmAxis};
  const auto planned = SynchronizedTrajectory::plan(start, start, limits);
  ASSERT_TRUE(planned.hasValue()) << toString(planned.error);
  EXPECT_EQ(planned.value.duration(), 0.0);
  EXPECT_EQ(planned.value.velocityBindingAxis(), planned.value.axisCount())
      << "no axis can be binding when nothing moves";

  std::array<MotionSample, 2> out{};
  ASSERT_TRUE(planned.value.sample(3.0, out));
  EXPECT_DOUBLE_EQ(out[0].position, 0.3);
  EXPECT_DOUBLE_EQ(out[1].position, -1.2);
  EXPECT_DOUBLE_EQ(out[0].velocity, 0.0);
}

TEST(SynchronizedTrajectory, OneBadAxisRefusesTheWholeMove) {
  const std::array<Scalar, 2> start{0.0, 0.0};
  const std::array<Scalar, 2> goal{1.0, 1.0};
  const std::array<MotionLimits, 2> limits{kArmAxis, MotionLimits{}};
  const auto planned = SynchronizedTrajectory::plan(start, goal, limits);
  // Not "plan the axis we understand and hope" -- the axes are one motion.
  EXPECT_EQ(planned.error, TrajectoryError::NonPositiveLimit);
  EXPECT_EQ(planned.value.axisCount(), 0u);
}

}  // namespace

// ---------------------------------------------------------------------------
// Stopping from an arbitrary state
// ---------------------------------------------------------------------------
namespace {

StopProfile mustStop(const MotionState& from, const MotionLimits& limits) {
  const auto planned = StopProfile::plan(from, limits);
  EXPECT_EQ(planned.error, TrajectoryError::None) << toString(planned.error);
  return planned.value;
}

/// Worst absolute acceleration and jerk, and the extreme velocity, over a stop.
struct StopExtremes {
  Scalar speed{0.0};
  Scalar acceleration{0.0};
  Scalar jerk{0.0};
  Scalar most_negative_velocity{0.0};
  Scalar least_position{0.0};
  Scalar most_position{0.0};
};

StopExtremes scanStop(const StopProfile& stop, std::size_t steps) {
  StopExtremes worst;
  for (std::size_t i = 0; i <= steps; ++i) {
    const Scalar t =
        stop.duration() * static_cast<Scalar>(i) / static_cast<Scalar>(steps);
    const MotionSample s = stop.sample(t);
    worst.speed = std::max(worst.speed, std::fabs(s.velocity));
    worst.acceleration = std::max(worst.acceleration, std::fabs(s.acceleration));
    worst.jerk = std::max(worst.jerk, std::fabs(s.jerk));
    worst.most_negative_velocity = std::min(worst.most_negative_velocity, s.velocity);
    worst.least_position = std::min(worst.least_position, s.position);
    worst.most_position = std::max(worst.most_position, s.position);
  }
  return worst;
}

TEST(StopProfile, StopFromConstantVelocityComesToRestExactly) {
  const StopProfile stop = mustStop(MotionState{0.0, 2.0, 0.0}, kArmAxis);
  ASSERT_GT(stop.duration(), 0.0);

  const MotionSample end = stop.sample(stop.duration());
  EXPECT_DOUBLE_EQ(end.velocity, 0.0);
  EXPECT_DOUBLE_EQ(end.acceleration, 0.0);
  EXPECT_DOUBLE_EQ(end.position, stop.restPosition());

  // Independent check on the distance. During a stop with symmetric jerk the
  // velocity curve is odd-symmetric about its own midpoint, so the distance is
  // exactly v * T / 2 -- a different derivation from the segment accumulation
  // the profile actually uses.
  EXPECT_NEAR(stop.stoppingDistance(), 2.0 * stop.duration() * 0.5, 1e-12);
  EXPECT_DOUBLE_EQ(stop.peakSpeed(), 2.0);
}

}  // namespace

namespace {

TEST(StopProfile, LimitsAreRespectedThroughoutEveryStop) {
  struct Case {
    const char* name;
    MotionState from;
  };
  const std::array<Case, 6> cases{{
      {"cruising", MotionState{0.0, 2.0, 0.0}},
      {"cruising backwards", MotionState{0.0, -2.0, 0.0}},
      {"still accelerating", MotionState{0.0, 1.5, 8.0}},
      {"already braking", MotionState{0.0, 1.5, -8.0}},
      {"at rest but accelerating", MotionState{0.0, 0.0, 8.0}},
      {"crawling", MotionState{0.0, 1e-4, 0.0}},
  }};

  for (const Case& c : cases) {
    const StopProfile stop = mustStop(c.from, kArmAxis);
    const StopExtremes worst = scanStop(stop, 20000);
    EXPECT_LE(worst.acceleration, kArmAxis.max_acceleration * (1.0 + 1e-9)) << c.name;
    EXPECT_LE(worst.jerk, kArmAxis.max_jerk * (1.0 + 1e-9)) << c.name;
    EXPECT_NEAR(worst.speed, stop.peakSpeed(), stop.peakSpeed() * 1e-4 + 1e-12) << c.name;
    EXPECT_DOUBLE_EQ(stop.sample(stop.duration()).velocity, 0.0) << c.name;
    EXPECT_DOUBLE_EQ(stop.sample(stop.duration()).acceleration, 0.0) << c.name;
  }
}

TEST(StopProfile, StopIsMirrorSymmetricInDirection) {
  const StopProfile forward = mustStop(MotionState{0.0, 2.0, 3.0}, kArmAxis);
  const StopProfile backward = mustStop(MotionState{0.0, -2.0, -3.0}, kArmAxis);
  EXPECT_DOUBLE_EQ(forward.duration(), backward.duration());
  EXPECT_NEAR(backward.stoppingDistance(), -forward.stoppingDistance(), 1e-15);
  EXPECT_DOUBLE_EQ(forward.peakSpeed(), backward.peakSpeed());
}

TEST(StopProfile, StoppingFromRestTakesNoTimeAndNoDistance) {
  const StopProfile stop = mustStop(MotionState{0.7, 0.0, 0.0}, kArmAxis);
  EXPECT_EQ(stop.duration(), 0.0);
  EXPECT_EQ(stop.stoppingDistance(), 0.0);
  EXPECT_DOUBLE_EQ(stop.restPosition(), 0.7);
  EXPECT_DOUBLE_EQ(stop.sample(5.0).position, 0.7);
}

TEST(StopProfile, SampleBeforeTheStopReportsTheStateHandedIn) {
  const MotionState from{0.3, 1.5, 4.0};
  const StopProfile stop = mustStop(from, kArmAxis);
  const MotionSample before = stop.sample(-1.0);
  EXPECT_DOUBLE_EQ(before.position, from.position);
  EXPECT_DOUBLE_EQ(before.velocity, from.velocity);
  EXPECT_DOUBLE_EQ(before.acceleration, from.acceleration);
  // The axis is accelerating in the direction of travel, so the stop's first
  // command is maximum negative jerk.
  EXPECT_DOUBLE_EQ(before.jerk, -kArmAxis.max_jerk);
}

TEST(StopProfile, NonFiniteStateIsRefused) {
  const Scalar nan = std::numeric_limits<Scalar>::quiet_NaN();
  EXPECT_EQ(StopProfile::plan(MotionState{0.0, nan, 0.0}, kArmAxis).error,
            TrajectoryError::NonFiniteInput);
  EXPECT_EQ(StopProfile::plan(MotionState{0.0, 1.0, 0.0}, MotionLimits{}).error,
            TrajectoryError::NonPositiveLimit);
}

}  // namespace

namespace {

// Acceleration cannot be changed instantaneously, so an axis that is still
// accelerating when the stop is called for has already committed to more speed
// than it currently has. This is the part of a stopping distance that intuition
// leaves out.
TEST(StopProfile, AnAxisStillAcceleratingCommitsToSpeedItDoesNotYetHave) {
  const StopProfile steady = mustStop(MotionState{0.0, 1.5, 0.0}, kArmAxis);
  const StopProfile climbing = mustStop(MotionState{0.0, 1.5, 8.0}, kArmAxis);

  EXPECT_GT(climbing.peakSpeed(), 1.5);
  EXPECT_GT(climbing.stoppingDistance(), steady.stoppingDistance());
  std::printf(
      "  from 1.5 m/s at rest-acceleration: peak %.3f m/s, stops in %.1f mm\n"
      "  from 1.5 m/s still at +8 m/s^2:    peak %.3f m/s, stops in %.1f mm\n",
      steady.peakSpeed(), steady.stoppingDistance() * 1000.0, climbing.peakSpeed(),
      climbing.stoppingDistance() * 1000.0);
}

// The counterintuitive case, and the reason a stop has to target zero
// acceleration rather than just zero velocity: an axis braking harder than the
// jerk limit can unwind will pass through zero velocity before the brake can be
// released, reverse, and settle from the other side.
TEST(StopProfile, BrakingHarderThanTheJerkLimitCanUnwindOvershootsAndComesBack) {
  const MotionState from{0.0, 0.05, -8.0};
  const StopProfile stop = mustStop(from, kArmAxis);
  const StopExtremes worst = scanStop(stop, 20000);

  // It does reverse. Reaching zero velocity is easy here; staying there is not,
  // and a profile that stopped at the first zero crossing would leave the axis
  // with -8 m/s^2 still applied and accelerating backwards.
  EXPECT_LT(worst.most_negative_velocity, 0.0);
  // And it ends up behind where it started, despite having been moving forward.
  EXPECT_LT(stop.stoppingDistance(), 0.0);
  EXPECT_DOUBLE_EQ(stop.sample(stop.duration()).velocity, 0.0);
  EXPECT_DOUBLE_EQ(stop.sample(stop.duration()).acceleration, 0.0);
  EXPECT_LE(worst.acceleration, kArmAxis.max_acceleration * (1.0 + 1e-9));

  std::printf(
      "  from +0.05 m/s at -8 m/s^2: reverses to %.4f m/s, settles %.2f mm behind\n",
      worst.most_negative_velocity, stop.stoppingDistance() * 1000.0);
}

TEST(StopProfile, AStateOutsideTheAccelerationLimitIsStoppedNotRefused) {
  // A drive that has overshot its own limit is exactly when a stop is wanted.
  const MotionState from{0.0, 1.0, -20.0};  // limit is 8
  const StopProfile stop = mustStop(from, kArmAxis);
  EXPECT_TRUE(stop.startedOutsideAccelerationLimit());
  EXPECT_NEAR(stop.peakAcceleration(), 20.0, 1e-12);
  EXPECT_DOUBLE_EQ(stop.sample(stop.duration()).velocity, 0.0);

  // Jerk is still honoured -- it is the one limit the profile can respect from
  // the first instant, and respecting it is how the acceleration gets back
  // inside its own bound at all.
  EXPECT_LE(scanStop(stop, 20000).jerk, kArmAxis.max_jerk * (1.0 + 1e-9));
  EXPECT_FALSE(
      mustStop(MotionState{0.0, 1.0, -8.0}, kArmAxis).startedOutsideAccelerationLimit());
}

}  // namespace

// ---------------------------------------------------------------------------
// The safety envelope
// ---------------------------------------------------------------------------
namespace {

TEST(StopSafetyEnvelope, TheSpeedItPermitsIsExactlyTheSpeedThatStopsInTime) {
  // Both sides of the same equation, computed by different routes: a closed
  // form solved for speed, against the segment accumulation of a planned stop.
  for (const Scalar room : {0.005, 0.05, 0.2, 0.45, 1.0}) {
    const auto permitted = maximumSafeSpeed(room, kArmAxis);
    ASSERT_TRUE(permitted.hasValue()) << toString(permitted.error);
    if (permitted.value >= kArmAxis.max_velocity) {
      continue;  // clamped by the velocity limit, checked separately below
    }
    const StopProfile stop = mustStop(MotionState{0.0, permitted.value, 0.0}, kArmAxis);
    EXPECT_NEAR(stop.stoppingDistance(), room, room * 1e-9)
        << "room = " << room << ", permitted = " << permitted.value;
  }
}

// The number a guard gets sized with, and how wrong the textbook version is.
TEST(StopSafetyEnvelope, TrapezoidalFormulaUnderstatesTheStoppingDistance) {
  constexpr Scalar speed = 2.0;
  const StopProfile stop = mustStop(MotionState{0.0, speed, 0.0}, kArmAxis);

  const Scalar trapezoidal = speed * speed / (2.0 * kArmAxis.max_acceleration);
  // The whole difference is the time spent building and releasing the braking
  // force, and it comes out to exactly v * a / (2j).
  const Scalar predicted_shortfall =
      speed * kArmAxis.max_acceleration / (2.0 * kArmAxis.max_jerk);

  std::printf(
      "  stopping from %.1f m/s at a=%.0f m/s^2, j=%.0f m/s^3:\n"
      "    v^2/(2a) says          %6.1f mm\n"
      "    jerk-limited stop is   %6.1f mm\n"
      "    shortfall              %6.1f mm  (predicted v*a/(2j) = %.1f mm)\n",
      speed, kArmAxis.max_acceleration, kArmAxis.max_jerk, trapezoidal * 1000.0,
      stop.stoppingDistance() * 1000.0, (stop.stoppingDistance() - trapezoidal) * 1000.0,
      predicted_shortfall * 1000.0);

  EXPECT_GT(stop.stoppingDistance(), trapezoidal);
  EXPECT_NEAR(stop.stoppingDistance() - trapezoidal, predicted_shortfall, 1e-12);
  // Not a rounding-level difference: at these limits the real stop is most of a
  // factor longer than the formula that ignores jerk.
  EXPECT_GT(stop.stoppingDistance(), trapezoidal * 1.5);
}

}  // namespace

namespace {

TEST(StopSafetyEnvelope, NoRoomPermitsNoSpeed) {
  // The theme: the absence of room is not permission to move. Zero is an
  // answer, not an error -- a guard flush against the hazard is a real thing to
  // describe, and the honest speed for it is zero.
  const auto flush = maximumSafeSpeed(0.0, kArmAxis);
  ASSERT_TRUE(flush.hasValue());
  EXPECT_EQ(flush.value, 0.0);

  const auto negative = maximumSafeSpeed(-0.1, kArmAxis);
  ASSERT_TRUE(negative.hasValue());
  EXPECT_EQ(negative.value, 0.0);
}

TEST(StopSafetyEnvelope, ThePermittedSpeedNeverExceedsTheVelocityLimit) {
  // Given a whole hangar to stop in, the answer is the axis limit rather than
  // the physics -- reporting a speed the axis cannot reach would be a
  // permission nobody can act on.
  const auto roomy = maximumSafeSpeed(1000.0, kArmAxis);
  ASSERT_TRUE(roomy.hasValue());
  EXPECT_DOUBLE_EQ(roomy.value, kArmAxis.max_velocity);
}

TEST(StopSafetyEnvelope, MorePermittedSpeedNeedsMoreRoomMonotonically) {
  Scalar previous = -1.0;
  for (std::size_t i = 1; i <= 200; ++i) {
    const Scalar room = 0.002 * static_cast<Scalar>(i);
    const auto permitted = maximumSafeSpeed(room, kArmAxis);
    ASSERT_TRUE(permitted.hasValue());
    EXPECT_GE(permitted.value, previous) << "more room permitted less speed";
    previous = permitted.value;
  }
}

TEST(StopSafetyEnvelope, BadInputsAreRefused) {
  EXPECT_EQ(maximumSafeSpeed(std::numeric_limits<Scalar>::quiet_NaN(), kArmAxis).error,
            TrajectoryError::NonFiniteInput);
  EXPECT_EQ(maximumSafeSpeed(1.0, MotionLimits{}).error,
            TrajectoryError::NonPositiveLimit);
}

}  // namespace
}  // namespace motionkit
