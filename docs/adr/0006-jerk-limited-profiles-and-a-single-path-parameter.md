# ADR-0006: Jerk-limited profiles, planned rest-to-rest, synchronised through one path parameter

- **Status**: Accepted
- **Date**: 2026-09-02
- **Deciders**: Onur Can Urhan

## Context

WP-05 needs point-to-point motion for a six-axis arm: a client asks for a joint
configuration, and the runtime has to produce a position setpoint for every axis
on every cycle of a 1 kHz task until the machine is there.

Three questions had to be answered, and each has an obvious answer that is
wrong in a way that only shows up on the machine.

## Decision 1: jerk-limited, not trapezoidal

A trapezoidal velocity profile steps acceleration discontinuously at three
points in every move. The drive cannot follow a step, so what the machine
actually does is ring at whatever frequency the structure happens to have — the
part of a cycle time that gets spent waiting for the tool to stop moving after
the axes have stopped.

Bounding jerk removes the steps. It costs time in principle and usually saves it
in practice, because settling time comes out of the same budget.

The profile is seven segments of constant jerk: `+J, 0, -J, 0, -J, 0, +J`. Short
moves lose the cruise, shorter ones lose the acceleration plateaus too and never
reach `max_acceleration` at all. All three shapes come out of one construction
with the unused segments at zero duration, so there is one code path rather than
three, and sampling costs the same wherever in the profile it lands.

### Consequence: no threshold decides the shape

Which shape a move gets is decided by closed-form comparisons against the
distance at which each phase vanishes (`2 * a³ / j²` for the plateau), not by an
epsilon. The first version computed the cruise duration as leftover distance
divided by peak velocity in every case, which for a short move asks for zero as
the difference between two numbers that agree to fifteen digits. It got back
about 1e-17 seconds — a segment with no duration and no numerical consequence,
except that `hasCruisePhase()` then reported that the move cruised. The
arithmetic was harmless; the predicate was not, and predicates are what callers
branch on.

## Decision 2: rest-to-rest only, and said so

`ScurveProfile` refuses to pretend it handles non-zero initial velocity.

Planning from a moving state is a substantially harder problem: the profile
stops being symmetric, deceleration can have to begin before acceleration has
finished, and there are states from which the goal is only reachable by
overshooting it and coming back. Ruckig solves that problem, and solving it
properly is a project rather than a work package.

The alternative — accepting an initial velocity and handling the easy cases —
would produce a class that works on every test anyone writes early and fails on
the machine during a blend. A stated limit is worth more than an unstated one.

## Decision 3: one path parameter, not one profile per axis

Every axis has to start and stop with the others. The obvious construction plans
each axis on its own limits and stretches the quick ones until the durations
match; time-scaling a rest-to-rest profile divides velocity by the factor,
acceleration by its square and jerk by its cube, so nothing exceeds its limits
and every axis does finish together.

It is still wrong. Each axis keeps its own profile *shape* — one has an
acceleration plateau, another does not — so the ratios between axis positions
vary through the move and the machine bows away from the straight line between
its endpoints.

`SynchronizedTrajectory` instead plans a single scalar `s` running 0 to 1 and
drives every axis from it:

    q_i(t) = q_i(0) + dq_i * s(t)

The ratios are constant by construction, so the joint-space path is exactly
straight. Each axis constrains the derivatives of `s` by its own limits divided
by its own displacement, and `s` is planned against the tightest of those, so
whichever axis binds each limit runs exactly at it and the rest run below.

`SynchronizedTrajectory.IndependentPlansStayInsideLimitsAndStillBowThePath`
measures the difference on a two-axis move: **13.6 mm** of excursion off the
straight line, from a plan in which no axis ever exceeds a limit. The
path-parameter version measures 1.1e-16.

Axes that do not move are skipped rather than scaled. Consulting one would mean
dividing its limits by a zero displacement, which yields infinities that
constrain nothing — or, if its limits were also zero, a NaN that constrains
everything.

## Decision 4: unset limits mean the axis may not move

`MotionLimits` default-constructs to zeros and `validate()` rejects zeros.

The alternative reading — an unset limit means no limit — makes forgetting to
configure an axis indistinguishable from configuring it for full speed, and the
difference is only observable on the machine. Finiteness is checked before
positivity for the same reason: a NaN limit passes a `<= 0` test, and then
passes every bound check downstream too, because comparisons against NaN are
false however they are written. It would be a limit nothing ever violates.

## Consequences

- Sampling is O(1), `noexcept` and allocation-free: 4.4 ns for one axis, 8.6 ns
  for six on an ordinary desktop.
- Planning is allocation-free as well, at 86 ns for a six-axis move, which makes
  a mid-move re-plan — a feed-rate override, a new goal — something the cyclic
  task can do itself instead of handing to another thread.
- Profiles must be **sampled, not integrated**. Forward Euler at 1 kHz lags the
  commanded position by half a step of velocity — 1.0 mm on a 2 m/s move — and
  then, because a rest-to-rest profile accelerates and decelerates by equal
  amounts, the error cancels to *exactly zero* by the end. An acceptance test
  that checks the final position passes while the machine was in the wrong place
  for the whole move. `TrajectorySampling.EulerIntegrationLagsMidMoveThenLandsOnTargetAnyway`
  measures both halves of that.
- Blending between moves is not addressed. Each move stops before the next
  begins, which is correct and slow. Blending needs the non-zero-boundary
  planner from Decision 2 and belongs with it.
