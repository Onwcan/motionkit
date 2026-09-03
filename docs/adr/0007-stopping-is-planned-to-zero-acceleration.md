# ADR-0007: A stop is planned to zero acceleration, and the envelope that follows from it

- **Status**: Accepted
- **Date**: 2026-09-03
- **Deciders**: Onur Can Urhan

## Context

ADR-0006 deferred planning from a non-zero initial state, on the grounds that
the general position-target version of it is a project rather than a work
package. One case of it is not: bringing a moving, accelerating axis to rest.
That one has a closed form, and it answers the question a cell layout actually
turns on — *how far does the tool travel after someone asks it to stop?*

## Decision 1: the target is (v = 0, a = 0), not v = 0

The obvious reading of "stop" is zero velocity. It is not enough, and the
counterexample is ordinary rather than contrived.

Take an axis at +0.05 m/s decelerating at −8 m/s², with a jerk limit of
40 m/s³. Velocity reaches zero almost immediately. Acceleration is still
−8 m/s² at that instant, and unwinding it takes 0.2 s no matter what, so the
axis carries straight on into reverse. The measured profile reverses to
**−0.75 m/s** and settles **199 mm behind where it started**.

So the terminal condition is a *state*, and reaching it can require the axis to
overshoot and come back. `StopProfile.BrakingHarderThanTheJerkLimitCanUnwindOvershootsAndComesBack`
pins that, including the sign of the net travel — `stoppingDistance()` can
oppose the initial velocity, which is a thing callers have to be told rather
than left to discover.

## Decision 2: the shape follows from one signed quantity

Everything about the profile is decided by the velocity the axis still carries
once acceleration has been brought back to zero:

    committed_velocity = v0 + a0 * |a0| / (2 * j)

Acceleration cannot change instantaneously, so this much of the motion is
already spent whatever the brake does. Its sign selects the whole construction:

- **positive** — the axis is still running forward and has to be braked, so the
  peak acceleration is negative: `a_peak² = j·v0 + a0²/2`
- **negative** — the axis is braking so hard it would reverse, so the profile
  pushes *forward* to arrive at rest instead of past it, mirrored:
  `a_peak² = a0²/2 − j·v0`
- **zero** — the axis is already on the trajectory that reaches rest, and all
  that is left is to unwind the acceleration it has

Clamping `a_peak` to `a_max` introduces the constant-acceleration hold, exactly
as the cruise appears in ADR-0006. Three segments cover every case.

The same quantity is also the extreme velocity of the stop whenever the initial
acceleration opposes the braking direction, which is why `peakSpeed()` is
`max(|v0|, |committed_velocity|)` and not a scan.

## Decision 3: an out-of-limit state is stopped, not refused

`plan()` accepts a state whose acceleration already exceeds `max_acceleration`
and reports it through `startedOutsideAccelerationLimit()`.

Declining to stop a machine because it is already misbehaving has the logic
backwards: that is the moment a stop is most wanted. Jerk is still honoured from
the first instant — it is the one limit the profile can respect immediately, and
respecting it is *how* the acceleration gets back inside its own bound.

`max_velocity` is not consulted at all. The axis is already travelling at
whatever speed it is travelling at, and a velocity ceiling cannot make a stop
safer; the honest thing is to report the speed reached and let the caller see it
exceed the limit if it does.

## Decision 4: no room to stop permits no speed

`maximumSafeSpeed(available_distance, limits)` inverts the stopping distance:
the highest constant speed from which the axis can still be brought to rest
inside a given distance. It returns **zero** for zero or negative room, as a
value rather than an error — a guard flush against the hazard is a real thing to
describe, and zero is the correct answer for it. It never returns more than
`max_velocity`, because a speed the axis cannot reach is not a useful
permission.

## Consequences

**The textbook formula is optimistic, by a term nobody carries.** `v²/(2a)` is
the trapezoidal stopping distance. The jerk-limited one is

    v·a/(2j) + v²/(2a)

so the shortfall is exactly `v·a/(2j)` — the time spent building and releasing
the braking force is time spent travelling. At 2 m/s with `a = 8 m/s²` and
`j = 40 m/s³`: the formula says **250 mm**, the real stop is **450 mm**, and the
missing **200 mm** is the term that got dropped. A guard positioned from
`v²/(2a)` sits 200 mm inside the hazard.
`StopSafetyEnvelope.TrapezoidalFormulaUnderstatesTheStoppingDistance` asserts
the closed form and the planned profile agree on it.

**A speed reading is not a stopping distance.** Two axes both reading 1.5 m/s
stop in **290 mm** and **967 mm** — a factor of 3.3 — because one of them is
still accelerating at 8 m/s². Any envelope computed from velocity alone is
wrong for every axis that is not already at constant speed, which during a move
is most of them.

**Planning a stop costs 17.5 ns and allocates nothing**, so the safety task can
plan its own rather than depend on a planner thread it does not control.

**Still deferred**: the general position-target problem from a non-zero state,
and the blending that needs it. This ADR takes the half with a closed form and
says which half that is.
