# ADR-0003: Use atan2, not acos, for the rotation metric

- **Status**: Accepted
- **Date**: 2026-08-25
- **Deciders**: Onur Can Urhan

## Context

`SO3::angleTo` returns the angle between two rotations. It is not a convenience
function: it is the metric underneath `isApprox`, and it will be the metric
underneath the numerical IK convergence check (WP-03), the calibration residual
(WP-06) and the servo error reported by the runtime (WP-08). Whatever accuracy
it has becomes a floor on all of them.

The textbook formula is

```
angle = 2 * acos(|q1 · q2|)
```

It is correct in exact arithmetic, it is what most tutorials show, and it is
wrong in floating point in a way that unit tests on large angles do not reveal.

## The measurement

This was found by a failing test, not by inspection. `SO3.MatrixRoundTrip`
converted 5000 uniformly sampled rotations to matrices and back and asserted
agreement to 1e-11. It failed. Instrumenting the same loop to report both the
componentwise quaternion difference and the reported angle gave:

```
worst |Δq|₁    = 1.249e-15     <- the conversion is exact to machine precision
worst angleTo  = 5.162e-08     <- the metric is not
```

Seven orders of magnitude apart. The conversion was never wrong; the ruler was.

## Why

`acos` has infinite derivative at 1:

```
d/dx acos(x) = -1 / sqrt(1 - x²)   →   ∞  as x → 1
```

Two nearly identical rotations have a dot product near 1, carrying the usual
~1e-16 of accumulated rounding. Propagating that through `acos` gives an angle
error of order `sqrt(2ε) ≈ 2.1e-8` rad. The error is *structural*: no amount of
care in computing the dot product improves it, because the conditioning of the
final step is what dominates.

Concretely, at a 400 mm reach, 5e-8 rad is about 20 µm of tool-point error —
the same order as the repeatability spec of the machines this code is meant to
drive. A convergence check that cannot distinguish "converged" from "20 µm out"
is not a convergence check.

## Decision

Compute the relative rotation `q_rel = q₁* · q₂` explicitly, then

```
angle = 2 * atan2(‖vec(q_rel)‖, |w(q_rel)|)
```

`atan2` is well conditioned across the whole range. Near identity the vector
part shrinks linearly with the angle, so the ratio tracks it down to machine
precision instead of losing half the mantissa. Taking `|w|` collapses the
double cover, so the result is the shorter arc and lands in `[0, π]` without a
separate branch.

The same reasoning was applied to `toRPY`, which now recovers pitch as
`atan2(-m₂₀, hypot(m₀₀, m₁₀))` rather than `asin(-m₂₀)`.

## Consequences

**Positive**

- Round-trip tests hold at 1e-13 instead of failing at 1e-11.
- The accuracy floor under IK convergence, calibration residuals and servo
  error is removed before any of those exist to inherit it.
- Cost is four extra multiplies and a square root versus a bare dot product.
  Irrelevant next to the `atan2` itself, which both forms pay.

**Negative**

- The implementation no longer matches the formula a reviewer expects to see,
  which is a real review cost. Paid down with a comment at the call site
  carrying the measured numbers, this ADR, and a regression test.

**Guarding it**

`SO3.AngleToStaysAccurateForVerySmallAngles` walks angles from 1e-3 down to
1e-13 and asserts relative accuracy of 1e-6. The `acos` implementation passes
every other test in the suite and fails this one by six orders of magnitude,
which is exactly the property a regression test should have.

## What this does not fix

Gimbal lock in the Z-Y-X decomposition is a separate and genuinely irreducible
limit. Near pitch = ±π/2, roll and yaw are read from matrix entries of size
`cos(pitch)` carrying absolute error ~1e-16, so their relative error is
`ε/cos(pitch)`; folding them together early instead costs `cos(pitch)·roll`.
The two terms cross near `√ε`, which is where the branch threshold sits and why
RPY round-trips through the singularity are accurate to ~1e-8 rad and no
better. That is a property of the parametrisation, not of this code — and it is
the reason ADR-0001 keeps Euler angles as an export format only.
