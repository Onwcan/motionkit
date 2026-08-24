# ADR-0001: Store rotations as canonical unit quaternions

- **Status**: Accepted
- **Date**: 2026-08-25
- **Deciders**: Onur Can Urhan
- **Supersedes**: —

## Context

Every layer above this one — forward kinematics, trajectory interpolation,
hand-eye calibration, the runtime execution loop — composes and interpolates
rotations continuously. A six-axis arm composes at least seven transforms per
forward-kinematics call, and a 1 kHz control loop does that a thousand times a
second. The internal representation is therefore a decision the entire stack
inherits, and changing it later means touching every consumer.

Four candidates were considered.

## Options

### A. 3×3 rotation matrix

Composition is a 27-multiply, 18-add matrix product. Rotating a vector is 9
multiplies. Debugging is pleasant because the columns are the images of the
basis vectors and can be eyeballed.

Repeated products drift off SO(3), and restoring orthonormality means
Gram-Schmidt or an SVD — expensive enough that implementations tend to skip it
and let the chain skew instead. Storage is 9 doubles for 3 degrees of freedom.

### B. Euler angles (Z-Y-X / RPY)

Minimal storage, and the only representation most operators and CAD tools will
read. Composition requires converting out and back. Fatally, the
parametrisation is singular: at pitch = ±π/2 roll and yaw collapse into one
degree of freedom, and accuracy near that point is bounded near `√ε` regardless
of implementation quality. Industrial tool-down poses sit *exactly* at
pitch = −π/2, so this is the common case rather than a corner case.

### C. Axis-angle / rotation vector

Minimal (3 doubles), and the natural space for interpolation and for the
Jacobian used by numerical IK. But composition has no closed form — you convert
to quaternion or matrix, multiply, and convert back — and the representation is
discontinuous at angle π.

### D. Unit quaternion

Composition is 16 multiplies, 12 adds. Rotating a vector via the Rodrigues form
is 15 multiplies, against 27 to materialise the matrix first. Renormalising is
one square root and four divides, cheap enough to do unconditionally on every
product. Interpolation (slerp) is closed-form and constant angular velocity.
Storage is 4 doubles.

The cost is the double cover: `q` and `−q` name the same rotation, so
componentwise comparison is meaningless and `log()` is two-valued unless the
sign is pinned.

## Decision

Store a unit quaternion in Hamilton convention with the real part first, and
enforce a two-part class invariant: `‖q‖ = 1` **and** `w ≥ 0`.

`operator*` renormalises on every call rather than deferring to the caller.
`fromMatrix`, `toRPY` and `rotationVector` remain available as conversions, so
matrices and Euler angles stay first-class at the boundary — for CAD interop,
operator-facing displays and vision stacks — without either becoming the
internal truth.

## Consequences

**Positive**

- Long kinematic chains stay on the manifold.
  `SO3.ChainOfManyProductsDoesNotDrift` composes 100 000 rotations and asserts
  the result is still in SO(3) to 1e-12.
- Pinning `w ≥ 0` makes `rotationVector()` single-valued, makes `angleTo`
  always return the shorter arc without a separate branch, and makes two `SO3`
  values comparable componentwise.
- Slerp is exact and closed-form, which WP-05 needs for Cartesian orientation
  blending.

**Negative**

- Renormalising on every product costs a `sqrt` and four divides that a
  matrix-based implementation would not pay per-composition. Measured against
  the alternative — an arm that silently skews after an hour of operation —
  this is not a close call, but it is a real cost and WP-08 will need to
  confirm it fits the 1 kHz budget.
- Quaternions are harder to inspect in a debugger than a matrix. Mitigated by
  `matrix()` and `toRPY()` being cheap and always available.
- The sign-pinning invariant must be restored anywhere the components are
  written. It lives in exactly one private method, `canonicalize()`, and every
  constructor routes through it.

**Neutral**

- `Scalar` is fixed to `double`. Industrial pose accuracy is specified in
  micrometres over metre-scale workspaces — roughly 1e-7 relative — which is
  outside float's comfortable range once transforms are chained through six
  links. The typedef exists so the decision can be revisited for a GPU kernel
  in WP-12 without touching call sites.

## Related

- ADR-0003 covers the numerical conditioning of the rotation metric, which is a
  consequence of this choice.
