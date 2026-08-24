# motionkit

Robot kinematics, dynamics, trajectory generation and calibration in modern C++.

A from-scratch motion core for a six-axis industrial arm, built to be correct
under adversarial numerics rather than merely correct on the happy path. No
Eigen, no KDL, no Pinocchio — the algorithms are the point.

[![CI](https://github.com/Onwcan/motionkit/actions/workflows/ci.yml/badge.svg)](https://github.com/Onwcan/motionkit/actions/workflows/ci.yml)
![C++20](https://img.shields.io/badge/C%2B%2B-20-blue)
![License](https://img.shields.io/badge/license-Apache--2.0-green)

---

## Status

| Work package | Scope | State |
|---|---|---|
| WP-01 | Build system, CI, static analysis, install/export | **Done** |
| WP-02 | SE(3) transforms, frame graph, tool modeling | **In progress** — SO(3)/SE(3) done, frame graph next |
| WP-03 | Forward and inverse kinematics (6R) | Planned |
| WP-04 | Rigid-body dynamics (RNEA, CRBA) | Planned |
| WP-05 | Trajectory planning (S-curve, TOPP, blending) | Planned |
| WP-06 | Hand-eye, TCP and base-frame calibration | Planned |
| WP-12 | CUDA batch IK and collision checking | Planned |

54 tests, all passing, under GCC and Clang in Debug and Release, plus ASan,
UBSan and TSan.

---

## Build

Requires a C++20 compiler, CMake 3.24+ and Ninja. GoogleTest is fetched
automatically at configure time.

```bash
cmake --preset debug
cmake --build --preset debug
ctest --preset debug
```

Other presets: `release`, `asan`, `tsan`, `tidy`.

Before pushing, run the formatter -- CI enforces it:

```bash
scripts/format.sh
```

It pins clang-format 18; a different major version formats differently and CI
will reject the result.

## Use it downstream

```cmake
find_package(motionkit REQUIRED)
target_link_libraries(your_target PRIVATE motionkit::core)
```

```cpp
#include "motionkit/core/se3.hpp"

using namespace motionkit;

// Read A_T_B as "the pose of frame B expressed in frame A".
const SE3 base_T_flange(SO3::fromRPY(0.0, -M_PI / 2, 0.3), Vec3{0.4, 0.0, 0.65});
const SE3 flange_T_tcp = SE3::fromTranslation(Vec3{0.0, 0.0, 0.125});

const SE3 base_T_tcp = base_T_flange * flange_T_tcp;
const Vec3 tcp_in_base = base_T_tcp * Vec3{0.0, 0.0, 0.0};
```

---

## Design decisions worth arguing about

Full reasoning lives in [`docs/adr/`](docs/adr/). Three that shaped the code:

**Rotations are stored as canonical unit quaternions, not matrices.**
Composing a six-link chain costs 16 multiplies per joint instead of 27, and
correcting drift is one divide instead of a Gram-Schmidt pass. The class
invariant is `‖q‖ = 1` and `w ≥ 0`; pinning the sign collapses the double cover
so `log()` is single-valued and two rotations are comparable componentwise.
`operator*` renormalises on every call — `SO3.ChainOfManyProductsDoesNotDrift`
composes 100 000 rotations and asserts the result is still on the manifold to
1e-12.

**`angleTo` is `2·atan2(‖v‖, |w|)`, never `2·acos(|q₁·q₂|)`.**
`acos` has infinite derivative at 1, so the usual 1e-16 of rounding in a dot
product becomes ~2e-8 rad of angle error. That was measured here, not assumed:
two quaternions agreeing to **1.2e-15** componentwise reported **5.2e-8 rad**
apart under the `acos` form. Every convergence check, calibration residual and
servo error built on this metric would have inherited that floor — about 20 µm
at a 400 mm reach. `SO3.AngleToStaysAccurateForVerySmallAngles` pins it.

**Euler angles are an export format, never a representation.**
`toRPY` recovers pitch via `atan2(-m₂₀, hypot(m₀₀, m₁₀))` rather than
`asin(-m₂₀)`, for the same conditioning reason — and tool-down poses sit exactly
at pitch = −π/2, so this is the common case, not the corner case. Even so, near
gimbal lock roll and yaw are read from quantities of size `cos(pitch)`, which
bounds any Z-Y-X decomposition near `√ε`. The branch threshold sits at 1e-8
where the two competing error terms cross, and the round-trip test through the
singularity is toleranced at 1e-7 **because that is the real limit of the
parametrisation** — tightening it would not improve the code, it would make the
test wrong.

---

## What CI enforces

| Gate | Why it is there |
|---|---|
| GCC + Clang × Debug + Release | `-Wconversion` and `-Wold-style-cast` fire on different constructs per compiler |
| `-Werror` with `-Wconversion -Wsign-conversion -Wold-style-cast -Wshadow` | Silent narrowing in a pose pipeline is a field failure, not a warning |
| ASan + UBSan, `-fno-sanitize-recover=all` | A UBSan finding fails the build rather than printing a note |
| TSan | Ahead of the threaded executor in WP-08 |
| clang-tidy, `--warnings-as-errors=*` | Rule set and exclusions justified in ADR-0002 |
| clang-format `--dry-run --Werror` | Formatting is not a review topic |
| **install + downstream consumer compile** | Caught a real bug on first run: the exported target was `motionkit::motionkit_core` while in-tree consumers used the `motionkit::core` alias. Every `find_package` downstream would have failed, and no unit test could have seen it |

---

## Testing approach

Unit tests assert known values; the interesting ones assert **properties** over
thousands of uniformly sampled rotations from a fixed seed — a property test you
cannot replay is a flake, not a test.

- **Group axioms**: associativity, inverse, composition matching matrix product
- **Invariants**: stored quaternion is always unit and canonical; `matrix()` is always in SO(3)
- **Round trips**: quaternion ↔ matrix ↔ rotation vector ↔ RPY
- **Isometry**: rotation preserves lengths and angles; SE(3) preserves distances
- **Singularities tested explicitly**, not left to random sampling to stumble
  into — angle near π (where the trace branch divides by zero), angle near zero
  (where `sin(θ/2)/θ` is 0/0), and pitch at ±π/2

---

## Licence

Apache-2.0.

Built as portfolio work. Every design decision here is one I can defend in
review — that was the point of building it rather than importing it.
