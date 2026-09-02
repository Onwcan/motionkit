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
| WP-02 | SE(3) transforms, frame graph, tool modeling | **Done** |
| WP-03 | Forward and inverse kinematics (6R) | Planned |
| WP-04 | Rigid-body dynamics (RNEA, CRBA) | Planned |
| WP-05 | Trajectory planning (jerk-limited S-curve, multi-axis synchronisation) | **Done** |
| WP-06 | Hand-eye, TCP and base-frame calibration | Planned |
| WP-11 | Blending, TOPP, and planning from a non-zero initial state | Planned |
| WP-12 | CUDA batch IK and collision checking | Planned |

108 tests, all passing under GCC and Clang in Debug and Release. ASan and UBSan
exercise the full suite. TSan exercises the 101 ordinary tests; the seven
allocator-interposition tests run in a dedicated executable and are excluded
from TSan because both the tests and the sanitizer runtime replace the global
allocation functions.

---

## Build

Requires a C++20 compiler, CMake 3.24+ and Ninja. GoogleTest is fetched
automatically at configure time.

```bash
cmake --preset debug
cmake --build --preset debug
ctest --preset debug
```

Other presets: `release`, `asan`, `tsan`, `tidy`. The `tsan` preset intentionally
runs 101 tests: the seven tests that instrument global allocation are a
test-harness incompatibility with TSan, not an exemption for production code.

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

Or let the frame graph compose the chain, so the relationship between any two
frames is a query rather than a hand-written product:

```cpp
#include "motionkit/core/frame_graph.hpp"

FrameGraph frames;
const FrameId base    = frames.declareRoot("base").value;
const FrameId flange  = frames.declareFrame("flange", base, base_T_flange).value;
const FrameId tcp     = frames.declareFrame("tcp", flange, flange_T_tcp).value;
const FrameId camera  = frames.declareFrame("camera", flange, flange_T_camera).value;

// A joint moves: update one edge, everything below it follows.
frames.setTransform(flange, base_T_flange_now);

// Where is the tool, as the camera sees it? Two edges, not six.
if (const auto camera_T_tcp = frames.lookup(camera, tcp)) {
  const Vec3 target = camera_T_tcp.value * Vec3{};
}
```

Point-to-point motion is planned once and sampled every cycle. Limits are
per-axis; the axes stay synchronised and travel a straight line in joint space:

```cpp
#include "motionkit/core/trajectory.hpp"

const std::array<Scalar, 6> here{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
const std::array<Scalar, 6> there{1.0, 0.2, -0.8, 0.4, 1.6, -0.3};
const std::array<MotionLimits, 6> limits{/* v, a, j per axis */};

const auto move = SynchronizedTrajectory::plan(here, there, limits);
if (!move) {
  return log(toString(move.error));
}

// In the 1 kHz task: sample, never integrate.
std::array<MotionSample, 6> setpoints{};
move.value.sample(elapsed_seconds, setpoints);
```

---

## Design decisions worth arguing about

Full reasoning lives in [`docs/adr/`](docs/adr/). Five that shaped the code:

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

**The frame graph is a tree, and lookups route through the lowest common
ancestor.**
A general graph lets a frame be reached by two paths, and the paths disagree by
whatever the calibration residuals are — both defensible, neither more correct.
One parent per frame makes the path unique, so the answer is unique, and forces
that disagreement to be resolved once by someone who knows which measurement to
trust. Cycles are then impossible by construction rather than by a check
somebody has to remember to run. Routing through the ancestor rather than the
root is *not* mainly about rounding: measured, that is 1.036e-15 against
1.139e-15, which justifies nothing. It is that two tools on the same wrist are
related by two edges, and going via the root composes twenty transforms down and
twenty back up, which cancel algebraically but not in floating point. Under a
20-deep spine the ancestor route returns the two-edge answer **exactly**; the
root route is 4.9e-16 away from it, error imported from frames the answer has
nothing to do with. Full argument in
[ADR-0005](docs/adr/0005-frame-graph-is-a-tree.md).

**Lookup does not allocate, and there is a test that proves it.**
`kMaxFrameDepth` is fixed at 32 so both ancestor chains fit in `std::array` on
the stack, which is what makes `lookup` callable from a control loop.
`FrameGraphRealtime.LookupDoesNotAllocate` replaces every form of global
`operator new` and asserts a zero delta across 1000 lookups — with
`TheAllocationCounterItselfWorks` as a positive control, because a test that
counts allocations proves nothing if the counter is inert. Failures come back as
`Expected<T>` rather than exceptions: a lookup failing is ordinary — a sensor not
yet calibrated — and `Disconnected` is deliberately a different answer from
`UnknownFrame`.

**Multi-axis moves are driven by one path parameter, not one profile per axis.**
Planning each axis separately and stretching the quick ones does synchronise the
endpoints, and no axis exceeds a limit — and the path is still bent, because
each axis keeps its own profile shape and the ratios between them drift through
the move. Measured on a two-axis move: **13.6 mm** off the straight line, from a
plan in which nothing was ever violated. Driving every axis from a single
`s: 0 → 1` makes the ratios constant by construction; the same measurement comes
back 1.1e-16. Whichever axis binds each limit runs exactly at it, which is what
time-optimal means once the path is fixed. See
[ADR-0006](docs/adr/0006-jerk-limited-profiles-and-a-single-path-parameter.md).

**Sample a trajectory; do not integrate it.** Forward Euler at 1 kHz lags the
commanded position by half a step of velocity — 1.0 mm on a 2 m/s move. Then,
because a rest-to-rest profile accelerates and decelerates by equal amounts, the
error cancels to **exactly zero** by the end. An acceptance test that checks the
final position passes, while the machine was in the wrong place for the entire
move. `TrajectorySampling.EulerIntegrationLagsMidMoveThenLandsOnTargetAnyway`
measures both halves.

**Unset limits mean the axis may not move.** `MotionLimits` defaults to zeros
and `validate()` rejects them. Reading an unset limit as "no limit" makes
forgetting to configure an axis indistinguishable from configuring it for full
speed, and the difference is only observable on the machine. Finiteness is
checked before positivity, because a NaN limit passes `<= 0` and then passes
every bound check downstream too — comparisons against NaN are false however
they are written.

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
| ASan + UBSan on all 108 tests, `-fno-sanitize-recover=all` | A UBSan finding fails the build rather than printing a note |
| TSan on the 101 ordinary tests | Ahead of the threaded executor in WP-08; the seven allocator-interposition tests are excluded because TSan defines the same global allocation hooks |
| clang-tidy, `--warnings-as-errors=*` | Rule set and exclusions justified in ADR-0002 |
| `scripts/format.sh --check` with clang-format 18 | Formatting is not a review topic, and CI runs the same check developers run |
| **install with repository tests off + downstream consumer compile and run** | Exercises only the installed package contract; it caught a real bug on first run when the exported target was `motionkit::motionkit_core` but consumers used `motionkit::core` |

---

## Testing approach

Unit tests assert known values; the interesting ones assert **properties** over
thousands of uniformly sampled rotations from a fixed seed — a property test you
cannot replay is a flake, not a test.

Seven allocation tests are instrumentation rather than ordinary unit tests. They
run in their own executable because their global `operator new`/`operator delete`
replacements affect an entire process. That target alone suppresses GNU's
`-Wmismatched-new-delete` diagnostic: the `malloc`/`free` pairing is deliberate
and is the mechanism being tested. The warning remains enabled everywhere else.

- **Group axioms**: associativity, inverse, composition matching matrix product
- **Invariants**: stored quaternion is always unit and canonical; `matrix()` is always in SO(3)
- **Round trips**: quaternion ↔ matrix ↔ rotation vector ↔ RPY
- **Isometry**: rotation preserves lengths and angles; SE(3) preserves distances
- **Singularities tested explicitly**, not left to random sampling to stumble
  into — angle near π (where the trace branch divides by zero), angle near zero
  (where `sin(θ/2)/θ` is 0/0), and pitch at ±π/2

---

## Benchmarks

The claim that these operations are callable from a cyclic task is only worth as
much as the number, so there is a number. No google-benchmark: the library takes
no third-party dependencies, and a benchmark you cannot build straight after
cloning is a benchmark nobody runs.

```bash
cmake -S . -B build/bench -DCMAKE_BUILD_TYPE=Release -DMOTIONKIT_BUILD_BENCHMARKS=ON
cmake --build build/bench -j
./build/bench/benchmarks/motionkit-bench
```

Nanoseconds per call, GCC 15 `-O2`, ordinary desktop with no core isolation and
no real-time scheduling:

| Operation | min | median | max | of a 1 kHz cycle |
|---|---|---|---|---|
| `SO3` composition | 46.1 | 46.3 | 95.3 | 0.005 % |
| `FrameGraph::lookup`, tool to camera | 250.2 | 251.8 | 311.5 | 0.025 % |
| `ScurveProfile::sample` | 4.0 | 4.4 | 11.7 | 0.0004 % |
| `SynchronizedTrajectory::sample`, 6 axes | 8.2 | 8.6 | 15.1 | 0.001 % |
| `SynchronizedTrajectory::plan`, 6 axes | 82.8 | 86.2 | 119.0 | 0.009 % |

The maximum column is dominated by whatever else the machine was doing, and is
reported anyway: a control loop is sized by its worst cycle, not its median.
The last row is the interesting one — planning a six-axis move costs less than a
`FrameGraph` lookup, so a mid-move re-plan on a feed-rate override is something
the cyclic task can do itself rather than hand to another thread.

---

## Licence

Apache-2.0.
