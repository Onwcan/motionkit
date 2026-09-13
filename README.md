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
| WP-03 | Forward and inverse kinematics (6R) | **Done** |
| WP-04 | Rigid-body dynamics (RNEA, CRBA) | **Done** |
| WP-05 | Trajectory planning (jerk-limited S-curve, multi-axis synchronisation) | **Done** |
| WP-11 | Stopping from an arbitrary state, and the safety envelope it defines | **Done** |
| WP-06 | Hand-eye and TCP calibration | **Done** |
| WP-12a | Straight-line Cartesian moves, paced by joint limits | **Done** |
| WP-12b | Moves from a non-zero state (the piece blending needed) | **Done** |
| WP-12c | Blended routes through waypoints, without stopping | **Done** |
| WP-12d | Capsule collision checking and clearance | **Done** |
| WP-12e | Pacing by path difficulty (not full TOPP — see ADR-0016) | **Done** |
| WP-12f | CUDA batch IK | Not planned — see ADR-0015 |
| WP-15 | API reference, architecture docs, contribution process | **Done** |

227 tests, all passing under GCC and Clang in Debug and Release. ASan and UBSan
exercise the full suite. TSan exercises the 210 ordinary tests; the seventeen
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
runs 210 tests: the seventeen tests that instrument global allocation are a
test-harness incompatibility with TSan, not an exemption for production code.

Before pushing, run the formatter -- CI enforces it:

```bash
scripts/format.sh
```

It pins clang-format 18; a different major version formats differently and CI
will reject the result.

## Compile-time limits

motionkit uses fixed-size storage to keep its core algorithms allocation-free.
The following public limits are intentionally fixed and cannot currently be
overridden through CMake:

| Constant | Defining header | Limit |
|---|---|---:|
| `kMaxPathKnots` | `motionkit/core/cartesian.hpp` | 65 |
| `kMaxWaypoints` | `motionkit/core/cartesian.hpp` | 16 |
| `kMaxObstacles` | `motionkit/core/collision.hpp` | 32 |

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

A six-axis arm is described by where its joints are, and solved from a seed:

```cpp
#include "motionkit/core/kinematics.hpp"

const SerialChain arm = SerialChain::sixAxisExample();

std::array<Scalar, 6> q{0.3, -0.6, 1.0, 0.4, 0.7, -0.2};
const SE3 base_T_tool = arm.forward(q).value;

// Seeded from where the arm is now, so it does not turn itself inside out
// reaching a pose it could also reach elbow-down.
const auto report = arm.inverse(target, q);
if (!report) {
  return log(toString(report.error));
}
log("converged in {} iterations, closest approach to a singularity {}",
    report.value.iterations, report.value.least_manipulability);
```

Stopping is planned from wherever the axis happens to be, which is the case
that decides how far back a guard has to sit:

```cpp
// How fast may this axis run, given 300 mm of clearance to the hazard?
const auto permitted = maximumSafeSpeed(0.300, limits);

// And when the stop is called for mid-move, from the state it is actually in.
const MotionState now = setpoints[0].state();
const auto stop = StopProfile::plan(now, limits);
log("stopping in {} mm, peaking at {} m/s",
    stop.value.stoppingDistance() * 1000.0, stop.value.peakSpeed());
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

**Joints are described by an axis and a point, not by DH parameters.** A DH
table needs a specific frame on every link, assigned by rules with real freedom
in them, and there are **two incompatible conventions** in circulation that
produce different arms from identical numbers. An axis direction and a point on
that axis are both readable off a CAD model with a ruler, and there is one way
to interpret them. Forward kinematics is then a product of exponentials reusing
the same `SO3` exponential map, and each factor is "translate to the point,
rotate, translate back" — checkable by inspection.

**The Jacobian is tested against the derivative it claims to be.** Central
differences at four configurations, including a singular one, agree with the
analytic form to **3.8e-10**. This matters more than it looks: a sign or frame
error in one of six rows produces a solver that converges for some targets and
spirals for others, which is much harder to diagnose than one that never works.

**Inverse kinematics is damped, and zero damping is not a neutral default.** The
undamped step is the pseudo-inverse, which divides by the Jacobian's smallest
singular value — and that goes to zero at a singularity. Measured with the wrist
a tenth of a milliradian from straight, asked for a **1 mrad** tool rotation:

| damping | commanded joint step |
|---|---|
| 0 | **3.188 rad** (183°) |
| 1e-3 | **0.0019 rad** (0.11°) |

A factor of about 1700 — asked to turn the tool a twentieth of a degree, the
undamped solver swings a joint half a turn. A seeded solve costs **2.3 µs** and
allocates nothing, which is 0.23 % of a 1 kHz cycle, so it can run in the loop
that needs the answer rather than on a thread with a queue in front of it. See
[ADR-0008](docs/adr/0008-kinematics-by-screws-and-a-damped-inverse.md).

**An optimisation that works on one shape of problem and not another, measured
both ways.** A Cartesian move is driven by one profile on one parameter, so its
speed is set by the *worst* knot on the path — one awkward stretch paces the
whole move. Spreading the parameter by difficulty instead of by distance fixes
that: a 50 mm move through a near-singular wrist falls from **8.15 s to 3.27 s**.

On a route with blended corners the same change makes things **worse** —
1.44 s to 2.31 s — so it is off by default and a test asserts the regression so
it stays known. The mechanism: the schedule is computed from the joint path's
curvature and then *changes* it, which is one step of a fixed-point iteration
nobody has shown converges. On a smooth path the step is small; at a corner
blending has already concentrated the difficulty and this overshoots. Damping was
tried at several settings and never rescued the blended case, so there is no knob
for it. It also costs path accuracy — knots serve two masters — which more knots
buy back. See [ADR-0016](docs/adr/0016-spending-the-path-parameter-by-difficulty.md).

**Skipping adjacent links is not enough for self-collision, and believing it is
breaks the model.** Two links sharing a joint touch always, so everyone skips
adjacent pairs. But a spherical wrist turns its last three links about nearly one
point, so links *two* apart overlap at every configuration too — and a model with
only the adjacent rule reports a collision while the arm is parked. Nobody debugs
that; they switch collision checking off, and then the cell has none. `build`
takes an allowed-collision set, and the dullest test in the repo asserts the
example arm is clear while standing still.

**Self-clearance and obstacle clearance are reported apart.** Returning only the
overall minimum was the first design and the tests killed it: a straight arm's own
links sit **0.22 m** apart, so the overall minimum reports 0.22 regardless of where
an obstacle is, until the obstacle is nearer than the arm is to itself. An obstacle
30 cm away became invisible. Not a bug in the arithmetic — a question answered so
literally it stopped being useful. See
[ADR-0015](docs/adr/0015-collision-checking-in-capsules.md).

**Blending cuts the corner, and the library says how much.** A tool cannot turn
a sharp corner at speed — its velocity would have to change direction
instantaneously — so the only choice is between stopping at the waypoint and not
passing exactly through it. Interior waypoints are replaced by a Bézier that
leaves the line `blend_radius` early and rejoins it the same distance later.
Measured on a three-legged staircase of 80 mm right angles: the tool passes each
waypoint **14.1 mm** away at **0.158 m/s** — 73 % of peak, rather than stopping —
and the route takes **1.44 s** against **1.83 s** planned as separate moves.
Asking for exact corners with `blend_radius = 0` is supported and takes
**3.02 s**, slightly more than double. Which of those you want is not something
the library can decide, so it doesn't. See
[ADR-0014](docs/adr/0014-blending-cuts-the-corner.md).

**Two deviations are reported, never one.** `chord_deviation` is discretisation
error and shrinks as knots are added; `corner_deviation` is the corner you asked
to cut and does not. One combined number would let a caller add knots, watch it
fall, and believe the corners were being hit more closely than they were.

**A moving axis does not have to stop before it can follow a new target.**
`ScurveProfile` starts at rest, so an axis already running can only use it by
stopping first — which is exactly what makes a machine look broken. `ReachProfile`
takes a full `MotionState`, so it can be planned from the sample the executor is
holding right now. From 1.0 m/s to a goal 3 m away it takes **1.82 s** against
**2.22 s** for stop-then-restart.

**The cruise velocity is found by bisection, not by a dozen closed-form cases.**
Distance covered is monotonic in cruise velocity, so a bracket that narrows
cannot be wrong in a case nobody thought of — where the closed form is a dozen
sign conventions, each wrong only for inputs nobody tried. It costs **1.32 µs**
against 120 ns for the closed-form branch, still 0.13 % of a 1 kHz cycle. From
rest it reproduces `ScurveProfile` to **1.1e-16 s**, which is two constructions
sharing no code agreeing. See
[ADR-0013](docs/adr/0013-reaching-a-position-from-a-moving-axis.md).

**A Cartesian move is paced by one speed, and the path between knots is C1.**
Linear interpolation between knots makes joint velocity piecewise constant, so
acceleration is an *impulse* at every knot crossing — a plan that satisfies its
own model and hands out a stream violating the limits dozens of times. Cubic
Hermite with the same tangents the pacing uses is C1, and the test differentiates
the **sampled output** rather than the internal model: peak **1.71 rad/s** of a
2.0 limit, peak **4.51 rad/s²** of an 8.0 limit.

**Joint acceleration has two sources and they compete for one limit.** The path
bending in joint space grows with the *square* of path speed; the profile
changing speed is linear in it. Bounding only the second respects every limit on
paper and exceeds them on a curve, so the budget is split explicitly and the
cornering half feeds back into the speed bound. See
[ADR-0012](docs/adr/0012-cartesian-moves-are-paced-by-one-speed.md).

**What a straight line costs, measured.** Near a singularity the Jacobian demands
enormous joint speed for an ordinary tool speed: the same 50 mm move takes
**8.15 s** at manipulability 6.2e-05 against **1.10 s** at 3.7e-03 — a factor of
**7.4**, from geometry alone. And holding the line at all costs **2.30×** the
time of letting the joints go where they like between the same endpoints. Both
are worth knowing before promising a cycle time. The tool is exactly on the line
at knots and within a reported **2.9e-05 m** between them — measured, not
claimed.

**Calibration refuses data that cannot determine the answer.** Six poses that
touch a point from one orientation, or six camera stations whose every motion
turns about the same axis, look like perfectly good measurement sets. They are
not: the problem is rank-deficient and the number that falls out is whatever the
rounding left behind. `DegenerateGeometry` is a separate error from
`NotEnoughSamples` precisely because more of the same useless pose does not
help, and telling an operator "not enough samples" sends them to collect twenty
more identical ones.

**The hand-eye rotation is one linear system, not an average of pairwise
guesses.** Written with quaternions, `A X = X B` becomes
`(L(q_A) − R(q_B)) q_X = 0` — *linear*, so every station contributes to a single
null-space problem. The usual approach solves each pair and averages, which
requires averaging rotations, and there is no way to do that which is both
simple and correct. Recovery from noise-free data is exact to **2.6e-15 rad**;
with 0.2 mm and 0.2 mrad of wobble on every observation it degrades in
proportion to **3.2e-04 rad**, and the reported residual is the same order as
the error it is reporting. See
[ADR-0011](docs/adr/0011-calibration-refuses-what-it-cannot-determine.md).

**A calibration reports two residuals, because one hides the case that matters.**
A tip known to a tenth of a millimetre and one known to five are the same
struct. With a single touch displaced by 1 mm among six good ones, the RMS reads
**3.6e-04 m** — sub-millimetre, fine — while the worst case reads **8.0e-04 m**.
The average is exactly what a bad touch hides behind.

**The mass matrix is computed twice, by two algorithms that share no derivation.**
It could have come out of the recursive Newton-Euler code already written — one
call per joint with a unit acceleration — for about twenty lines. The
composite-rigid-body algorithm was written separately anyway, propagating the
momentum of each frozen distal subtree. The two share the link frames and
nothing else, so each is a test of the other, and the agreement is not a
tolerance anybody chose: **8.882e-16 kg·m²**, about one unit in the last place.
A hand-written expected matrix would only have proved that the author and the
implementation multiplied the same way.

**Gravity enters as an acceleration of the base, not as a weight on each link.**
A base accelerating upward at 9.81 m/s² is indistinguishable from inside from a
base at rest in a gravitational field, so one line at the top of the recursion
replaces a term in every link's force balance — and removes `n` chances to get a
sign wrong in configurations nobody tested. It also makes an arm on a wall or a
ceiling free rather than special: `setGravity` takes any vector, and a test
asserts that reversing the field reverses every torque. The gravity torque is
checked against a third derivation entirely — it is the gradient of the
potential energy, which is a one-line sum over link heights sharing no code with
the recursion, and the two agree to **9.05e-09 N·m**. See
[ADR-0010](docs/adr/0010-dynamics-in-the-base-frame.md).

**An inertia tensor that could not belong to a real body is refused.** Symmetric
and positive definite is not enough: the principal moments must also satisfy the
triangle inequality, because no distribution of mass makes one axis harder to
spin than the other two together. Checking that needs the eigenvalues, so there
is a closed-form symmetric 3×3 eigensolver that exists for this alone. It earns
its place because an implausible inertia produces *plausible* torques — nothing
downstream fails, the arm just needs numbers no real machine would, and the
error gets blamed on the controller.

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

**A speed reading is not a stopping distance.** Two axes both reading 1.5 m/s
stop in **290 mm** and **967 mm** — a factor of 3.3 — because one of them is
still accelerating at 8 m/s². Acceleration cannot be changed instantaneously,
so that axis has already committed to 2.3 m/s it does not yet have. Any safety
envelope computed from velocity alone is wrong for every axis that is not
already at constant speed, which during a move is most of them.

**And `v²/(2a)` is optimistic by exactly `v·a/(2j)`.** That is the trapezoidal
stopping distance; the jerk-limited one is `v·a/(2j) + v²/(2a)`, because the
time spent building and releasing the braking force is time spent travelling.
At 2 m/s with `a = 8 m/s²` and `j = 40 m/s³` the formula says **250 mm**, the
real stop is **450 mm**, and a guard positioned from the formula sits 200 mm
inside the hazard. `maximumSafeSpeed()` inverts the real one; it returns zero
when there is no room, because the absence of room is not permission to move.
See [ADR-0007](docs/adr/0007-stopping-is-planned-to-zero-acceleration.md).

**"Stop" means zero acceleration, not zero velocity.** An axis at +0.05 m/s
decelerating at −8 m/s² reaches zero velocity almost at once, and unwinding that
acceleration takes 0.2 s regardless, so it carries on into reverse — measured,
to −0.75 m/s, settling 199 mm *behind* where it started. So `stoppingDistance()`
can oppose the initial velocity, and a planner that targeted velocity alone
would hand back a stop that does not stay stopped.

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
| ASan + UBSan on all 227 tests, `-fno-sanitize-recover=all` | A UBSan finding fails the build rather than printing a note |
| TSan on the 210 ordinary tests | Ahead of the threaded executor in WP-08; the seventeen allocator-interposition tests are excluded because TSan defines the same global allocation hooks |
| clang-tidy, `--warnings-as-errors=*` | Rule set and exclusions justified in ADR-0002 |
| `scripts/format.sh --check` with clang-format 18 | Formatting is not a review topic, and CI runs the same check developers run |
| **install with repository tests off + downstream consumer compile and run** | Exercises only the installed package contract; it caught a real bug on first run when the exported target was `motionkit::motionkit_core` but consumers used `motionkit::core` |
| **Doxygen with `WARN_AS_ERROR`** | A public entity added without a doc comment fails the pull request that added it. Turning it on found 86 gaps and two comments that bound to the wrong member — see [ADR-0009](docs/adr/0009-the-api-reference-is-a-gate.md) |

---

## Documentation

| Document | What it answers |
|---|---|
| [docs/architecture.md](docs/architecture.md) | C4 context, container and component views, and the rules that decide where new code goes |
| [docs/adr/](docs/adr/) | Sixteen decisions, each with the alternatives that lost and why |
| [CONTRIBUTING.md](CONTRIBUTING.md) | How to build, what the gates are, and the conventions clang-format cannot express |
| [docs/review-checklist.md](docs/review-checklist.md) | The questions that have actually caught something here |
| [CHANGELOG.md](CHANGELOG.md) | What changed, and what `0.x` promises |

The API reference is generated from the headers and **enforced**, not merely
published:

```bash
cmake -S . -B build-docs -DMOTIONKIT_BUILD_DOCS=ON
cmake --build build-docs --target docs
```

Doxygen is not required for an ordinary build — the option defaults to `OFF`,
and CI is what keeps the reference honest. Publication to GitHub Pages is
opt-in via the `PUBLISH_DOCS` repository variable, because a deploy job that
assumed Pages was configured would put a red cross on `main` for a repository
setting rather than for a commit.

The single most useful thing to know before adding code here is in
[docs/architecture.md](docs/architecture.md): the pose side (SO3, SE3,
FrameGraph, SerialChain, DynamicChain) and the motion side (MotionState,
ScurveProfile, StopProfile) are two subtrees that meet in exactly **one** place.
Trajectory planning is over scalar axes and knows nothing about poses;
kinematics knows nothing about time. `cartesian` is the sole crossing, because
a straight line traversed within joint limits cannot be described by either
side alone — and keeping it the only one means the awkward part of the design
lives in one file.

---

## Testing approach

Unit tests assert known values; the interesting ones assert **properties** over
thousands of uniformly sampled rotations from a fixed seed — a property test you
cannot replay is a flake, not a test.

Seventeen allocation tests are instrumentation rather than ordinary unit tests. They
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
| `SO3` composition | 46.3 | 46.5 | 92.0 | 0.005 % |
| `FrameGraph::lookup`, tool to camera | 250.3 | 250.8 | 646.7 | 0.025 % |
| `ScurveProfile::sample` | 4.0 | 4.1 | 12.1 | 0.0004 % |
| `SynchronizedTrajectory::sample`, 6 axes | 8.6 | 8.8 | 14.8 | 0.001 % |
| `SynchronizedTrajectory::plan`, 6 axes | 86.6 | 91.8 | 339.5 | 0.009 % |
| `StopProfile::plan` | 17.6 | 18.6 | 26.5 | 0.002 % |
| `ReachProfile::plan`, long move | 117.4 | 120.3 | 289.8 | 0.012 % |
| `ReachProfile::plan`, short move | 1257.1 | 1320.8 | 4240.3 | 0.132 % |
| `maximumSafeSpeed` | 4.8 | 5.1 | 11.7 | 0.0005 % |
| `SerialChain::forward`, 6R | 175.7 | 176.6 | 527.7 | 0.018 % |
| `SerialChain::jacobian`, 6R | 417.4 | 427.2 | 1029.7 | 0.043 % |
| `SerialChain::inverse`, 6R seeded | 2209.1 | 2319.8 | 7499.1 | 0.232 % |
| `DynamicChain::inverseDynamics`, 6R | 415.6 | 445.6 | 4967.7 | 0.045 % |
| `DynamicChain::gravityTorque`, 6R | 400.3 | 428.6 | 1038.0 | 0.043 % |
| `DynamicChain::massMatrix`, 6R CRBA | 399.8 | 427.0 | 1050.9 | 0.043 % |
| `CartesianPlan::sample`, 6R 33 knots | 17.9 | 18.4 | 28.5 | 0.002 % |
| `CartesianPlan::plan`, 6R 33 knots | 76109 | 77373 | 87793 | 7.74 % |
| `CollisionModel::clearance`, 6R + 3 obstacles | 416.1 | 431.6 | 1013.2 | 0.043 % |
| `CartesianPlan::plan`, paced by difficulty | 145466 | 148714 | 155124 | 14.87 % |

The maximum column is dominated by whatever else the machine was doing, and is
reported anyway: a control loop is sized by its worst cycle, not its median.
Planning a six-axis move costs less than a `FrameGraph` lookup, and planning a
stop costs a fifth of that, so both a mid-move re-plan on a feed-rate override
and a stop decided by the safety task are things those tasks can do themselves
rather than hand to a thread they do not control.

---

## Licence

Apache-2.0.
