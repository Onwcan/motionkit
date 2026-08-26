# ADR-0005: The frame graph is a tree, and lookups route through the lowest common ancestor

- **Status**: Accepted
- **Date**: 2026-08-26
- **Deciders**: Onur Can Urhan

## Context

Everything above the core needs to ask "where is B, expressed in A?" — inverse
kinematics needs the tool in the base frame (WP-03), calibration needs the
camera in the flange frame (WP-06), and the runtime needs the TCP in world
coordinates every control cycle (WP-08). That question needs one answer, cheaply,
from a cyclic thread.

Two decisions determine whether it works: the shape of the structure, and the
path taken through it.

## Decision 1: a tree, not a general graph

Each frame has exactly one parent. Several roots are allowed; a frame may not be
re-parented.

### Why not a general graph

A general graph lets a frame be reached by more than one path, and in practice
the paths disagree. Measure the camera-to-base transform directly, measure
base-to-flange and flange-to-camera separately, and the two routes to the camera
differ by whatever the calibration residuals are. Both are defensible. Neither
is more correct.

That leaves a graph-shaped API three options, all bad:

- **Pick a path arbitrarily** — the answer depends on an implementation detail
  like edge insertion order, and changes when someone adds an unrelated edge.
- **Return the set of answers** — the caller now has to resolve a calibration
  dispute in the middle of a control loop.
- **Average or optimise over paths** — this is pose-graph optimisation. It is a
  real and useful thing, it belongs in the calibration package, and it is not
  what a transform lookup should be doing silently.

A tree makes the path unique, so the answer is unique. Disagreement between
measurements does not disappear; it is forced to be resolved once, at the point
where the tree is built, by a person who knows which measurement to trust.

This is the same choice ROS's `tf2` makes, for the same reason.

### Cycles are impossible rather than checked

A frame is created together with its parent, the parent must already exist, and
re-parenting is not offered. There is no call sequence that produces a cycle.

The alternative — allow arbitrary edges, then validate — means a cycle check
that has to be run at the right moment, and a failure mode that only appears
once the graph is already wrong. A constraint that cannot be violated needs no
check, and there is no check to forget.

The cost is real: re-parenting a frame at run time is genuinely useful, for a
tool passed between two robots or a part moved from a conveyor to a fixture.
That is a `detach`/`re-declare` operation, and when it is needed it should be
added with an explicit cycle check rather than by loosening this invariant.

### Several roots, and Disconnected as a real answer

A robot base and an uncalibrated camera rig are genuinely unrelated until
someone measures the transform between them. Forcing a single root would mean
inventing an identity transform between them, which is a fiction that reads as
data.

`lookup` across that gap returns `Disconnected`, which is deliberately distinct
from `UnknownFrame`. "I have never heard of that frame" and "both frames exist
and nothing relates them" are different problems with different fixes, and
collapsing them costs someone an afternoon.

## Decision 2: route through the lowest common ancestor

`lookup(a, b)` walks both frames up to their lowest common ancestor and composes
`inverse(lca_T_a) * lca_T_b`, rather than the more obvious
`inverse(root_T_a) * root_T_b`.

### The measurement

The obvious argument is that fewer compositions round less. That argument is
true and it is too weak to decide anything. On a 12-deep spine with 8-deep
branches:

| route | compositions | round-trip translation error |
|---|---:|---:|
| lowest common ancestor | 16 | 1.036e-15 |
| via the root | 40 | 1.139e-15 |

A 10% difference at the 1e-15 level justifies nothing.

The argument that does decide it is structural. Consider two frames that share a
parent — two tools on the same wrist — under a 20-deep spine:

| route | error against the exact two-edge answer |
|---|---:|
| lowest common ancestor | **0.000e+00** |
| via the root | 4.918e-16 |

The relationship between two siblings depends on exactly two edges. Routing
through the root composes twenty transforms down and twenty back up; those
contributions cancel algebraically but not in floating point. The result is that
a purely local relationship acquires error from twenty frames it has nothing to
do with — and that error grows with the depth of the tree, which is to say with
how far away the irrelevant frames are.

Routing through the lowest common ancestor touches only the edges the answer
depends on, and returns the two-edge answer exactly.

Both figures come from `FrameGraphRealtime.LowestCommonAncestorBeatsRoutingThroughTheRoot`
and `FrameGraphRealtime.DeepLookupStaysExactForSiblings`, which print them on
every run.

### The cost

Finding the ancestor is a nested scan over two chains, `O(depth²)` integer
comparisons. With `kMaxFrameDepth` at 32 that is at most 1024 comparisons of a
`uint32_t`, against 40 SE(3) compositions saved. It is not close.

## Decision 3: bounded depth, no allocation, errors as values

`kMaxFrameDepth` is a fixed 32. That bound is what lets `lookup` hold both
ancestor chains in `std::array` on the stack, which is what makes it callable
from a control loop. `FrameGraphRealtime.LookupDoesNotAllocate` pins it by
replacing global `operator new` and asserting a zero delta across 1000 lookups —
with `TheAllocationCounterItselfWorks` as the positive control, because a test
that counts allocations proves nothing if the counter is inert.

Depth is refused at `declareFrame`, not discovered at `lookup`. A tree that
cannot be walked is not something to find out about from a cyclic thread.

Failures are returned as `Expected<T>`, not thrown. A frame lookup failing is
ordinary — a sensor not yet calibrated, a tool not yet attached — and an
exception on a predictable path is a cost a control loop should not pay.

## Consequences

- Transform queries are unique, allocation-free and non-throwing.
- Contradictory measurements must be resolved when the tree is built. This is
  more work up front and it is the point.
- Re-parenting is unavailable until someone needs it enough to add it with a
  cycle check.
- Trees deeper than 32 are rejected. A six-axis arm with tooling and sensors
  reaches about ten.
- `find()` by name is a linear scan. It is documented as setup-only; the
  intended pattern is to resolve names to `FrameId` handles once and keep them.
  At the scale of a real cell — tens of frames — an index would cost more in
  complexity than it saves.

## Notes

Replacing global `operator new` in the test binary has to be done completely.
Replacing only `operator new(size_t)` and `operator delete(void*)` leaves the
nothrow and sized forms pointing at the runtime's implementation, so memory
obtained one way is released the other. AddressSanitizer catches it immediately —
`alloc-dealloc-mismatch (operator new vs free)`, raised from inside GoogleTest's
own `stable_sort`, which allocates a temporary buffer. All eight `new` forms and
all twelve `delete` forms now route through `malloc`/`free` so the pairing holds
whichever form the standard library reaches for.
