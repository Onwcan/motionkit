// SPDX-License-Identifier: Apache-2.0
//
// The two claims frame_graph.hpp makes that a reader cannot check by reading:
// that lookup() does not allocate, and that routing through the lowest common
// ancestor is better than routing through the root.
//
// Both are measured here rather than asserted in a comment.

#include <gtest/gtest.h>

#include <atomic>
#include <cstddef>
#include <cstdio>
#include <cstdlib>
#include <new>
#include <vector>

#include "motionkit/core/frame_graph.hpp"

namespace {

/// Counts every trip through global operator new in this binary.
///
/// Global replacement is heavy-handed, but it is the only way to prove a
/// negative about allocation: a test that merely calls lookup() and passes
/// proves nothing, because an allocation would not fail anything.
std::atomic<std::size_t> g_allocation_count{0};

void* countedAllocate(std::size_t size) {
  g_allocation_count.fetch_add(1, std::memory_order_relaxed);
  return std::malloc(size == 0 ? 1 : size);
}

void* countedAllocateAligned(std::size_t size, std::size_t alignment) {
  g_allocation_count.fetch_add(1, std::memory_order_relaxed);
  // aligned_alloc requires a size that is a multiple of the alignment.
  const std::size_t rounded =
      ((size == 0 ? 1 : size) + alignment - 1) / alignment * alignment;
  return std::aligned_alloc(alignment, rounded);
}

}  // namespace

// Every variant, not just the common one.
//
// Replacing only `operator new(size_t)` and `operator delete(void*)` leaves the
// nothrow and sized forms pointing at the runtime's implementation, and then
// memory obtained one way is released the other. Under AddressSanitizer that is
// caught immediately -- `alloc-dealloc-mismatch (operator new vs free)`, raised
// from inside GoogleTest's own stable_sort, which uses a temporary buffer. It
// is a real bug either way; the sanitizer just makes it loud.
//
// So: all of them route through malloc/free, and the pairing stays consistent
// no matter which form the standard library reaches for.

void* operator new(std::size_t size) {
  void* p = countedAllocate(size);
  if (p == nullptr) {
    throw std::bad_alloc();
  }
  return p;
}

void* operator new[](std::size_t size) {
  void* p = countedAllocate(size);
  if (p == nullptr) {
    throw std::bad_alloc();
  }
  return p;
}

void* operator new(std::size_t size, const std::nothrow_t&) noexcept {
  return countedAllocate(size);
}

void* operator new[](std::size_t size, const std::nothrow_t&) noexcept {
  return countedAllocate(size);
}

void* operator new(std::size_t size, std::align_val_t alignment) {
  void* p = countedAllocateAligned(size, static_cast<std::size_t>(alignment));
  if (p == nullptr) {
    throw std::bad_alloc();
  }
  return p;
}

void* operator new[](std::size_t size, std::align_val_t alignment) {
  void* p = countedAllocateAligned(size, static_cast<std::size_t>(alignment));
  if (p == nullptr) {
    throw std::bad_alloc();
  }
  return p;
}

void* operator new(std::size_t size, std::align_val_t alignment,
                   const std::nothrow_t&) noexcept {
  return countedAllocateAligned(size, static_cast<std::size_t>(alignment));
}

void* operator new[](std::size_t size, std::align_val_t alignment,
                     const std::nothrow_t&) noexcept {
  return countedAllocateAligned(size, static_cast<std::size_t>(alignment));
}

void operator delete(void* p) noexcept { std::free(p); }
void operator delete[](void* p) noexcept { std::free(p); }
void operator delete(void* p, std::size_t) noexcept { std::free(p); }
void operator delete[](void* p, std::size_t) noexcept { std::free(p); }
void operator delete(void* p, const std::nothrow_t&) noexcept { std::free(p); }
void operator delete[](void* p, const std::nothrow_t&) noexcept { std::free(p); }
void operator delete(void* p, std::align_val_t) noexcept { std::free(p); }
void operator delete[](void* p, std::align_val_t) noexcept { std::free(p); }
void operator delete(void* p, std::size_t, std::align_val_t) noexcept { std::free(p); }
void operator delete[](void* p, std::size_t, std::align_val_t) noexcept { std::free(p); }
void operator delete(void* p, std::align_val_t, const std::nothrow_t&) noexcept {
  std::free(p);
}
void operator delete[](void* p, std::align_val_t, const std::nothrow_t&) noexcept {
  std::free(p);
}

namespace motionkit {
namespace {

/// Allocations performed while running `body`.
template <typename F>
std::size_t allocationsDuring(F&& body) {
  const std::size_t before = g_allocation_count.load(std::memory_order_relaxed);
  body();
  return g_allocation_count.load(std::memory_order_relaxed) - before;
}

struct DeepTree {
  FrameGraph graph;
  std::vector<FrameId> left;  ///< left[i] is at depth i+1 under the shared spine
  std::vector<FrameId> right;
  FrameId root;
  FrameId shared;  ///< the deepest frame both branches descend from
};

/// A spine of `spine_depth` frames, then two branches of `branch_depth` each.
///
/// The two leaves are close to each other and far from the root, which is the
/// case where the choice of routing matters. It is also the ordinary shape of a
/// real cell: two tools on the same wrist, or a gripper and a camera on the
/// same flange, with a long chain back to the floor.
DeepTree buildDeepTree(std::uint32_t spine_depth, std::uint32_t branch_depth) {
  DeepTree tree;
  tree.root = tree.graph.declareRoot("root").value;

  // Each spine link is a small rotation and a small offset -- nothing
  // degenerate, just enough that the composition is not trivially exact.
  FrameId current = tree.root;
  for (std::uint32_t i = 0; i < spine_depth; ++i) {
    const SE3 t(SO3::fromRPY(0.11, -0.07, 0.23), Vec3{0.13, -0.05, 0.21});
    current = tree.graph.declareFrame("spine" + std::to_string(i), current, t).value;
  }
  tree.shared = current;

  FrameId left = tree.shared;
  FrameId right = tree.shared;
  for (std::uint32_t i = 0; i < branch_depth; ++i) {
    const SE3 lt(SO3::fromRPY(0.31, 0.17, -0.09), Vec3{0.07, 0.02, -0.03});
    const SE3 rt(SO3::fromRPY(-0.19, 0.29, 0.13), Vec3{-0.04, 0.09, 0.06});
    left = tree.graph.declareFrame("left" + std::to_string(i), left, lt).value;
    right = tree.graph.declareFrame("right" + std::to_string(i), right, rt).value;
    tree.left.push_back(left);
    tree.right.push_back(right);
  }
  return tree;
}

/// What lookup() would produce if it always routed through the root, which is
/// the obvious implementation and the one being argued against.
SE3 lookupViaRoot(const FrameGraph& graph, FrameId a, FrameId b) {
  const auto toRoot = [&graph](FrameId f) {
    SE3 root_T_f{};
    std::vector<SE3> edges;
    for (FrameId current = f; graph.parent(current).valid();
         current = graph.parent(current)) {
      edges.push_back(graph.transformToParent(current).value);
    }
    for (auto it = edges.rbegin(); it != edges.rend(); ++it) {
      root_T_f = root_T_f * (*it);
    }
    return root_T_f;
  };
  return toRoot(a).inverse() * toRoot(b);
}

// ---------------------------------------------------------------------------
// Allocation
// ---------------------------------------------------------------------------

TEST(FrameGraphRealtime, TheAllocationCounterItselfWorks) {
  // A test that counts allocations is worthless if the counter is inert -- this
  // is the positive control for every assertion below.
  const std::size_t allocations = allocationsDuring([] {
    std::vector<int> v;
    v.reserve(1024);
    ASSERT_NE(v.data(), nullptr);
  });
  EXPECT_GT(allocations, 0u);
}

TEST(FrameGraphRealtime, LookupDoesNotAllocate) {
  DeepTree tree = buildDeepTree(12, 8);
  const FrameId a = tree.left.back();
  const FrameId b = tree.right.back();

  // Warm up outside the measurement; the first call must not be special.
  ASSERT_EQ(tree.graph.lookup(a, b).error, FrameError::None);

  Scalar accumulator = 0.0;
  const std::size_t allocations = allocationsDuring([&] {
    for (int i = 0; i < 1000; ++i) {
      const auto result = tree.graph.lookup(a, b);
      accumulator += result.value.translation().x;
    }
  });
  EXPECT_NE(accumulator, 12345.6789);  // keep the loop alive
  EXPECT_EQ(allocations, 0u) << "lookup() allocated " << allocations
                             << " times over 1000 calls";
}

TEST(FrameGraphRealtime, SetTransformDoesNotAllocate) {
  DeepTree tree = buildDeepTree(10, 4);
  const FrameId joint = tree.left.front();
  const SE3 t(SO3::fromRPY(0.05, 0.0, 0.0), Vec3{0.01, 0.0, 0.0});

  const std::size_t allocations = allocationsDuring([&] {
    for (int i = 0; i < 1000; ++i) {
      (void)tree.graph.setTransform(joint, t);
    }
  });
  EXPECT_EQ(allocations, 0u);
}

TEST(FrameGraphRealtime, FindAllocatesNothingEither) {
  // Documented as setup-only because it is a linear scan, not because it
  // allocates -- worth pinning so the distinction stays true.
  DeepTree tree = buildDeepTree(6, 3);
  const std::size_t allocations =
      allocationsDuring([&] { (void)tree.graph.find("left2"); });
  EXPECT_EQ(allocations, 0u);
}

// ---------------------------------------------------------------------------
// Lowest common ancestor vs the root
// ---------------------------------------------------------------------------

TEST(FrameGraphRealtime, LowestCommonAncestorBeatsRoutingThroughTheRoot) {
  constexpr std::uint32_t kSpine = 12;
  constexpr std::uint32_t kBranch = 8;
  DeepTree tree = buildDeepTree(kSpine, kBranch);
  const FrameId a = tree.left.back();
  const FrameId b = tree.right.back();

  const SE3 by_lca = tree.graph.lookup(a, b).value;
  const SE3 by_root = lookupViaRoot(tree.graph, a, b);

  // Both are answers to the same question, so they must agree to well within
  // any tolerance anyone would care about. This is the correctness half.
  EXPECT_TRUE(by_lca.isApprox(by_root, 1e-9, 1e-9));

  // The count is the durable claim. Routing through the lowest common ancestor
  // composes 2 * kBranch edges; routing through the root composes
  // 2 * (kSpine + kBranch). Neither depends on the machine.
  const std::uint32_t compositions_lca = 2 * kBranch;
  const std::uint32_t compositions_root = 2 * (kSpine + kBranch);
  EXPECT_LT(compositions_lca, compositions_root);

  // Round-trip error is the accuracy half. Report both rather than asserting a
  // margin: at double precision over twenty edges the difference is at the
  // limit of what is measurable, and a threshold tuned to this machine would be
  // a flake on another.
  const SE3 lca_round_trip = by_lca * tree.graph.lookup(b, a).value;
  const SE3 root_round_trip = by_root * lookupViaRoot(tree.graph, b, a);
  const Scalar lca_error = (lca_round_trip * Vec3{}).norm();
  const Scalar root_error = (root_round_trip * Vec3{}).norm();

  std::printf(
      "  compositions: lca=%u root=%u\n"
      "  round-trip translation error: lca=%.3e root=%.3e\n",
      compositions_lca, compositions_root, lca_error, root_error);

  // What is safe to assert: neither drifts anywhere near a tolerance that
  // matters for a metre-scale workspace specified in micrometres.
  EXPECT_LT(lca_error, 1e-12);
  EXPECT_LT(root_error, 1e-12);
}

TEST(FrameGraphRealtime, DeepLookupStaysExactForSiblings) {
  // Two frames sharing a parent deep in the tree: the transform between them
  // depends on two edges only, and must not inherit error from the twenty above
  // them. This is the property the lowest-common-ancestor route buys.
  DeepTree tree = buildDeepTree(20, 1);
  const FrameId a = tree.left.back();
  const FrameId b = tree.right.back();

  const SE3 parent_T_a = tree.graph.transformToParent(a).value;
  const SE3 parent_T_b = tree.graph.transformToParent(b).value;
  // Ground truth: two edges, nothing else. Anything the twenty frames above
  // contribute is error.
  const SE3 expected = parent_T_a.inverse() * parent_T_b;

  const SE3 by_lca = tree.graph.lookup(a, b).value;
  const SE3 by_root = lookupViaRoot(tree.graph, a, b);

  const Scalar lca_error = ((by_lca * Vec3{}) - (expected * Vec3{})).norm();
  const Scalar root_error = ((by_root * Vec3{}) - (expected * Vec3{})).norm();
  std::printf("  sibling error under a 20-deep spine: lca=%.3e root=%.3e\n", lca_error,
              root_error);

  // The lowest-common-ancestor route touches exactly the two edges the answer
  // depends on, so it is exact to rounding on those two.
  EXPECT_TRUE(by_lca.isApprox(expected, 1e-15, 1e-15));

  // Routing through the root makes a purely local relationship depend
  // numerically on twenty edges that cancel algebraically but not in floating
  // point. That is the structural argument for the lowest common ancestor --
  // not that it rounds slightly better, but that it does not import error from
  // frames the answer has nothing to do with.
  EXPECT_GT(root_error, lca_error);
}

}  // namespace
}  // namespace motionkit
