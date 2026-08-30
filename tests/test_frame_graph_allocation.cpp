// SPDX-License-Identifier: Apache-2.0
//
// Proves that the FrameGraph hot path does not allocate by replacing every
// global new/delete form in this executable. This must remain a separate test
// binary: ThreadSanitizer provides its own replacements for the same symbols.

#include <gtest/gtest.h>

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <new>
#include <string>
#include <vector>

#if defined(_MSC_VER)
#include <malloc.h>
#endif

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
#if defined(_MSC_VER)
  return _aligned_malloc(rounded, alignment);
#else
  return std::aligned_alloc(alignment, rounded);
#endif
}

void countedDeallocateAligned(void* p) noexcept {
#if defined(_MSC_VER)
  _aligned_free(p);
#else
  std::free(p);
#endif
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
// So: all of them route through a matching malloc-family pair, and the pairing
// stays consistent no matter which form the standard library reaches for.

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
void operator delete(void* p, std::align_val_t) noexcept { countedDeallocateAligned(p); }
void operator delete[](void* p, std::align_val_t) noexcept {
  countedDeallocateAligned(p);
}
void operator delete(void* p, std::size_t, std::align_val_t) noexcept {
  countedDeallocateAligned(p);
}
void operator delete[](void* p, std::size_t, std::align_val_t) noexcept {
  countedDeallocateAligned(p);
}
void operator delete(void* p, std::align_val_t, const std::nothrow_t&) noexcept {
  countedDeallocateAligned(p);
}
void operator delete[](void* p, std::align_val_t, const std::nothrow_t&) noexcept {
  countedDeallocateAligned(p);
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
};

/// A spine of `spine_depth` frames, then two branches of `branch_depth` each.
DeepTree buildDeepTree(std::uint32_t spine_depth, std::uint32_t branch_depth) {
  DeepTree tree;
  FrameId current = tree.graph.declareRoot("root").value;

  for (std::uint32_t i = 0; i < spine_depth; ++i) {
    const SE3 t(SO3::fromRPY(0.11, -0.07, 0.23), Vec3{0.13, -0.05, 0.21});
    current = tree.graph.declareFrame("spine" + std::to_string(i), current, t).value;
  }

  FrameId left = current;
  FrameId right = current;
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

}  // namespace
}  // namespace motionkit
