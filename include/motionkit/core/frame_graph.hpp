// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

#include "motionkit/core/se3.hpp"
#include "motionkit/core/types.hpp"

namespace motionkit {

/// Maximum ancestors between a frame and its root.
///
/// A six-axis arm with a tool, a base offset and a couple of sensors reaches
/// about ten. Thirty-two is generous, and it is a *fixed* bound on purpose:
/// it lets lookup() run with stack storage and no allocation, which is what
/// makes the function callable from a control loop.
inline constexpr std::uint32_t kMaxFrameDepth = 32;

/// Why an operation could not produce a transform.
///
/// Returned as a value rather than thrown. A frame lookup failing is an
/// ordinary, expected outcome -- a sensor not yet calibrated, a tool not yet
/// attached -- and a cyclic control loop cannot pay for an exception on a
/// predictable path.
enum class FrameError : std::uint8_t {
  None = 0,
  /// A FrameId that this graph never issued, or a default-constructed one.
  UnknownFrame,
  /// Both frames exist but belong to separate trees, so no transform relates
  /// them. Distinct from UnknownFrame: the question was well-formed, the answer
  /// is that there isn't one.
  Disconnected,
  /// The chain to the root is longer than kMaxFrameDepth.
  DepthExceeded,
  /// A frame with that name already exists.
  DuplicateName,
};

/// Human-readable form of a FrameError, for logs and test failures.
std::string_view toString(FrameError error) noexcept;

/// A value or the reason there isn't one.
template <typename T>
struct Expected {
  T value{};
  FrameError error{FrameError::None};

  constexpr explicit operator bool() const noexcept { return error == FrameError::None; }
  [[nodiscard]] constexpr bool hasValue() const noexcept {
    return error == FrameError::None;
  }
};

/// Opaque handle to a frame in one FrameGraph.
///
/// A distinct type rather than an index so a frame cannot be confused with a
/// joint number, a link index or any other integer at a call site. Handles are
/// only meaningful for the graph that issued them.
class FrameId {
 public:
  constexpr FrameId() noexcept = default;

  [[nodiscard]] constexpr bool valid() const noexcept { return index_ != kInvalidIndex; }
  constexpr bool operator==(const FrameId& other) const noexcept = default;

  /// Index into the owning graph, for diagnostics only.
  [[nodiscard]] constexpr std::uint32_t index() const noexcept { return index_; }

 private:
  friend class FrameGraph;
  static constexpr std::uint32_t kInvalidIndex = 0xFFFFFFFFU;

  explicit constexpr FrameId(std::uint32_t index) noexcept : index_(index) {}

  std::uint32_t index_{kInvalidIndex};
};

/// A forest of named coordinate frames, each holding the transform to its
/// parent, with transforms between any two related frames on demand.
///
/// **This is a tree, not a general graph, and that is the whole design.**
///
/// In a general graph a frame can be reached by more than one path, and the
/// paths disagree: chain a measured camera-to-base transform one way and a
/// measured base-to-camera the other and you get two different answers, both
/// defensible. There is no principled rule for choosing between them, so a
/// graph-shaped API has to either pick arbitrarily or return a set. Giving each
/// frame exactly one parent makes the path unique, so the answer is unique.
///
/// Cycles are impossible by construction rather than by validation: a frame is
/// created with its parent, the parent must already exist, and re-parenting is
/// not offered. There is no sequence of calls that produces a cycle, so there is
/// no cycle check to forget to run.
///
/// Several roots are permitted. A robot base and an uncalibrated camera rig
/// genuinely are unrelated until someone measures the transform between them,
/// and a lookup across that gap returns Disconnected rather than a fiction.
///
/// **Threading**: not synchronised. Declaring frames allocates and invalidates
/// nothing but is not concurrent with anything; setTransform() and lookup() are
/// const-correct but a lookup racing a setTransform is a data race like any
/// other. The intended pattern is one owning thread updating joints and
/// publishing snapshots.
class FrameGraph {
 public:
  FrameGraph() = default;

  /// Pre-allocates storage for `count` frames so declareFrame() does not grow
  /// the backing store later.
  void reserve(std::size_t count);

  /// Creates a frame with no parent, the origin of its own tree.
  Expected<FrameId> declareRoot(std::string_view name);

  /// Creates `name` as a child of `parent`, positioned by `parent_T_frame`.
  ///
  /// Read `parent_T_frame` as "the pose of the new frame expressed in the
  /// parent", matching the SE3 naming convention.
  Expected<FrameId> declareFrame(std::string_view name, FrameId parent,
                                 const SE3& parent_T_frame);

  /// Updates the transform from `frame` to its parent -- a joint moving.
  ///
  /// O(1) and allocation-free, which is the point: a control loop updates a
  /// handful of edges per cycle and reads many transforms out of them.
  FrameError setTransform(FrameId frame, const SE3& parent_T_frame) noexcept;

  /// The transform from `frame` to its parent.
  [[nodiscard]] Expected<SE3> transformToParent(FrameId frame) const noexcept;

  /// Returns `a_T_b`: the pose of frame `b` expressed in frame `a`.
  ///
  /// Composes along the path through the lowest common ancestor, not the root.
  ///
  /// The reason is not that it rounds better. Measured on a 12-deep spine with
  /// 8-deep branches, the round-trip error is 1.04e-15 through the ancestor
  /// against 1.14e-15 through the root -- a real difference, and far too small
  /// to justify anything.
  ///
  /// The reason is that routing through the root makes a *local* relationship
  /// depend on distant frames. Two tools on the same wrist are related by two
  /// edges; going via the root composes the whole chain down and back up again,
  /// and those contributions cancel algebraically but not in floating point.
  /// Under a 20-deep spine the ancestor route returns the two-edge answer
  /// **exactly**, while the root route is 4.9e-16 away from it -- error
  /// imported from twenty frames the answer has nothing to do with. It also
  /// composes 16 transforms instead of 40.
  ///
  /// Both figures come from `FrameGraphRealtime.DeepLookupStaysExactForSiblings`
  /// and `FrameGraphRealtime.LowestCommonAncestorBeatsRoutingThroughTheRoot`.
  ///
  /// Allocation-free and non-throwing.
  [[nodiscard]] Expected<SE3> lookup(FrameId a, FrameId b) const noexcept;

  /// Finds a frame by name. Linear scan: intended for setup and diagnostics,
  /// not for the hot path. Resolve names to FrameIds once and keep the handles.
  [[nodiscard]] Expected<FrameId> find(std::string_view name) const noexcept;

  /// Name of a frame, or an empty view for an unknown handle.
  [[nodiscard]] std::string_view name(FrameId frame) const noexcept;

  /// Parent of `frame`, or an invalid FrameId if it is a root.
  [[nodiscard]] FrameId parent(FrameId frame) const noexcept;

  /// Number of ancestors between `frame` and its root; a root has depth 0.
  [[nodiscard]] Expected<std::uint32_t> depth(FrameId frame) const noexcept;

  [[nodiscard]] std::size_t size() const noexcept { return nodes_.size(); }
  [[nodiscard]] bool empty() const noexcept { return nodes_.empty(); }

 private:
  struct Node {
    std::string name;
    FrameId parent;
    SE3 parent_T_this;
    std::uint32_t depth{0};
  };

  [[nodiscard]] bool isKnown(FrameId frame) const noexcept {
    return frame.valid() && frame.index_ < nodes_.size();
  }

  std::vector<Node> nodes_;
};

}  // namespace motionkit
