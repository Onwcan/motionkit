// SPDX-License-Identifier: Apache-2.0
#include "motionkit/core/frame_graph.hpp"

#include <array>

namespace motionkit {
namespace {

/// One step of an ancestor walk: a frame, and the pose of the frame we started
/// from expressed in it.
struct ChainEntry {
  FrameId frame{};
  SE3 entry_T_start{};
};

using Chain = std::array<ChainEntry, kMaxFrameDepth>;

}  // namespace

std::string_view toString(FrameError error) noexcept {
  switch (error) {
    case FrameError::None:
      return "none";
    case FrameError::UnknownFrame:
      return "unknown frame";
    case FrameError::Disconnected:
      return "frames are in separate trees";
    case FrameError::DepthExceeded:
      return "frame chain exceeds kMaxFrameDepth";
    case FrameError::DuplicateName:
      return "a frame with that name already exists";
  }
  return "unrecognised FrameError";
}

void FrameGraph::reserve(std::size_t count) { nodes_.reserve(count); }

Expected<FrameId> FrameGraph::declareRoot(std::string_view name) {
  if (find(name)) {
    return {FrameId{}, FrameError::DuplicateName};
  }
  const auto index = static_cast<std::uint32_t>(nodes_.size());
  nodes_.push_back(Node{std::string{name}, FrameId{}, SE3{}, 0});
  return {FrameId{index}, FrameError::None};
}

Expected<FrameId> FrameGraph::declareFrame(std::string_view name, FrameId parent,
                                           const SE3& parent_T_frame) {
  if (!isKnown(parent)) {
    return {FrameId{}, FrameError::UnknownFrame};
  }
  if (find(name)) {
    return {FrameId{}, FrameError::DuplicateName};
  }
  const std::uint32_t parent_depth = nodes_[parent.index_].depth;
  // Refuse at declaration rather than at lookup. A tree that cannot be walked
  // is not something to discover later from a control loop.
  if (parent_depth + 1 >= kMaxFrameDepth) {
    return {FrameId{}, FrameError::DepthExceeded};
  }
  const auto index = static_cast<std::uint32_t>(nodes_.size());
  nodes_.push_back(Node{std::string{name}, parent, parent_T_frame, parent_depth + 1});
  return {FrameId{index}, FrameError::None};
}

FrameError FrameGraph::setTransform(FrameId frame, const SE3& parent_T_frame) noexcept {
  if (!isKnown(frame)) {
    return FrameError::UnknownFrame;
  }
  // A root has no parent edge to set; silently accepting would let a caller
  // believe they had moved something.
  if (!nodes_[frame.index_].parent.valid()) {
    return FrameError::UnknownFrame;
  }
  nodes_[frame.index_].parent_T_this = parent_T_frame;
  return FrameError::None;
}

Expected<SE3> FrameGraph::transformToParent(FrameId frame) const noexcept {
  if (!isKnown(frame)) {
    return {SE3{}, FrameError::UnknownFrame};
  }
  if (!nodes_[frame.index_].parent.valid()) {
    return {SE3{}, FrameError::Disconnected};
  }
  return {nodes_[frame.index_].parent_T_this, FrameError::None};
}

Expected<SE3> FrameGraph::lookup(FrameId a, FrameId b) const noexcept {
  if (!isKnown(a) || !isKnown(b)) {
    return {SE3{}, FrameError::UnknownFrame};
  }
  if (a == b) {
    return {SE3{}, FrameError::None};
  }

  // Walk each frame up to its root, accumulating the pose of the starting frame
  // in each ancestor. Both chains live on the stack; nothing here allocates.
  const auto walk = [this](FrameId start, Chain& chain,
                           std::uint32_t& length) noexcept -> bool {
    length = 0;
    FrameId current = start;
    SE3 current_T_start{};  // identity: start expressed in itself
    while (true) {
      if (length >= kMaxFrameDepth) {
        return false;
      }
      chain[length] = ChainEntry{current, current_T_start};
      ++length;

      const FrameId next = nodes_[current.index_].parent;
      if (!next.valid()) {
        return true;  // reached a root
      }
      // next_T_start = next_T_current * current_T_start
      current_T_start = nodes_[current.index_].parent_T_this * current_T_start;
      current = next;
    }
  };

  Chain chain_a{};
  Chain chain_b{};
  std::uint32_t len_a = 0;
  std::uint32_t len_b = 0;
  if (!walk(a, chain_a, len_a) || !walk(b, chain_b, len_b)) {
    return {SE3{}, FrameError::DepthExceeded};
  }

  // The first frame on a's chain that also lies on b's chain is the lowest
  // common ancestor. Both chains are bounded by kMaxFrameDepth, so this is a
  // bounded number of integer comparisons rather than a search over the graph.
  for (std::uint32_t i = 0; i < len_a; ++i) {
    for (std::uint32_t j = 0; j < len_b; ++j) {
      if (chain_a[i].frame == chain_b[j].frame) {
        // a_T_b = a_T_lca * lca_T_b = inverse(lca_T_a) * lca_T_b
        return {chain_a[i].entry_T_start.inverse() * chain_b[j].entry_T_start,
                FrameError::None};
      }
    }
  }

  // Both frames exist and both chains terminated at a root, but not the same
  // one: separate trees, and no transform relates them.
  return {SE3{}, FrameError::Disconnected};
}

Expected<FrameId> FrameGraph::find(std::string_view name) const noexcept {
  for (std::size_t i = 0; i < nodes_.size(); ++i) {
    if (nodes_[i].name == name) {
      return {FrameId{static_cast<std::uint32_t>(i)}, FrameError::None};
    }
  }
  return {FrameId{}, FrameError::UnknownFrame};
}

std::string_view FrameGraph::name(FrameId frame) const noexcept {
  if (!isKnown(frame)) {
    return {};
  }
  return nodes_[frame.index_].name;
}

FrameId FrameGraph::parent(FrameId frame) const noexcept {
  if (!isKnown(frame)) {
    return FrameId{};
  }
  return nodes_[frame.index_].parent;
}

Expected<std::uint32_t> FrameGraph::depth(FrameId frame) const noexcept {
  if (!isKnown(frame)) {
    return {0, FrameError::UnknownFrame};
  }
  return {nodes_[frame.index_].depth, FrameError::None};
}

}  // namespace motionkit
