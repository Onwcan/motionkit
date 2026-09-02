// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>

#include <atomic>
#include <cstdlib>
#include <new>
#include <numbers>
#include <random>
#include <vector>

#include "motionkit/core/frame_graph.hpp"

namespace motionkit {
namespace {

constexpr Scalar kPi = std::numbers::pi_v<Scalar>;
constexpr Scalar kLinearTol = 1e-12;
constexpr Scalar kAngularTol = 1e-12;

SE3 pose(Scalar roll, Scalar pitch, Scalar yaw, Scalar x, Scalar y, Scalar z) {
  return SE3(SO3::fromRPY(roll, pitch, yaw), Vec3{x, y, z});
}

/// Unwraps an Expected, failing the test rather than returning a default.
template <typename T>
T mustHave(const Expected<T, FrameError>& result) {
  EXPECT_EQ(result.error, FrameError::None) << toString(result.error);
  return result.value;
}

// ---------------------------------------------------------------------------
// Structure
// ---------------------------------------------------------------------------

TEST(FrameGraph, EmptyGraphKnowsNothing) {
  const FrameGraph graph;
  EXPECT_TRUE(graph.empty());
  EXPECT_EQ(graph.find("base").error, FrameError::UnknownFrame);
  EXPECT_EQ(graph.lookup(FrameId{}, FrameId{}).error, FrameError::UnknownFrame);
}

TEST(FrameGraph, DefaultFrameIdIsInvalidAndRejected) {
  FrameGraph graph;
  const FrameId base = mustHave(graph.declareRoot("base"));
  EXPECT_FALSE(FrameId{}.valid());
  EXPECT_EQ(graph.lookup(base, FrameId{}).error, FrameError::UnknownFrame);
  EXPECT_EQ(graph.lookup(FrameId{}, base).error, FrameError::UnknownFrame);
  EXPECT_EQ(graph.declareFrame("tool", FrameId{}, SE3{}).error, FrameError::UnknownFrame);
}

TEST(FrameGraph, DuplicateNamesRejected) {
  FrameGraph graph;
  const FrameId base = mustHave(graph.declareRoot("base"));
  EXPECT_EQ(graph.declareRoot("base").error, FrameError::DuplicateName);
  EXPECT_EQ(graph.declareFrame("base", base, SE3{}).error, FrameError::DuplicateName);
}

TEST(FrameGraph, DepthIsTrackedAndBounded) {
  FrameGraph graph;
  FrameId current = mustHave(graph.declareRoot("root"));
  EXPECT_EQ(mustHave(graph.depth(current)), 0u);

  for (std::uint32_t i = 1; i < kMaxFrameDepth; ++i) {
    const auto child = graph.declareFrame("f" + std::to_string(i), current,
                                          SE3::fromTranslation(Vec3{0.1, 0.0, 0.0}));
    ASSERT_EQ(child.error, FrameError::None) << "at depth " << i;
    current = child.value;
    EXPECT_EQ(mustHave(graph.depth(current)), i);
  }

  // The next one would exceed the bound. Refused at declaration, so a lookup
  // never has to discover it.
  EXPECT_EQ(graph.declareFrame("one_too_deep", current, SE3{}).error,
            FrameError::DepthExceeded);
}

TEST(FrameGraph, RootHasNoParentTransform) {
  FrameGraph graph;
  const FrameId base = mustHave(graph.declareRoot("base"));
  EXPECT_FALSE(graph.parent(base).valid());
  EXPECT_EQ(graph.transformToParent(base).error, FrameError::Disconnected);
  // Setting a root's parent edge would be a no-op that looks like it worked.
  EXPECT_EQ(graph.setTransform(base, SE3::fromTranslation(Vec3{1.0, 0.0, 0.0})),
            FrameError::UnknownFrame);
}

TEST(FrameGraph, NamesResolveToHandles) {
  FrameGraph graph;
  const FrameId base = mustHave(graph.declareRoot("base"));
  const FrameId flange = mustHave(graph.declareFrame("flange", base, SE3{}));
  EXPECT_EQ(mustHave(graph.find("base")), base);
  EXPECT_EQ(mustHave(graph.find("flange")), flange);
  EXPECT_EQ(graph.name(base), "base");
  EXPECT_EQ(graph.name(flange), "flange");
  EXPECT_TRUE(graph.name(FrameId{}).empty());
}

// ---------------------------------------------------------------------------
// Transforms
// ---------------------------------------------------------------------------

TEST(FrameGraph, LookupOfAFrameAgainstItselfIsIdentity) {
  FrameGraph graph;
  const FrameId base = mustHave(graph.declareRoot("base"));
  const FrameId tool =
      mustHave(graph.declareFrame("tool", base, pose(0.1, 0.2, 0.3, 1.0, 2.0, 3.0)));
  EXPECT_TRUE(
      mustHave(graph.lookup(tool, tool)).isApprox(SE3{}, kLinearTol, kAngularTol));
}

TEST(FrameGraph, ParentChildLookupMatchesTheDeclaredTransform) {
  FrameGraph graph;
  const SE3 base_T_tool = pose(0.3, -0.2, 1.1, 0.4, -0.1, 0.65);
  const FrameId base = mustHave(graph.declareRoot("base"));
  const FrameId tool = mustHave(graph.declareFrame("tool", base, base_T_tool));

  EXPECT_TRUE(
      mustHave(graph.lookup(base, tool)).isApprox(base_T_tool, kLinearTol, kAngularTol));
  EXPECT_TRUE(mustHave(graph.lookup(tool, base))
                  .isApprox(base_T_tool.inverse(), kLinearTol, kAngularTol));
}

TEST(FrameGraph, LookupIsItsOwnInverse) {
  FrameGraph graph;
  const FrameId base = mustHave(graph.declareRoot("base"));
  const FrameId a =
      mustHave(graph.declareFrame("a", base, pose(0.2, 0.4, -0.6, 1.0, 0.0, 0.5)));
  const FrameId b =
      mustHave(graph.declareFrame("b", a, pose(-1.0, 0.1, 0.9, 0.0, 0.3, 0.2)));

  const SE3 a_T_b = mustHave(graph.lookup(a, b));
  const SE3 b_T_a = mustHave(graph.lookup(b, a));
  EXPECT_TRUE((a_T_b * b_T_a).isApprox(SE3{}, kLinearTol, kAngularTol));
}

TEST(FrameGraph, ChainComposesTheSameAsMultiplyingByHand) {
  const SE3 base_T_shoulder = pose(0.0, 0.0, 0.7, 0.0, 0.0, 0.4);
  const SE3 shoulder_T_elbow = pose(0.0, -0.9, 0.0, 0.3, 0.0, 0.0);
  const SE3 elbow_T_flange = pose(0.4, 0.2, -0.3, 0.25, 0.0, 0.0);
  const SE3 flange_T_tcp = SE3::fromTranslation(Vec3{0.0, 0.0, 0.125});

  FrameGraph graph;
  const FrameId base = mustHave(graph.declareRoot("base"));
  const FrameId shoulder =
      mustHave(graph.declareFrame("shoulder", base, base_T_shoulder));
  const FrameId elbow = mustHave(graph.declareFrame("elbow", shoulder, shoulder_T_elbow));
  const FrameId flange = mustHave(graph.declareFrame("flange", elbow, elbow_T_flange));
  const FrameId tcp = mustHave(graph.declareFrame("tcp", flange, flange_T_tcp));

  const SE3 expected = base_T_shoulder * shoulder_T_elbow * elbow_T_flange * flange_T_tcp;
  EXPECT_TRUE(
      mustHave(graph.lookup(base, tcp)).isApprox(expected, kLinearTol, kAngularTol));
}

TEST(FrameGraph, SiblingLookupGoesThroughTheCommonAncestor) {
  const SE3 base_T_left = pose(0.0, 0.0, 0.4, 0.5, 0.2, 0.0);
  const SE3 base_T_right = pose(0.1, 0.0, -0.4, 0.5, -0.2, 0.0);

  FrameGraph graph;
  const FrameId base = mustHave(graph.declareRoot("base"));
  const FrameId left = mustHave(graph.declareFrame("left_tool", base, base_T_left));
  const FrameId right = mustHave(graph.declareFrame("right_tool", base, base_T_right));

  const SE3 expected = base_T_left.inverse() * base_T_right;
  EXPECT_TRUE(
      mustHave(graph.lookup(left, right)).isApprox(expected, kLinearTol, kAngularTol));
}

TEST(FrameGraph, PointTransformsAgreeWithTheChain) {
  FrameGraph graph;
  const FrameId base = mustHave(graph.declareRoot("base"));
  const SE3 base_T_flange = pose(0.0, -kPi / 2, 0.3, 0.4, 0.0, 0.65);
  const SE3 flange_T_tcp = SE3::fromTranslation(Vec3{0.0, 0.0, 0.125});
  const FrameId flange = mustHave(graph.declareFrame("flange", base, base_T_flange));
  const FrameId tcp = mustHave(graph.declareFrame("tcp", flange, flange_T_tcp));

  const Vec3 origin_of_tcp_in_base = mustHave(graph.lookup(base, tcp)) * Vec3{};
  const Vec3 by_hand = (base_T_flange * flange_T_tcp) * Vec3{};
  EXPECT_TRUE(origin_of_tcp_in_base.isApprox(by_hand, kLinearTol));
}

// ---------------------------------------------------------------------------
// Motion
// ---------------------------------------------------------------------------

TEST(FrameGraph, SetTransformMovesEverythingBelowIt) {
  FrameGraph graph;
  const FrameId base = mustHave(graph.declareRoot("base"));
  const FrameId joint = mustHave(graph.declareFrame("joint", base, SE3{}));
  const FrameId tcp = mustHave(
      graph.declareFrame("tcp", joint, SE3::fromTranslation(Vec3{1.0, 0.0, 0.0})));

  EXPECT_TRUE((mustHave(graph.lookup(base, tcp)) * Vec3{})
                  .isApprox(Vec3{1.0, 0.0, 0.0}, kLinearTol));

  // Rotate the joint a quarter turn about z; the tool swings with it.
  ASSERT_EQ(graph.setTransform(joint, SE3::fromRotation(SO3::fromRPY(0.0, 0.0, kPi / 2))),
            FrameError::None);
  EXPECT_TRUE(
      (mustHave(graph.lookup(base, tcp)) * Vec3{}).isApprox(Vec3{0.0, 1.0, 0.0}, 1e-12));
}

TEST(FrameGraph, SetTransformRejectsUnknownFrames) {
  FrameGraph graph;
  EXPECT_EQ(graph.setTransform(FrameId{}, SE3{}), FrameError::UnknownFrame);
}

// ---------------------------------------------------------------------------
// Separate trees
// ---------------------------------------------------------------------------

TEST(FrameGraph, FramesInSeparateTreesAreDisconnectedNotUnknown) {
  FrameGraph graph;
  const FrameId base = mustHave(graph.declareRoot("robot_base"));
  const FrameId tcp = mustHave(
      graph.declareFrame("tcp", base, SE3::fromTranslation(Vec3{0.5, 0.0, 0.3})));

  // A camera rig nobody has calibrated against the robot yet.
  const FrameId rig = mustHave(graph.declareRoot("camera_rig"));
  const FrameId lens = mustHave(
      graph.declareFrame("lens", rig, SE3::fromTranslation(Vec3{0.0, 0.0, 0.02})));

  // The question is well-formed and both frames exist. There is simply no
  // answer, and saying Disconnected is different from saying UnknownFrame.
  EXPECT_EQ(graph.lookup(tcp, lens).error, FrameError::Disconnected);
  EXPECT_EQ(graph.lookup(lens, tcp).error, FrameError::Disconnected);

  // Within each tree, lookups still work.
  EXPECT_EQ(graph.lookup(base, tcp).error, FrameError::None);
  EXPECT_EQ(graph.lookup(rig, lens).error, FrameError::None);
}

// ---------------------------------------------------------------------------
// Properties over many random trees
// ---------------------------------------------------------------------------

TEST(FrameGraph, RoundTripHoldsOverRandomTrees) {
  std::mt19937 rng(0xF00Du);
  std::normal_distribution<Scalar> gauss(0.0, 1.0);
  std::uniform_real_distribution<Scalar> metres(-1.0, 1.0);

  FrameGraph graph;
  std::vector<FrameId> frames;
  frames.push_back(mustHave(graph.declareRoot("root")));

  constexpr int kFrameCount = 120;
  for (int i = 1; i < kFrameCount; ++i) {
    std::uniform_int_distribution<std::size_t> pick(0, frames.size() - 1);
    FrameId parent = frames[pick(rng)];
    // Keep within the depth bound; re-pick from the root if we are too deep.
    if (mustHave(graph.depth(parent)) + 1 >= kMaxFrameDepth) {
      parent = frames.front();
    }
    const SE3 t(SO3::fromQuaternion(gauss(rng), gauss(rng), gauss(rng), gauss(rng)),
                Vec3{metres(rng), metres(rng), metres(rng)});
    frames.push_back(mustHave(graph.declareFrame("f" + std::to_string(i), parent, t)));
  }

  std::uniform_int_distribution<std::size_t> any(0, frames.size() - 1);
  for (int trial = 0; trial < 2000; ++trial) {
    const FrameId a = frames[any(rng)];
    const FrameId b = frames[any(rng)];
    const SE3 a_T_b = mustHave(graph.lookup(a, b));
    const SE3 b_T_a = mustHave(graph.lookup(b, a));
    ASSERT_TRUE((a_T_b * b_T_a).isApprox(SE3{}, 1e-10, 1e-10))
        << "round trip failed for " << graph.name(a) << " <-> " << graph.name(b);
  }
}

TEST(FrameGraph, LookupIsTransitiveOverRandomTrees) {
  std::mt19937 rng(0x5EEDu);
  std::normal_distribution<Scalar> gauss(0.0, 1.0);
  std::uniform_real_distribution<Scalar> metres(-1.0, 1.0);

  FrameGraph graph;
  std::vector<FrameId> frames;
  frames.push_back(mustHave(graph.declareRoot("root")));
  for (int i = 1; i < 60; ++i) {
    std::uniform_int_distribution<std::size_t> pick(0, frames.size() - 1);
    FrameId parent = frames[pick(rng)];
    if (mustHave(graph.depth(parent)) + 1 >= kMaxFrameDepth) {
      parent = frames.front();
    }
    const SE3 t(SO3::fromQuaternion(gauss(rng), gauss(rng), gauss(rng), gauss(rng)),
                Vec3{metres(rng), metres(rng), metres(rng)});
    frames.push_back(mustHave(graph.declareFrame("f" + std::to_string(i), parent, t)));
  }

  std::uniform_int_distribution<std::size_t> any(0, frames.size() - 1);
  for (int trial = 0; trial < 2000; ++trial) {
    const FrameId a = frames[any(rng)];
    const FrameId b = frames[any(rng)];
    const FrameId c = frames[any(rng)];
    const SE3 direct = mustHave(graph.lookup(a, c));
    const SE3 via_b = mustHave(graph.lookup(a, b)) * mustHave(graph.lookup(b, c));
    ASSERT_TRUE(direct.isApprox(via_b, 1e-9, 1e-9))
        << graph.name(a) << " -> " << graph.name(b) << " -> " << graph.name(c);
  }
}

}  // namespace
}  // namespace motionkit
