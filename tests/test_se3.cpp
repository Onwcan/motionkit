// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>

#include <numbers>
#include <random>
#include <vector>

#include "motionkit/core/se3.hpp"

namespace motionkit {
namespace {

constexpr Scalar kPi = std::numbers::pi_v<Scalar>;

std::vector<SE3> samplePoses(std::size_t n, std::uint32_t seed = 0xBEEF01u) {
  std::mt19937 rng(seed);
  std::normal_distribution<Scalar> gauss(0.0, 1.0);
  std::uniform_real_distribution<Scalar> metres(-2.0, 2.0);
  std::vector<SE3> out;
  out.reserve(n);
  while (out.size() < n) {
    const Scalar w = gauss(rng);
    const Scalar x = gauss(rng);
    const Scalar y = gauss(rng);
    const Scalar z = gauss(rng);
    if (w * w + x * x + y * y + z * z < 1e-6) {
      continue;
    }
    out.emplace_back(SO3::fromQuaternion(w, x, y, z),
                     Vec3{metres(rng), metres(rng), metres(rng)});
  }
  return out;
}

TEST(SE3, DefaultIsIdentity) {
  const SE3 t;
  const Vec3 p{1.0, 2.0, 3.0};
  EXPECT_TRUE((t * p).isApprox(p, 1e-15));
}

TEST(SE3, TranslationThenRotationOrderMatters) {
  // A pose applies rotation first, then translation. Composing in the other
  // order gives a different result, and confusing the two is the single most
  // common frame bug in an integration.
  const SE3 rot = SE3::fromRotation(SO3::rotZ(kPi / 2.0));
  const SE3 tr = SE3::fromTranslation(Vec3{1.0, 0.0, 0.0});
  const Vec3 origin{0.0, 0.0, 0.0};
  EXPECT_TRUE((rot * tr * origin).isApprox(Vec3{0.0, 1.0, 0.0}, 1e-14));
  EXPECT_TRUE((tr * rot * origin).isApprox(Vec3{1.0, 0.0, 0.0}, 1e-14));
}

TEST(SE3, InverseUndoesTransform) {
  for (const SE3& t : samplePoses(2000)) {
    const SE3 identity = t * t.inverse();
    EXPECT_TRUE(identity.isApprox(SE3{}, 1e-12, 1e-12));
    EXPECT_TRUE((t.inverse() * t).isApprox(SE3{}, 1e-12, 1e-12));
  }
}

TEST(SE3, InverseMapsPointsBack) {
  const Vec3 p{0.35, -1.2, 4.0};
  for (const SE3& t : samplePoses(2000)) {
    EXPECT_TRUE((t.inverse() * (t * p)).isApprox(p, 1e-12));
  }
}

TEST(SE3, CompositionIsAssociative) {
  const std::vector<SE3> s = samplePoses(300);
  for (std::size_t i = 0; i + 2 < s.size(); i += 3) {
    const SE3 lhs = (s[i] * s[i + 1]) * s[i + 2];
    const SE3 rhs = s[i] * (s[i + 1] * s[i + 2]);
    EXPECT_TRUE(lhs.isApprox(rhs, 1e-12, 1e-12));
  }
}

TEST(SE3, CompositionAgreesWithSequentialApplication) {
  // a_T_c * p == a_T_b * (b_T_c * p): the whole point of the naming convention.
  const std::vector<SE3> s = samplePoses(400);
  const Vec3 p{1.5, -0.25, 0.75};
  for (std::size_t i = 0; i + 1 < s.size(); i += 2) {
    const SE3 a_T_b = s[i];
    const SE3 b_T_c = s[i + 1];
    EXPECT_TRUE(((a_T_b * b_T_c) * p).isApprox(a_T_b * (b_T_c * p), 1e-12));
  }
}

TEST(SE3, PreservesDistanceBetweenPoints) {
  const Vec3 a{1.0, 2.0, 3.0};
  const Vec3 b{-1.0, 0.5, 2.0};
  const Scalar d = (a - b).norm();
  for (const SE3& t : samplePoses(1000)) {
    EXPECT_NEAR(((t * a) - (t * b)).norm(), d, 1e-12);
  }
}

TEST(SE3, RotateVectorIgnoresTranslation) {
  const SE3 t(SO3::rotZ(kPi / 2.0), Vec3{10.0, -5.0, 3.0});
  EXPECT_TRUE(t.rotateVector(Vec3::unitX()).isApprox(Vec3::unitY(), 1e-14));
}

TEST(SE3, HomogeneousMatrixLayoutIsRowMajorWithTranslationInLastColumn) {
  const SE3 t(SO3::rotZ(kPi / 2.0), Vec3{1.0, 2.0, 3.0});
  const std::array<Scalar, 16> m = t.matrix();
  EXPECT_NEAR(m[3], 1.0, 1e-15);
  EXPECT_NEAR(m[7], 2.0, 1e-15);
  EXPECT_NEAR(m[11], 3.0, 1e-15);
  EXPECT_NEAR(m[12], 0.0, 1e-15);
  EXPECT_NEAR(m[13], 0.0, 1e-15);
  EXPECT_NEAR(m[14], 0.0, 1e-15);
  EXPECT_NEAR(m[15], 1.0, 1e-15);
  // Upper-left block must be the rotation.
  EXPECT_NEAR(m[0], 0.0, 1e-15);
  EXPECT_NEAR(m[1], -1.0, 1e-15);
  EXPECT_NEAR(m[4], 1.0, 1e-15);
  EXPECT_NEAR(m[5], 0.0, 1e-15);
}

TEST(SE3, SixLinkChainMatchesStepwiseApplication) {
  // Stands in for a serial arm: compose the whole chain once, versus walking a
  // point through link by link. Divergence here means forward kinematics will
  // not agree with a per-joint simulation.
  const std::vector<SE3> links = samplePoses(6, 0x515E115u);
  SE3 chained;
  for (const SE3& l : links) {
    chained = chained * l;
  }
  const Vec3 tool{0.0, 0.0, 0.125};
  Vec3 stepwise = tool;
  for (auto it = links.rbegin(); it != links.rend(); ++it) {
    stepwise = *it * stepwise;
  }
  EXPECT_TRUE((chained * tool).isApprox(stepwise, 1e-12));
}

}  // namespace
}  // namespace motionkit
