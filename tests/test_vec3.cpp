// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>

#include <stdexcept>

#include "motionkit/core/types.hpp"

namespace motionkit {
namespace {

constexpr Scalar kTol = 1e-12;

TEST(Vec3, DefaultIsZero) {
  constexpr Vec3 v;
  EXPECT_EQ(v.x, 0.0);
  EXPECT_EQ(v.y, 0.0);
  EXPECT_EQ(v.z, 0.0);
}

TEST(Vec3, CrossProductFollowsRightHandRule) {
  EXPECT_TRUE(Vec3::unitX().cross(Vec3::unitY()).isApprox(Vec3::unitZ(), kTol));
  EXPECT_TRUE(Vec3::unitY().cross(Vec3::unitZ()).isApprox(Vec3::unitX(), kTol));
  EXPECT_TRUE(Vec3::unitZ().cross(Vec3::unitX()).isApprox(Vec3::unitY(), kTol));
}

TEST(Vec3, CrossProductIsAntisymmetric) {
  const Vec3 a{1.0, -2.0, 3.5};
  const Vec3 b{-4.0, 0.5, 2.0};
  EXPECT_TRUE(a.cross(b).isApprox(-b.cross(a), kTol));
}

TEST(Vec3, CrossProductIsOrthogonalToBothOperands) {
  const Vec3 a{1.0, -2.0, 3.5};
  const Vec3 b{-4.0, 0.5, 2.0};
  const Vec3 c = a.cross(b);
  EXPECT_NEAR(c.dot(a), 0.0, 1e-12);
  EXPECT_NEAR(c.dot(b), 0.0, 1e-12);
}

TEST(Vec3, NormalizedHasUnitLength) {
  const Vec3 v{3.0, 4.0, 12.0};
  EXPECT_NEAR(v.norm(), 13.0, 1e-12);
  EXPECT_NEAR(v.normalized().norm(), 1.0, 1e-15);
}

TEST(Vec3, NormalizingANearZeroVectorThrows) {
  EXPECT_THROW((void)Vec3{}.normalized(), std::domain_error);
  EXPECT_THROW((void)(Vec3{1e-20, 0.0, 0.0}.normalized()), std::domain_error);
}

TEST(Vec3, IndexOperatorRejectsOutOfRange) {
  const Vec3 v{1.0, 2.0, 3.0};
  EXPECT_EQ(v[0], 1.0);
  EXPECT_EQ(v[1], 2.0);
  EXPECT_EQ(v[2], 3.0);
  EXPECT_THROW((void)v[3], std::out_of_range);
}

TEST(Mat3, IdentityIsARotation) {
  EXPECT_TRUE(Mat3::identity().isRotation());
  EXPECT_NEAR(Mat3::identity().determinant(), 1.0, 1e-15);
}

TEST(Mat3, ReflectionIsOrthonormalButNotARotation) {
  // Orthonormal columns, determinant -1. A left-handed frame like this is the
  // classic sign error in a calibration pipeline, so it must be rejected.
  Mat3 reflection = Mat3::identity();
  reflection(2, 2) = -1.0;
  EXPECT_NEAR(reflection.determinant(), -1.0, 1e-15);
  EXPECT_FALSE(reflection.isRotation());
}

TEST(Mat3, ScaledMatrixIsNotARotation) {
  Mat3 scaled = Mat3::identity();
  scaled(0, 0) = 1.001;
  EXPECT_FALSE(scaled.isRotation());
}

TEST(Mat3, TransposeIsAnInvolution) {
  Mat3 m;
  m.d = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0};
  EXPECT_TRUE(m.transpose().transpose().isApprox(m, kTol));
}

TEST(Mat3, FromColumnsPlacesColumnsCorrectly) {
  const Vec3 c0{1.0, 2.0, 3.0};
  const Vec3 c1{4.0, 5.0, 6.0};
  const Vec3 c2{7.0, 8.0, 9.0};
  const Mat3 m = Mat3::fromColumns(c0, c1, c2);
  EXPECT_TRUE(m.column(0).isApprox(c0, kTol));
  EXPECT_TRUE(m.column(1).isApprox(c1, kTol));
  EXPECT_TRUE(m.column(2).isApprox(c2, kTol));
}

TEST(Mat3, MatrixVectorProductMatchesManualExpansion) {
  Mat3 m;
  m.d = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0};
  const Vec3 v{1.0, 0.5, -2.0};
  const Vec3 got = m * v;
  EXPECT_NEAR(got.x, 1.0 * 1.0 + 2.0 * 0.5 + 3.0 * -2.0, 1e-12);
  EXPECT_NEAR(got.y, 4.0 * 1.0 + 5.0 * 0.5 + 6.0 * -2.0, 1e-12);
  EXPECT_NEAR(got.z, 7.0 * 1.0 + 8.0 * 0.5 + 9.0 * -2.0, 1e-12);
}

}  // namespace
}  // namespace motionkit
