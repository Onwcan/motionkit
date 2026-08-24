// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>

#include <cmath>
#include <numbers>
#include <random>
#include <stdexcept>
#include <vector>

#include "motionkit/core/so3.hpp"

namespace motionkit {
namespace {

constexpr Scalar kPi = std::numbers::pi_v<Scalar>;
constexpr Scalar kAngTol = 1e-12;

/// Uniformly distributed rotations, generated from a normalised Gaussian
/// quaternion. A fixed seed keeps failures reproducible: a property test that
/// cannot be replayed is a flake, not a test.
std::vector<SO3> sampleRotations(std::size_t n, std::uint32_t seed = 0xC0FFEEu) {
  std::mt19937 rng(seed);
  std::normal_distribution<Scalar> gauss(0.0, 1.0);
  std::vector<SO3> out;
  out.reserve(n);
  while (out.size() < n) {
    const Scalar w = gauss(rng);
    const Scalar x = gauss(rng);
    const Scalar y = gauss(rng);
    const Scalar z = gauss(rng);
    if (w * w + x * x + y * y + z * z < 1e-6) {
      continue;  // vanishingly rare, but do not feed a degenerate quaternion in
    }
    out.push_back(SO3::fromQuaternion(w, x, y, z));
  }
  return out;
}

// ---------------------------------------------------------------------------
// Invariants
// ---------------------------------------------------------------------------
TEST(SO3, DefaultIsIdentity) {
  const SO3 r;
  EXPECT_TRUE(r.matrix().isApprox(Mat3::identity(), 1e-15));
  EXPECT_NEAR(r.rotationVector().norm(), 0.0, 1e-15);
}

TEST(SO3, StoredQuaternionIsAlwaysUnitAndCanonical) {
  for (const SO3& r : sampleRotations(2000)) {
    const Scalar n2 = r.w() * r.w() + r.x() * r.x() + r.y() * r.y() + r.z() * r.z();
    EXPECT_NEAR(n2, 1.0, 1e-14);
    EXPECT_GE(r.w(), 0.0) << "canonical form requires a non-negative real part";
  }
}

TEST(SO3, UnnormalizedInputIsNormalized) {
  const SO3 r = SO3::fromQuaternion(2.0, 0.0, 0.0, 0.0);
  EXPECT_NEAR(r.w(), 1.0, 1e-15);
}

TEST(SO3, NegatedQuaternionIsTheSameRotation) {
  const SO3 a = SO3::fromQuaternion(0.5, 0.5, 0.5, 0.5);
  const SO3 b = SO3::fromQuaternion(-0.5, -0.5, -0.5, -0.5);
  EXPECT_TRUE(a.isApprox(b, 1e-14));
}

TEST(SO3, DegenerateQuaternionThrows) {
  EXPECT_THROW((void)SO3::fromQuaternion(0.0, 0.0, 0.0, 0.0), std::domain_error);
}

TEST(SO3, MatrixIsAlwaysAMemberOfSO3) {
  for (const SO3& r : sampleRotations(2000)) {
    EXPECT_TRUE(r.matrix().isRotation(1e-12));
  }
}

// ---------------------------------------------------------------------------
// Known values
// ---------------------------------------------------------------------------
TEST(SO3, QuarterTurnAboutZMapsXOntoY) {
  const Vec3 got = SO3::rotZ(kPi / 2.0) * Vec3::unitX();
  EXPECT_TRUE(got.isApprox(Vec3::unitY(), 1e-15));
}

TEST(SO3, QuarterTurnAboutXMapsYOntoZ) {
  const Vec3 got = SO3::rotX(kPi / 2.0) * Vec3::unitY();
  EXPECT_TRUE(got.isApprox(Vec3::unitZ(), 1e-15));
}

TEST(SO3, QuarterTurnAboutYMapsZOntoX) {
  const Vec3 got = SO3::rotY(kPi / 2.0) * Vec3::unitZ();
  EXPECT_TRUE(got.isApprox(Vec3::unitX(), 1e-15));
}

TEST(SO3, RotationAboutItsOwnAxisIsAFixedPoint) {
  const Vec3 axis = Vec3{1.0, 2.0, -0.5}.normalized();
  const SO3 r = SO3::fromAxisAngle(axis, 1.234);
  EXPECT_TRUE((r * axis).isApprox(axis, 1e-14));
}

TEST(SO3, ZeroAxisThrows) {
  EXPECT_THROW((void)SO3::fromAxisAngle(Vec3{}, 1.0), std::domain_error);
}

// ---------------------------------------------------------------------------
// Round trips
// ---------------------------------------------------------------------------
TEST(SO3, MatrixRoundTrip) {
  for (const SO3& r : sampleRotations(5000)) {
    EXPECT_TRUE(SO3::fromMatrix(r.matrix()).isApprox(r, 1e-11));
  }
}

TEST(SO3, RotationVectorRoundTrip) {
  for (const SO3& r : sampleRotations(5000)) {
    EXPECT_TRUE(SO3::fromRotationVector(r.rotationVector()).isApprox(r, 1e-11));
  }
}

TEST(SO3, RpyRoundTrip) {
  for (const SO3& r : sampleRotations(5000)) {
    Scalar roll{};
    Scalar pitch{};
    Scalar yaw{};
    r.toRPY(roll, pitch, yaw);
    EXPECT_TRUE(SO3::fromRPY(roll, pitch, yaw).isApprox(r, 1e-11));
  }
}

TEST(SO3, RpyRoundTripSurvivesGimbalLock) {
  // Pitch at exactly +/-pi/2 collapses roll and yaw into one degree of freedom.
  // The recovered angles will differ from the inputs, but the rotation they
  // name must still be identical -- that is the property that matters.
  //
  // The tolerance here is 1e-7 rather than the 1e-11 used away from the
  // singularity, and that is deliberate rather than a fudge: see the note in
  // SO3::toRPY. Near lock, roll and yaw are read from matrix entries of size
  // cos(pitch), so the achievable accuracy of any Z-Y-X decomposition is
  // bounded near sqrt(machine epsilon). Tightening this number would not make
  // the code better, it would make the test wrong. At a 400 mm reach 1e-7 rad
  // is 40 nanometres, four orders below what the mechanics can hold.
  for (const Scalar pitch : {kPi / 2.0, -kPi / 2.0}) {
    for (const Scalar roll : {0.0, 0.3, -1.1, 2.7}) {
      for (const Scalar yaw : {0.0, 0.9, -2.2}) {
        const SO3 r = SO3::fromRPY(roll, pitch, yaw);
        Scalar r2{};
        Scalar p2{};
        Scalar y2{};
        r.toRPY(r2, p2, y2);
        EXPECT_TRUE(SO3::fromRPY(r2, p2, y2).isApprox(r, 1e-7))
            << "rpy=(" << roll << ", " << pitch << ", " << yaw << ")";
      }
    }
  }
}

TEST(SO3, RpyAccuracyDegradesGracefullyApproachingGimbalLock) {
  // Walks in towards the singularity and asserts the error stays bounded the
  // whole way rather than blowing up at some particular offset -- which is what
  // a badly chosen branch threshold looks like from the outside.
  const Scalar roll = 0.7;
  const Scalar yaw = -1.4;
  for (const Scalar offset : {1e-1, 1e-3, 1e-5, 1e-7, 1e-9, 1e-11, 0.0}) {
    const SO3 r = SO3::fromRPY(roll, kPi / 2.0 - offset, yaw);
    Scalar r2{};
    Scalar p2{};
    Scalar y2{};
    r.toRPY(r2, p2, y2);
    EXPECT_LT(SO3::fromRPY(r2, p2, y2).angleTo(r), 1e-7) << "offset=" << offset;
  }
}

TEST(SO3, RotationVectorRoundTripNearPi) {
  // The angle-pi case is where the trace branch of matrix-to-quaternion
  // conversion divides by zero, so it gets its own test rather than relying on
  // random sampling to stumble into it.
  const Vec3 axis = Vec3{0.3, -0.7, 0.65}.normalized();
  for (const Scalar eps : {0.0, 1e-9, 1e-6, 1e-3}) {
    const SO3 r = SO3::fromAxisAngle(axis, kPi - eps);
    EXPECT_TRUE(SO3::fromMatrix(r.matrix()).isApprox(r, 1e-7)) << "eps=" << eps;
    EXPECT_TRUE(SO3::fromRotationVector(r.rotationVector()).isApprox(r, 1e-9))
        << "eps=" << eps;
  }
}

TEST(SO3, RotationVectorRoundTripNearIdentity) {
  const Vec3 axis = Vec3{1.0, -1.0, 2.0}.normalized();
  for (const Scalar angle : {0.0, 1e-15, 1e-12, 1e-9, 1e-6}) {
    const SO3 r = SO3::fromRotationVector(axis * angle);
    EXPECT_LE(r.rotationVector().norm(), angle + 1e-15);
    EXPECT_TRUE(SO3::fromRotationVector(r.rotationVector()).isApprox(r, 1e-14));
  }
}

// ---------------------------------------------------------------------------
// Group structure
// ---------------------------------------------------------------------------
TEST(SO3, InverseUndoesRotation) {
  for (const SO3& r : sampleRotations(2000)) {
    EXPECT_TRUE((r * r.inverse()).isApprox(SO3{}, 1e-12));
    EXPECT_TRUE((r.inverse() * r).isApprox(SO3{}, 1e-12));
  }
}

TEST(SO3, CompositionIsAssociative) {
  const std::vector<SO3> s = sampleRotations(300);
  for (std::size_t i = 0; i + 2 < s.size(); i += 3) {
    const SO3 lhs = (s[i] * s[i + 1]) * s[i + 2];
    const SO3 rhs = s[i] * (s[i + 1] * s[i + 2]);
    EXPECT_TRUE(lhs.isApprox(rhs, 1e-12));
  }
}

TEST(SO3, CompositionMatchesMatrixProduct) {
  const std::vector<SO3> s = sampleRotations(400);
  for (std::size_t i = 0; i + 1 < s.size(); i += 2) {
    const Mat3 by_quaternion = (s[i] * s[i + 1]).matrix();
    const Mat3 by_matrix = s[i].matrix() * s[i + 1].matrix();
    EXPECT_TRUE(by_quaternion.isApprox(by_matrix, 1e-13));
  }
}

TEST(SO3, ActionOnVectorMatchesMatrixProduct) {
  const Vec3 v{0.4, -1.7, 2.9};
  for (const SO3& r : sampleRotations(2000)) {
    EXPECT_TRUE((r * v).isApprox(r.matrix() * v, 1e-13));
  }
}

TEST(SO3, RotationPreservesLengthAndAngle) {
  const Vec3 a{1.0, 2.0, 3.0};
  const Vec3 b{-2.0, 0.5, 1.0};
  for (const SO3& r : sampleRotations(1000)) {
    EXPECT_NEAR((r * a).norm(), a.norm(), 1e-13);
    EXPECT_NEAR((r * a).dot(r * b), a.dot(b), 1e-12);
  }
}

TEST(SO3, ChainOfManyProductsDoesNotDrift) {
  // Renormalising inside operator* is what makes this hold. Without it the
  // quaternion slowly leaves the unit sphere and a long kinematic chain skews.
  SO3 acc;
  const SO3 step = SO3::fromAxisAngle(Vec3{1.0, 1.0, 1.0}, 0.01);
  for (int i = 0; i < 100000; ++i) {
    acc = acc * step;
  }
  const Scalar n2 = acc.w() * acc.w() + acc.x() * acc.x() + acc.y() * acc.y() +
                    acc.z() * acc.z();
  EXPECT_NEAR(n2, 1.0, 1e-12);
  EXPECT_TRUE(acc.matrix().isRotation(1e-11));
}

// ---------------------------------------------------------------------------
// Interpolation
// ---------------------------------------------------------------------------
TEST(SO3, SlerpHitsBothEndpoints) {
  const SO3 a = SO3::rotZ(0.3);
  const SO3 b = SO3::fromRPY(0.4, -0.8, 2.1);
  EXPECT_TRUE(a.slerp(b, 0.0).isApprox(a, 1e-13));
  EXPECT_TRUE(a.slerp(b, 1.0).isApprox(b, 1e-13));
}

TEST(SO3, SlerpMidpointIsEquidistant) {
  const SO3 a = SO3::rotZ(0.3);
  const SO3 b = SO3::fromRPY(0.4, -0.8, 2.1);
  const SO3 mid = a.slerp(b, 0.5);
  EXPECT_NEAR(a.angleTo(mid), mid.angleTo(b), 1e-12);
}

TEST(SO3, SlerpAdvancesAtConstantAngularRate) {
  const SO3 a = SO3::rotZ(0.1);
  const SO3 b = SO3::fromRPY(0.9, 0.4, -1.3);
  const Scalar total = a.angleTo(b);
  for (int i = 0; i <= 10; ++i) {
    const Scalar t = static_cast<Scalar>(i) / 10.0;
    EXPECT_NEAR(a.angleTo(a.slerp(b, t)), t * total, 1e-11) << "t=" << t;
  }
}

TEST(SO3, SlerpBetweenIdenticalRotationsIsStable) {
  const SO3 a = SO3::fromRPY(0.2, 0.3, 0.4);
  for (int i = 0; i <= 4; ++i) {
    const Scalar t = static_cast<Scalar>(i) / 4.0;
    EXPECT_TRUE(a.slerp(a, t).isApprox(a, 1e-13));
  }
}

TEST(SO3, AngleToStaysAccurateForVerySmallAngles) {
  // Regression guard. An implementation written as 2 * acos(|dot|) passes every
  // other test in this file and still fails here by six orders of magnitude,
  // because acos is ill-conditioned at 1. Any convergence check, calibration
  // residual or servo error built on angleTo inherits that floor, so the
  // precision is pinned here explicitly.
  const Vec3 axis = Vec3{0.2, -0.9, 0.35}.normalized();
  for (const Scalar angle : {1e-3, 1e-5, 1e-7, 1e-9, 1e-11, 1e-13}) {
    const SO3 a;
    const SO3 b = SO3::fromAxisAngle(axis, angle);
    EXPECT_NEAR(a.angleTo(b), angle, angle * 1e-6 + 1e-16) << "angle=" << angle;
  }
}

TEST(SO3, AngleToIsPreciseAfterAMatrixRoundTrip) {
  // The end-to-end version of the guard above: a conversion that is exact to
  // machine precision componentwise must also read as exact through angleTo.
  for (const SO3& r : sampleRotations(1000)) {
    EXPECT_LT(r.angleTo(SO3::fromMatrix(r.matrix())), 1e-13);
  }
}

TEST(SO3, AngleToIsSymmetricAndBounded) {
  const std::vector<SO3> s = sampleRotations(500);
  for (std::size_t i = 0; i + 1 < s.size(); i += 2) {
    const Scalar d = s[i].angleTo(s[i + 1]);
    EXPECT_NEAR(d, s[i + 1].angleTo(s[i]), kAngTol);
    EXPECT_GE(d, 0.0);
    EXPECT_LE(d, kPi + 1e-12);
  }
}

}  // namespace
}  // namespace motionkit
