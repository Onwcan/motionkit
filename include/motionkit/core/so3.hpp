// SPDX-License-Identifier: Apache-2.0
#pragma once

#include "motionkit/core/types.hpp"

namespace motionkit {

/// A rotation in 3D, stored as a unit quaternion in Hamilton convention with
/// the real part first: q = w + xi + yj + zk.
///
/// Design notes
/// ------------
/// * Quaternion storage rather than a 3x3 matrix: composing a six-link chain
///   costs 16 multiplies per joint instead of 27, and renormalising away drift
///   is a single divide instead of a Gram-Schmidt pass.
/// * The class invariant is `norm(q) == 1` and `w >= 0`. Because q and -q name
///   the same rotation, pinning the sign makes log() single-valued and makes
///   two SO3 values comparable component-wise.
/// * Rotations are *active*: `R * v` rotates the vector v within a fixed frame.
///   To read `R` as a change of basis from frame B to frame A, write it as
///   `A_R_B` and the same operator applies to coordinates.
class SO3 {
 public:
  /// Identity rotation.
  constexpr SO3() noexcept = default;

  /// Normalises the input; throws std::domain_error if it is degenerate.
  static SO3 fromQuaternion(Scalar w, Scalar x, Scalar y, Scalar z);

  /// Nearest rotation to `m`. Throws if `m` is not orthonormal within `tol`.
  static SO3 fromMatrix(const Mat3& m, Scalar tol = 1e-6);

  /// Right-handed rotation of `angle` radians about `axis` (need not be unit).
  static SO3 fromAxisAngle(const Vec3& axis, Scalar angle);

  /// Exponential map: a vector whose direction is the axis and whose magnitude
  /// is the angle in radians.
  static SO3 fromRotationVector(const Vec3& rotvec);

  /// Intrinsic Z-Y-X Tait-Bryan angles, i.e. R = Rz(yaw) * Ry(pitch) * Rx(roll).
  /// This is the convention used by ROS, KDL and most industrial controllers.
  static SO3 fromRPY(Scalar roll, Scalar pitch, Scalar yaw);

  static SO3 rotX(Scalar angle);
  static SO3 rotY(Scalar angle);
  static SO3 rotZ(Scalar angle);

  [[nodiscard]] constexpr Scalar w() const noexcept { return w_; }
  [[nodiscard]] constexpr Scalar x() const noexcept { return x_; }
  [[nodiscard]] constexpr Scalar y() const noexcept { return y_; }
  [[nodiscard]] constexpr Scalar z() const noexcept { return z_; }

  [[nodiscard]] Mat3 matrix() const noexcept;

  /// Logarithmic map, inverse of fromRotationVector(). Magnitude lies in [0, pi].
  [[nodiscard]] Vec3 rotationVector() const noexcept;

  /// Z-Y-X Tait-Bryan decomposition. At the pitch = +/-pi/2 singularity the
  /// roll/yaw split is not unique; this returns roll = 0 and folds the whole
  /// rotation into yaw.
  void toRPY(Scalar& roll, Scalar& pitch, Scalar& yaw) const noexcept;

  [[nodiscard]] SO3 inverse() const noexcept;
  SO3 operator*(const SO3& rhs) const noexcept;
  Vec3 operator*(const Vec3& v) const noexcept;

  /// Geodesic (constant angular velocity) interpolation, t in [0, 1], taking
  /// the shorter of the two arcs.
  [[nodiscard]] SO3 slerp(const SO3& other, Scalar t) const noexcept;

  /// Angle of the relative rotation, in [0, pi].
  [[nodiscard]] Scalar angleTo(const SO3& other) const noexcept;

  /// True when both name the same rotation to within `tol` radians.
  [[nodiscard]] bool isApprox(const SO3& other, Scalar tol) const noexcept;

 private:
  constexpr SO3(Scalar w, Scalar x, Scalar y, Scalar z) noexcept
      : w_(w), x_(x), y_(y), z_(z) {}

  /// Restores the unit-norm and w >= 0 invariants.
  void canonicalize();

  /// Restores the invariants when `norm` is known to be nonzero.
  void canonicalizeWithKnownNorm(Scalar norm) noexcept;

  Scalar w_{1.0};
  Scalar x_{0.0};
  Scalar y_{0.0};
  Scalar z_{0.0};
};

}  // namespace motionkit
