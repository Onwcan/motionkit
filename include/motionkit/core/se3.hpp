// SPDX-License-Identifier: Apache-2.0
#pragma once

#include "motionkit/core/so3.hpp"
#include "motionkit/core/types.hpp"

namespace motionkit {

/// A rigid-body transform: rotation followed by translation.
///
/// Read `A_T_B` as "the pose of frame B expressed in frame A". Then
/// `A_T_B * B_T_C == A_T_C`, and `A_T_B * p_B == p_A`. Keeping to that naming
/// makes frame-mismatch bugs visible at the call site instead of at the robot.
class SE3 {
 public:
  /// Identity transform.
  constexpr SE3() noexcept = default;

  SE3(const SO3& rotation, const Vec3& translation) noexcept
      : r_(rotation), t_(translation) {}

  static SE3 fromTranslation(const Vec3& t) noexcept { return SE3(SO3{}, t); }
  static SE3 fromRotation(const SO3& r) noexcept { return SE3(r, Vec3{}); }

  const SO3& rotation() const noexcept { return r_; }
  const Vec3& translation() const noexcept { return t_; }

  SE3 inverse() const noexcept;
  SE3 operator*(const SE3& rhs) const noexcept;

  /// Transforms a point: rotate, then translate.
  Vec3 operator*(const Vec3& point) const noexcept;

  /// Transforms a free vector (a direction or velocity): rotation only.
  Vec3 rotateVector(const Vec3& v) const noexcept { return r_ * v; }

  /// Row-major 4x4 homogeneous matrix, for interop with CAD and vision stacks
  /// that speak matrices rather than quaternions.
  std::array<Scalar, 16> matrix() const noexcept;

  bool isApprox(const SE3& other, Scalar linear_tol, Scalar angular_tol) const noexcept;

 private:
  SO3 r_{};
  Vec3 t_{};
};

}  // namespace motionkit
