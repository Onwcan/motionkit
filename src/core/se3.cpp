// SPDX-License-Identifier: Apache-2.0
#include "motionkit/core/se3.hpp"

#include <array>

#include "motionkit/core/so3.hpp"
#include "motionkit/core/types.hpp"

namespace motionkit {

SE3 SE3::inverse() const noexcept {
  // The inverse of (R, t) is (R^-1, -R^-1 t). Building it in closed form keeps
  // the result exactly on the manifold; a general 4x4 inverse would not.
  const SO3 r_inv = r_.inverse();
  return {r_inv, -(r_inv * t_)};
}

SE3 SE3::operator*(const SE3& rhs) const noexcept {
  return {r_ * rhs.r_, t_ + (r_ * rhs.t_)};
}

Vec3 SE3::operator*(const Vec3& point) const noexcept { return (r_ * point) + t_; }

std::array<Scalar, 16> SE3::matrix() const noexcept {
  const Mat3 r = r_.matrix();
  return {r(0, 0), r(0, 1), r(0, 2), t_.x, r(1, 0), r(1, 1), r(1, 2), t_.y,
          r(2, 0), r(2, 1), r(2, 2), t_.z, 0.0,     0.0,     0.0,     1.0};
}

bool SE3::isApprox(const SE3& other, Scalar linear_tol,
                   Scalar angular_tol) const noexcept {
  return t_.isApprox(other.t_, linear_tol) && r_.isApprox(other.r_, angular_tol);
}

}  // namespace motionkit
