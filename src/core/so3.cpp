// SPDX-License-Identifier: Apache-2.0
#include "motionkit/core/so3.hpp"

#include <algorithm>
#include <stdexcept>

namespace motionkit {

// ---------------------------------------------------------------------------
// Mat3 helpers declared in types.hpp
// ---------------------------------------------------------------------------
bool Mat3::isRotation(Scalar tol) const noexcept {
  const Mat3 should_be_identity = transpose() * (*this);
  if (!should_be_identity.isApprox(Mat3::identity(), tol)) {
    return false;
  }
  // Reject reflections: orthonormal but left-handed.
  return std::abs(determinant() - 1.0) <= tol;
}

bool Mat3::isApprox(const Mat3& o, Scalar tol) const noexcept {
  for (std::size_t i = 0; i < 9; ++i) {
    if (std::abs(d[i] - o.d[i]) > tol) {
      return false;
    }
  }
  return true;
}

// ---------------------------------------------------------------------------
// Construction
// ---------------------------------------------------------------------------
void SO3::canonicalize() {
  const Scalar n = std::sqrt(w_ * w_ + x_ * x_ + y_ * y_ + z_ * z_);
  if (n < kAngleEpsilon) {
    throw std::domain_error("motionkit: degenerate quaternion cannot be normalized");
  }
  const Scalar inv = 1.0 / n;
  w_ *= inv;
  x_ *= inv;
  y_ *= inv;
  z_ *= inv;
  // q and -q denote the same rotation; pinning the sign makes log single-valued.
  if (w_ < 0.0) {
    w_ = -w_;
    x_ = -x_;
    y_ = -y_;
    z_ = -z_;
  }
}

SO3 SO3::fromQuaternion(Scalar w, Scalar x, Scalar y, Scalar z) {
  SO3 q(w, x, y, z);
  q.canonicalize();
  return q;
}

SO3 SO3::fromMatrix(const Mat3& m, Scalar tol) {
  if (!m.isRotation(tol)) {
    throw std::domain_error("motionkit: matrix is not a member of SO(3)");
  }
  // Shepperd branch selection: pick the divisor that is largest, which keeps
  // the division well conditioned for every rotation, including the pi case
  // where the naive trace formula divides by zero.
  const Scalar t = m.trace();
  Scalar w{};
  Scalar x{};
  Scalar y{};
  Scalar z{};
  if (t > 0.0) {
    const Scalar s = std::sqrt(t + 1.0) * 2.0;  // s == 4w
    w = 0.25 * s;
    x = (m(2, 1) - m(1, 2)) / s;
    y = (m(0, 2) - m(2, 0)) / s;
    z = (m(1, 0) - m(0, 1)) / s;
  } else if (m(0, 0) > m(1, 1) && m(0, 0) > m(2, 2)) {
    const Scalar s = std::sqrt(1.0 + m(0, 0) - m(1, 1) - m(2, 2)) * 2.0;  // s == 4x
    w = (m(2, 1) - m(1, 2)) / s;
    x = 0.25 * s;
    y = (m(0, 1) + m(1, 0)) / s;
    z = (m(0, 2) + m(2, 0)) / s;
  } else if (m(1, 1) > m(2, 2)) {
    const Scalar s = std::sqrt(1.0 + m(1, 1) - m(0, 0) - m(2, 2)) * 2.0;  // s == 4y
    w = (m(0, 2) - m(2, 0)) / s;
    x = (m(0, 1) + m(1, 0)) / s;
    y = 0.25 * s;
    z = (m(1, 2) + m(2, 1)) / s;
  } else {
    const Scalar s = std::sqrt(1.0 + m(2, 2) - m(0, 0) - m(1, 1)) * 2.0;  // s == 4z
    w = (m(1, 0) - m(0, 1)) / s;
    x = (m(0, 2) + m(2, 0)) / s;
    y = (m(1, 2) + m(2, 1)) / s;
    z = 0.25 * s;
  }
  return fromQuaternion(w, x, y, z);
}

SO3 SO3::fromAxisAngle(const Vec3& axis, Scalar angle) {
  const Vec3 unit = axis.normalized();  // throws on a zero axis
  const Scalar half = 0.5 * angle;
  const Scalar s = std::sin(half);
  return fromQuaternion(std::cos(half), unit.x * s, unit.y * s, unit.z * s);
}

SO3 SO3::fromRotationVector(const Vec3& rotvec) {
  const Scalar angle = rotvec.norm();
  if (angle < kAngleEpsilon) {
    // sin(t/2)/t tends to 1/2 as t tends to 0. Using the limit avoids 0/0 and
    // stays exact to first order.
    return fromQuaternion(1.0, 0.5 * rotvec.x, 0.5 * rotvec.y, 0.5 * rotvec.z);
  }
  const Scalar half = 0.5 * angle;
  const Scalar k = std::sin(half) / angle;
  return fromQuaternion(std::cos(half), rotvec.x * k, rotvec.y * k, rotvec.z * k);
}

SO3 SO3::fromRPY(Scalar roll, Scalar pitch, Scalar yaw) {
  const Scalar cr = std::cos(0.5 * roll);
  const Scalar sr = std::sin(0.5 * roll);
  const Scalar cp = std::cos(0.5 * pitch);
  const Scalar sp = std::sin(0.5 * pitch);
  const Scalar cy = std::cos(0.5 * yaw);
  const Scalar sy = std::sin(0.5 * yaw);
  return fromQuaternion(cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy,
                        cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy);
}

SO3 SO3::rotX(Scalar angle) { return fromAxisAngle(Vec3::unitX(), angle); }
SO3 SO3::rotY(Scalar angle) { return fromAxisAngle(Vec3::unitY(), angle); }
SO3 SO3::rotZ(Scalar angle) { return fromAxisAngle(Vec3::unitZ(), angle); }

// ---------------------------------------------------------------------------
// Conversions
// ---------------------------------------------------------------------------
Mat3 SO3::matrix() const noexcept {
  const Scalar xx = x_ * x_;
  const Scalar yy = y_ * y_;
  const Scalar zz = z_ * z_;
  const Scalar xy = x_ * y_;
  const Scalar xz = x_ * z_;
  const Scalar yz = y_ * z_;
  const Scalar wx = w_ * x_;
  const Scalar wy = w_ * y_;
  const Scalar wz = w_ * z_;

  Mat3 m;
  m.d = {1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz),       2.0 * (xz + wy),
         2.0 * (xy + wz),       1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx),
         2.0 * (xz - wy),       2.0 * (yz + wx),       1.0 - 2.0 * (xx + yy)};
  return m;
}

Vec3 SO3::rotationVector() const noexcept {
  const Vec3 v{x_, y_, z_};
  const Scalar sin_half = v.norm();
  if (sin_half < kAngleEpsilon) {
    // Near identity the angle is approximately 2 * sin_half and the axis is
    // v / sin_half, so the product is just 2v and the ratio never underflows.
    return v * 2.0;
  }
  // w_ >= 0 is a class invariant, so atan2 lands in [0, pi/2] and the angle in
  // [0, pi] -- always the shorter arc.
  const Scalar angle = 2.0 * std::atan2(sin_half, w_);
  return v * (angle / sin_half);
}

void SO3::toRPY(Scalar& roll, Scalar& pitch, Scalar& yaw) const noexcept {
  const Mat3 m = matrix();

  // m(2,0) == -sin(pitch), and hypot(m(0,0), m(1,0)) == |cos(pitch)|.
  // Recovering pitch as atan2 of the two, rather than asin of the first, is
  // the same trade as in angleTo(): asin has infinite derivative at 1, so it
  // would give up half the mantissa exactly where robots spend their time --
  // tool-down poses sit at pitch = -pi/2.
  const Scalar cos_pitch = std::sqrt(m(0, 0) * m(0, 0) + m(1, 0) * m(1, 0));
  pitch = std::atan2(-m(2, 0), cos_pitch);

  // Below this, roll and yaw stop being separately observable: they are each
  // read from quantities of size cos_pitch that carry an absolute error of
  // ~1e-16, so the relative error is eps/cos_pitch. Folding them together too
  // early costs cos_pitch * roll instead. The two error terms cross near
  // sqrt(eps), which is where this threshold sits and why RPY round-trips
  // through gimbal lock are accurate to ~1e-8 rad and no better. That floor is
  // a property of the Z-Y-X parametrisation, not of this code, and is the
  // reason SO3 stores a quaternion and treats Euler angles as an export format.
  constexpr Scalar kGimbalThreshold = 1e-8;

  if (cos_pitch < kGimbalThreshold) {
    // One degree of freedom left; fold it entirely into yaw. The expression
    // below is correct for pitch at both +pi/2 and -pi/2, which is why there
    // is no second branch.
    roll = 0.0;
    yaw = std::atan2(-m(0, 1), m(1, 1));
    return;
  }
  roll = std::atan2(m(2, 1), m(2, 2));
  yaw = std::atan2(m(1, 0), m(0, 0));
}

// ---------------------------------------------------------------------------
// Group operations
// ---------------------------------------------------------------------------
SO3 SO3::inverse() const noexcept {
  // For a unit quaternion the inverse is the conjugate: no division needed.
  return SO3(w_, -x_, -y_, -z_);
}

SO3 SO3::operator*(const SO3& rhs) const noexcept {
  SO3 out(w_ * rhs.w_ - x_ * rhs.x_ - y_ * rhs.y_ - z_ * rhs.z_,
          w_ * rhs.x_ + x_ * rhs.w_ + y_ * rhs.z_ - z_ * rhs.y_,
          w_ * rhs.y_ - x_ * rhs.z_ + y_ * rhs.w_ + z_ * rhs.x_,
          w_ * rhs.z_ + x_ * rhs.y_ - y_ * rhs.x_ + z_ * rhs.w_);
  // Repeated products drift off the unit sphere. Renormalising here rather
  // than leaving it to the caller is what keeps a six-link chain exact.
  out.canonicalize();
  return out;
}

Vec3 SO3::operator*(const Vec3& v) const noexcept {
  // Rodrigues in quaternion form: fifteen multiplies, against the twenty-seven
  // of materialising the rotation matrix first.
  const Vec3 qv{x_, y_, z_};
  const Vec3 t = qv.cross(v) * 2.0;
  return v + t * w_ + qv.cross(t);
}

SO3 SO3::slerp(const SO3& other, Scalar t) const noexcept {
  Scalar dot = w_ * other.w_ + x_ * other.x_ + y_ * other.y_ + z_ * other.z_;
  Scalar ow = other.w_;
  Scalar ox = other.x_;
  Scalar oy = other.y_;
  Scalar oz = other.z_;
  if (dot < 0.0) {  // take the shorter of the two arcs
    dot = -dot;
    ow = -ow;
    ox = -ox;
    oy = -oy;
    oz = -oz;
  }
  Scalar k0{};
  Scalar k1{};
  if (dot > 1.0 - 1e-9) {
    // Nearly parallel: sin(theta) underflows and plain lerp sits within
    // rounding noise of the geodesic over this range.
    k0 = 1.0 - t;
    k1 = t;
  } else {
    const Scalar theta = std::acos(std::clamp(dot, -1.0, 1.0));
    const Scalar inv_sin = 1.0 / std::sin(theta);
    k0 = std::sin((1.0 - t) * theta) * inv_sin;
    k1 = std::sin(t * theta) * inv_sin;
  }
  return fromQuaternion(k0 * w_ + k1 * ow, k0 * x_ + k1 * ox, k0 * y_ + k1 * oy,
                        k0 * z_ + k1 * oz);
}

Scalar SO3::angleTo(const SO3& other) const noexcept {
  // Deliberately NOT 2 * acos(|dot|). That form is catastrophically
  // ill-conditioned for small angles: acos has infinite derivative at 1, so a
  // dot product carrying the usual 1e-16 of rounding error yields an angle
  // error of sqrt(2 * eps), about 2e-8 radians. Measured on this code, two
  // quaternions agreeing to 1.2e-15 componentwise reported 5.2e-8 rad apart --
  // which would make any tolerance tighter than 1e-7 unusable, and would put a
  // false floor of ~20 micrometres under convergence checks at a 400 mm reach.
  //
  // Taking atan2 of the relative rotation instead is well conditioned across
  // the whole range: near identity the vector part shrinks linearly with the
  // angle and atan2 tracks it down to machine precision.
  const Scalar rw = w_ * other.w_ + x_ * other.x_ + y_ * other.y_ + z_ * other.z_;
  const Scalar rx = w_ * other.x_ - x_ * other.w_ - y_ * other.z_ + z_ * other.y_;
  const Scalar ry = w_ * other.y_ + x_ * other.z_ - y_ * other.w_ - z_ * other.x_;
  const Scalar rz = w_ * other.z_ - x_ * other.y_ + y_ * other.x_ - z_ * other.w_;
  const Scalar vec_norm = std::sqrt(rx * rx + ry * ry + rz * rz);
  // std::abs on the real part collapses the double cover, so the result is the
  // shorter arc and always lands in [0, pi].
  return 2.0 * std::atan2(vec_norm, std::abs(rw));
}

bool SO3::isApprox(const SO3& other, Scalar tol) const noexcept {
  return angleTo(other) <= tol;
}

}  // namespace motionkit
