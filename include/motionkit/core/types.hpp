// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <stdexcept>

namespace motionkit {

/// Single scalar typedef so the whole stack can be retargeted (double -> float)
/// without touching call sites. Fixed to double: industrial pose accuracy is
/// specified in micrometres over metre-scale workspaces, which is ~1e-7
/// relative precision -- outside float's comfortable range once transforms are
/// chained through a six-link frame graph.
using Scalar = double;

/// Tolerance used for "is this a unit quaternion / orthonormal matrix" checks.
inline constexpr Scalar kOrthoTolerance = 1e-9;
/// Below this, a rotation angle is treated as zero and the axis as undefined.
inline constexpr Scalar kAngleEpsilon = 1e-12;

// ---------------------------------------------------------------------------
// Vec3
// ---------------------------------------------------------------------------
struct Vec3 {
  Scalar x{0.0};
  Scalar y{0.0};
  Scalar z{0.0};

  constexpr Vec3() noexcept = default;
  constexpr Vec3(Scalar vx, Scalar vy, Scalar vz) noexcept : x(vx), y(vy), z(vz) {}

  static constexpr Vec3 zero() noexcept { return {}; }
  static constexpr Vec3 unitX() noexcept { return {1.0, 0.0, 0.0}; }
  static constexpr Vec3 unitY() noexcept { return {0.0, 1.0, 0.0}; }
  static constexpr Vec3 unitZ() noexcept { return {0.0, 0.0, 1.0}; }

  constexpr Vec3 operator+(const Vec3& o) const noexcept {
    return {x + o.x, y + o.y, z + o.z};
  }
  constexpr Vec3 operator-(const Vec3& o) const noexcept {
    return {x - o.x, y - o.y, z - o.z};
  }
  constexpr Vec3 operator-() const noexcept { return {-x, -y, -z}; }
  constexpr Vec3 operator*(Scalar s) const noexcept { return {x * s, y * s, z * s}; }
  constexpr Vec3 operator/(Scalar s) const { return {x / s, y / s, z / s}; }

  constexpr Vec3& operator+=(const Vec3& o) noexcept {
    x += o.x;
    y += o.y;
    z += o.z;
    return *this;
  }
  constexpr Vec3& operator-=(const Vec3& o) noexcept {
    x -= o.x;
    y -= o.y;
    z -= o.z;
    return *this;
  }
  constexpr Vec3& operator*=(Scalar s) noexcept {
    x *= s;
    y *= s;
    z *= s;
    return *this;
  }

  [[nodiscard]] constexpr Scalar dot(const Vec3& o) const noexcept {
    return x * o.x + y * o.y + z * o.z;
  }
  [[nodiscard]] constexpr Vec3 cross(const Vec3& o) const noexcept {
    return {y * o.z - z * o.y, z * o.x - x * o.z, x * o.y - y * o.x};
  }
  [[nodiscard]] constexpr Scalar squaredNorm() const noexcept { return dot(*this); }
  [[nodiscard]] Scalar norm() const noexcept { return std::sqrt(squaredNorm()); }

  /// Throws if the vector is too close to zero to define a direction; callers
  /// that must not throw should test squaredNorm() first.
  [[nodiscard]] Vec3 normalized() const {
    const Scalar n = norm();
    if (n < kAngleEpsilon) {
      throw std::domain_error("motionkit: cannot normalize a near-zero Vec3");
    }
    return *this / n;
  }

  constexpr Scalar operator[](std::size_t i) const {
    switch (i) {
      case 0:
        return x;
      case 1:
        return y;
      case 2:
        return z;
      default:
        throw std::out_of_range("Vec3 index");
    }
  }

  [[nodiscard]] constexpr bool isApprox(const Vec3& o, Scalar tol) const noexcept {
    return (*this - o).squaredNorm() <= tol * tol;
  }
};

constexpr Vec3 operator*(Scalar s, const Vec3& v) noexcept { return v * s; }

// ---------------------------------------------------------------------------
// Mat3 -- row-major 3x3
// ---------------------------------------------------------------------------
struct Mat3 {
  /// Row-major: element (r, c) lives at d[r * 3 + c].
  std::array<Scalar, 9> d{};

  constexpr Mat3() noexcept = default;

  static constexpr Mat3 identity() noexcept {
    Mat3 m;
    m.d = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
    return m;
  }
  static constexpr Mat3 zero() noexcept { return {}; }

  /// Column-wise construction -- the natural form for a rotation matrix built
  /// from the images of the basis vectors.
  static constexpr Mat3 fromColumns(const Vec3& c0, const Vec3& c1,
                                    const Vec3& c2) noexcept {
    Mat3 m;
    m.d = {c0.x, c1.x, c2.x, c0.y, c1.y, c2.y, c0.z, c1.z, c2.z};
    return m;
  }

  constexpr Scalar& operator()(std::size_t r, std::size_t c) { return d[r * 3 + c]; }
  constexpr Scalar operator()(std::size_t r, std::size_t c) const { return d[r * 3 + c]; }

  [[nodiscard]] constexpr Vec3 column(std::size_t c) const {
    return {(*this)(0, c), (*this)(1, c), (*this)(2, c)};
  }
  [[nodiscard]] constexpr Vec3 row(std::size_t r) const {
    return {(*this)(r, 0), (*this)(r, 1), (*this)(r, 2)};
  }

  [[nodiscard]] constexpr Mat3 transpose() const noexcept {
    Mat3 t;
    for (std::size_t r = 0; r < 3; ++r) {
      for (std::size_t c = 0; c < 3; ++c) {
        t(r, c) = (*this)(c, r);
      }
    }
    return t;
  }

  constexpr Mat3 operator*(const Mat3& o) const noexcept {
    Mat3 p;
    for (std::size_t r = 0; r < 3; ++r) {
      for (std::size_t c = 0; c < 3; ++c) {
        Scalar acc = 0.0;
        for (std::size_t k = 0; k < 3; ++k) {
          acc += (*this)(r, k) * o(k, c);
        }
        p(r, c) = acc;
      }
    }
    return p;
  }

  constexpr Vec3 operator*(const Vec3& v) const noexcept {
    return {d[0] * v.x + d[1] * v.y + d[2] * v.z, d[3] * v.x + d[4] * v.y + d[5] * v.z,
            d[6] * v.x + d[7] * v.y + d[8] * v.z};
  }

  [[nodiscard]] constexpr Scalar determinant() const noexcept {
    return d[0] * (d[4] * d[8] - d[5] * d[7]) - d[1] * (d[3] * d[8] - d[5] * d[6]) +
           d[2] * (d[3] * d[7] - d[4] * d[6]);
  }

  [[nodiscard]] constexpr Scalar trace() const noexcept { return d[0] + d[4] + d[8]; }

  /// True when the columns are orthonormal and right-handed, i.e. the matrix is
  /// a member of SO(3) to within `tol`.
  [[nodiscard]] bool isRotation(Scalar tol = kOrthoTolerance) const noexcept;

  [[nodiscard]] bool isApprox(const Mat3& o, Scalar tol) const noexcept;
};

}  // namespace motionkit
