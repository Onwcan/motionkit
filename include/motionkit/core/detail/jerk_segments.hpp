// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <array>
#include <cstddef>

#include "motionkit/core/motion_state.hpp"
#include "motionkit/core/types.hpp"

namespace motionkit::detail {

/// A fixed run of constant-jerk segments, evaluated in closed form.
///
/// Every profile in this library is a short sequence of segments over which
/// jerk is constant, differing only in how many there are and how their
/// durations are chosen. That arithmetic -- accumulating the state at each
/// boundary once, then evaluating a cubic within the segment containing `t` --
/// is the same either way, and is worth having in one place where its
/// correctness is argued once.
///
/// Positions are relative to the start, so a caller that works in signed
/// displacement and one that works in magnitude and mirrors can share it.
template <std::size_t N>
class JerkSegments {
 public:
  /// Lays out N segments of the given durations and jerks, starting from
  /// velocity `v0` and acceleration `a0` at relative position zero.
  ///
  /// Durations may be zero. A zero-duration segment is kept rather than
  /// dropped, so the number of segments never depends on the arguments and
  /// neither does the cost of sampling.
  constexpr void build(const std::array<Scalar, N>& durations,
                       const std::array<Scalar, N>& jerks, Scalar v0,
                       Scalar a0) noexcept {
    Scalar t = 0.0;
    Scalar p = 0.0;
    Scalar v = v0;
    Scalar a = a0;
    for (std::size_t i = 0; i < N; ++i) {
      segments_[i] = Segment{t, durations[i], jerks[i], MotionState{p, v, a}};
      const Scalar d = durations[i];
      const Scalar j = jerks[i];
      // Order matters: each line consumes the entry values that the lines
      // below it are about to overwrite.
      p += v * d + 0.5 * a * d * d + (1.0 / 6.0) * j * d * d * d;
      v += a * d + 0.5 * j * d * d;
      a += j * d;
      t += d;
    }
    duration_ = t;
    terminal_ = MotionState{p, v, a};
  }

  [[nodiscard]] constexpr Scalar duration() const noexcept { return duration_; }

  /// State reached by running every segment. Subject to rounding: callers that
  /// promise an exact endpoint substitute their own.
  [[nodiscard]] constexpr MotionState terminal() const noexcept { return terminal_; }

  [[nodiscard]] constexpr Scalar segmentDuration(std::size_t i) const noexcept {
    return segments_[i].duration;
  }

  /// State at `t`, with position relative to the start.
  ///
  /// `t` is assumed to lie in (0, duration()); the endpoints belong to the
  /// caller, which knows what it promised there.
  [[nodiscard]] constexpr MotionSample sampleInterior(Scalar t) const noexcept {
    // The last non-empty segment beginning at or before t is the one containing
    // it: segments tile [0, duration()) in order, so any later segment starts
    // at or after this one ends. No early exit, so the cost does not depend on
    // where in the run the sample falls.
    std::size_t index = 0;
    for (std::size_t i = 0; i < N; ++i) {
      if (segments_[i].duration > 0.0 && t >= segments_[i].t0) {
        index = i;
      }
    }

    const Segment& segment = segments_[index];
    const MotionState& e = segment.entry;
    const Scalar dt = t - segment.t0;
    const Scalar j = segment.jerk;
    return MotionSample{e.position + e.velocity * dt + 0.5 * e.acceleration * dt * dt +
                            (1.0 / 6.0) * j * dt * dt * dt,
                        e.velocity + e.acceleration * dt + 0.5 * j * dt * dt,
                        e.acceleration + j * dt, j};
  }

 private:
  struct Segment {
    Scalar t0{0.0};
    Scalar duration{0.0};
    Scalar jerk{0.0};
    MotionState entry{};
  };

  std::array<Segment, N> segments_{};
  Scalar duration_{0.0};
  MotionState terminal_{};
};

}  // namespace motionkit::detail
