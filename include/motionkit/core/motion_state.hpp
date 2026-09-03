// SPDX-License-Identifier: Apache-2.0
#pragma once

#include "motionkit/core/types.hpp"

namespace motionkit {

/// Where an axis is and what it is doing, at one instant.
///
/// Distinct from MotionSample, which carries jerk as well. Jerk is the control
/// input rather than part of the state: it can be chosen freely at any instant,
/// while position, velocity and acceleration cannot be. A planner that took
/// jerk as an initial condition would be accepting something it is about to
/// overwrite.
struct MotionState {
  Scalar position{0.0};
  Scalar velocity{0.0};
  Scalar acceleration{0.0};
};

/// The state of one axis at one instant, together with the jerk being applied.
struct MotionSample {
  Scalar position{0.0};
  Scalar velocity{0.0};
  Scalar acceleration{0.0};
  Scalar jerk{0.0};

  [[nodiscard]] constexpr MotionState state() const noexcept {
    return MotionState{position, velocity, acceleration};
  }
};

}  // namespace motionkit
