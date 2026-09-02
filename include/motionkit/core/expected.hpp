// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <type_traits>

namespace motionkit {

/// An enum usable as the failure channel of Expected.
///
/// The `None` enumerator is required rather than conventional: Expected
/// default-constructs its error member, so a type without a name for success
/// would make "no error" unrepresentable and every default-constructed
/// Expected would claim to hold a value it does not have.
template <typename E>
concept ErrorEnum = std::is_enum_v<E> && requires { E::None; };

/// A value, or the reason there isn't one.
///
/// Deliberately not std::expected. Every failure in this library is an
/// ordinary, predictable outcome on a path a control loop runs every cycle --
/// a frame that is not calibrated yet, a move whose limits do not admit a
/// profile -- and the caller is expected to branch on it, not to unwind.
/// A plain aggregate keeps the object trivially copyable and the branch
/// visible at the call site.
///
/// The error type is spelled out at every use rather than defaulted, because
/// the set of things that can go wrong is part of a function's signature and
/// worth reading.
///
/// `value` is meaningful only when the error is None. It is default-constructed
/// otherwise -- never a partially filled result, which is the failure mode this
/// type exists to prevent.
template <typename T, ErrorEnum E>
struct Expected {
  T value{};
  E error{E::None};

  constexpr explicit operator bool() const noexcept { return error == E::None; }
  [[nodiscard]] constexpr bool hasValue() const noexcept { return error == E::None; }
};

}  // namespace motionkit
