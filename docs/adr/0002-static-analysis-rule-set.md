# ADR-0002: Static analysis rule set and its exclusions

- **Status**: Accepted
- **Date**: 2026-08-25
- **Deciders**: Onur Can Urhan

## Context

A rule set nobody can explain gets disabled the first time it is inconvenient.
This ADR exists so that every exclusion in `.clang-tidy` has a reason attached
to it, and so that adding another one is a decision rather than a reflex.

The target domain matters: this code is intended to feed a motion runtime with
safety-relevant behaviour. The analysis posture is closer to MISRA/AUTOSAR than
to a typical application — but without pretending to certification this project
has not earned.

## Decision

Enable `bugprone-*`, `cert-*`, `clang-analyzer-*`, `concurrency-*`,
`cppcoreguidelines-*`, `misc-*`, `modernize-*`, `performance-*`,
`portability-*` and `readability-*`, with `WarningsAsErrors: '*'`.

Compiler warnings run at `-Wall -Wextra -Wpedantic -Wshadow -Wconversion
-Wsign-conversion -Wold-style-cast -Wcast-align -Wdouble-promotion
-Wnull-dereference` with `-Werror`.

`-Wconversion` and `-Wsign-conversion` are the two that matter most here.
Silent narrowing in a pose pipeline does not crash — it produces a slightly
wrong number that propagates into a trajectory and shows up as a wrong tool
position on a real machine. That class of defect is invisible at runtime and
trivial to catch at compile time, so it is treated as an error.

## Exclusions, each with cause

| Check | Why it is off |
|---|---|
| `cppcoreguidelines-avoid-magic-numbers`, `readability-magic-numbers` | Rotation code is made of `0.25`, `2.0` and `0.5` from the quaternion algebra itself. Naming them `kQuarter` obscures rather than clarifies. Physical constants and thresholds *are* named — `kGimbalThreshold`, `kAngleEpsilon` — so the intent of the rule is honoured where it applies |
| `readability-identifier-length` | `x`, `y`, `z`, `w`, `q`, `t`, `R` are the standard symbols in this domain. Renaming them to satisfy a length minimum would make the code harder to check against a textbook, which is the opposite of the goal |
| `modernize-use-trailing-return-type` | Stylistic only, and it fights the Google-derived format style in `.clang-format`. No defect-detection value |
| `misc-non-private-member-variables-in-classes`, `cppcoreguidelines-non-private-member-variables-in-classes` | `Vec3` and `Mat3` are deliberately aggregates with public members. They have no invariant to protect, and aggregate initialisation plus `constexpr` use depends on it. `SO3` and `SE3`, which *do* have invariants, keep their state private |
| `cppcoreguidelines-pro-bounds-constant-array-index` | `Mat3` indexes a `std::array<Scalar, 9>` by computed `r * 3 + c`. The alternative is `gsl::at` and a dependency on GSL for a bound that is structurally guaranteed |
| `bugprone-easily-swappable-parameters` | `fromQuaternion(w, x, y, z)` and `toRPY(roll, pitch, yaw)` are the conventional signatures in this domain. Wrapping each scalar in a strong type is defensible but would make the library alien to its users. Revisit if a real ordering bug ever ships |

## Consequences

- The build is noisy to satisfy at first and quiet afterwards. Both the library
  and the tests compile clean under `-Werror` with the full warning set today.
- Every exclusion above is falsifiable. If `bugprone-easily-swappable-parameters`
  would have caught a real defect, that is grounds to re-enable it and adopt
  strong types.
- `clang-format --dry-run --Werror` runs in the same CI job, so formatting never
  reaches code review as a topic.

## Not yet decided

Whether to adopt a MISRA C++ or AUTOSAR C++14 subset for the safety-relevant
components. That question belongs to the `safeedge` runtime rather than to this
library, and will get its own ADR there once the safety supervisor exists.
