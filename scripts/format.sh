#!/usr/bin/env bash
#
# Formats (or checks) every first-party source file.
#
# CI runs this exact script rather than its own find/xargs pipeline, so the two
# cannot drift. The first version of this repository had them separate, and the
# CI copy searched a `benchmarks` directory that was empty -- and therefore not
# tracked by git, and therefore absent on a fresh checkout. `find` exited 1 and
# the job failed even when every file was correctly formatted.
#
# Usage:
#   scripts/format.sh            # reformat in place
#   scripts/format.sh --check    # report violations, exit non-zero, change nothing
#
# Override the binary with CLANG_FORMAT=/path/to/clang-format.

set -euo pipefail

REQUIRED_MAJOR=18

# Directories that may contain first-party sources. Missing ones are skipped
# rather than treated as an error: not every repository has every directory,
# and an empty directory does not survive a git checkout.
CANDIDATE_DIRS="include src tests benchmarks examples fuzz"

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$repo_root"

# --- locate clang-format ----------------------------------------------------
find_clang_format() {
  if [ -n "${CLANG_FORMAT:-}" ]; then
    echo "$CLANG_FORMAT"
    return
  fi
  for candidate in "clang-format-${REQUIRED_MAJOR}" clang-format; do
    if command -v "$candidate" >/dev/null 2>&1; then
      echo "$candidate"
      return
    fi
  done
  echo ""
}

CF="$(find_clang_format)"
if [ -z "$CF" ]; then
  cat >&2 <<EOF
error: clang-format not found.

Install the pinned version:
  Debian/Ubuntu : sudo apt-get install -y clang-format-${REQUIRED_MAJOR}
  no root       : pip install "clang-format==${REQUIRED_MAJOR}.1.8"
  or            : CLANG_FORMAT=/path/to/clang-format scripts/format.sh
EOF
  exit 127
fi

# --- verify the version -----------------------------------------------------
# Formatting output changes between major versions. A contributor running a
# different one produces a diff that CI then rejects, which looks like a bug in
# their editor rather than a version mismatch. Say so explicitly.
version_line="$("$CF" --version)"
actual_major="$(echo "$version_line" | sed -n 's/.*version \([0-9][0-9]*\)\..*/\1/p')"
if [ "$actual_major" != "$REQUIRED_MAJOR" ]; then
  echo "warning: $version_line" >&2
  echo "warning: this project is formatted with clang-format ${REQUIRED_MAJOR}." >&2
  echo "warning: a different major version will produce a diff CI rejects." >&2
fi

# --- collect files ----------------------------------------------------------
existing_dirs=""
for dir in $CANDIDATE_DIRS; do
  if [ -d "$dir" ]; then
    existing_dirs="$existing_dirs $dir"
  fi
done

if [ -z "$existing_dirs" ]; then
  echo "no source directories found; nothing to do"
  exit 0
fi

# shellcheck disable=SC2086
mapfile -t files < <(find $existing_dirs \
  \( -name '*.hpp' -o -name '*.cpp' -o -name '*.h' -o -name '*.cc' \) -type f | sort)

if [ "${#files[@]}" -eq 0 ]; then
  echo "no source files found; nothing to do"
  exit 0
fi

# --- run --------------------------------------------------------------------
if [ "${1:-}" = "--check" ]; then
  failed=0
  for file in "${files[@]}"; do
    if ! "$CF" --style=file "$file" | diff -q - "$file" >/dev/null 2>&1; then
      echo "needs formatting: $file"
      failed=1
    fi
  done
  if [ "$failed" -ne 0 ]; then
    echo ""
    echo "Run scripts/format.sh to fix." >&2
    exit 1
  fi
  echo "formatting OK (${#files[@]} files, $version_line)"
else
  "$CF" -i --style=file "${files[@]}"
  echo "formatted ${#files[@]} files with $version_line"
fi
