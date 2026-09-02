// SPDX-License-Identifier: Apache-2.0
//
// Per-call cost of the operations this library claims are callable from a
// cyclic control task. The claim is only worth as much as the number.
//
// Method: each operation runs in batches, and the batch is timed rather than
// the call, because a steady_clock read costs more than most of the calls
// measured here. Batches are repeated and the distribution reported, since the
// median says what a cycle normally costs and the maximum says whether a cycle
// can be missed.

#include <algorithm>
#include <array>
#include <chrono>
#include <cstddef>
#include <cstdio>
#include <string>
#include <vector>

#include "motionkit/core/frame_graph.hpp"
#include "motionkit/core/trajectory.hpp"

namespace motionkit {
namespace {

constexpr std::size_t kBatch = 1000;
constexpr std::size_t kRepeats = 500;
/// One cycle of a 1 kHz control task, for scale.
constexpr double kCycleNanos = 1'000'000.0;

/// Somewhere for results to go that the optimiser cannot reason about.
volatile Scalar g_sink = 0.0;

void barrier() {
#if defined(__GNUC__) || defined(__clang__)
  asm volatile("" ::: "memory");
#endif
}

struct Result {
  double min_ns{0.0};
  double median_ns{0.0};
  double max_ns{0.0};
};

template <typename F>
Result measure(F&& body) {
  std::vector<double> per_call;
  per_call.reserve(kRepeats);

  // Warm the caches and let the branch predictors settle; the first batch is
  // not what a control loop experiences.
  for (std::size_t i = 0; i < kBatch; ++i) {
    body(i);
  }

  for (std::size_t r = 0; r < kRepeats; ++r) {
    barrier();
    const auto start = std::chrono::steady_clock::now();
    for (std::size_t i = 0; i < kBatch; ++i) {
      body(i);
    }
    const auto finish = std::chrono::steady_clock::now();
    barrier();
    const auto elapsed =
        std::chrono::duration_cast<std::chrono::nanoseconds>(finish - start).count();
    per_call.push_back(static_cast<double>(elapsed) / static_cast<double>(kBatch));
  }

  std::sort(per_call.begin(), per_call.end());
  return Result{per_call.front(), per_call[per_call.size() / 2], per_call.back()};
}

void report(const char* name, const Result& r) {
  std::printf("  %-38s %8.1f %9.1f %9.1f   %7.4f%%\n", name, r.min_ns, r.median_ns,
              r.max_ns, 100.0 * r.median_ns / kCycleNanos);
}

FrameGraph buildArm(std::array<FrameId, 3>& of_interest) {
  FrameGraph graph;
  graph.reserve(24);
  FrameId current = graph.declareRoot("base").value;
  for (int i = 0; i < 6; ++i) {
    const SE3 link(SO3::fromRPY(0.11, -0.07, 0.23), Vec3{0.13, -0.05, 0.21});
    current = graph.declareFrame("joint" + std::to_string(i), current, link).value;
  }
  const SE3 offset(SO3::fromRPY(0.0, 0.0, 0.4), Vec3{0.0, 0.0, 0.125});
  of_interest[0] = graph.declareFrame("tcp", current, offset).value;
  of_interest[1] = graph.declareFrame("camera", current, offset).value;
  of_interest[2] = current;
  return graph;
}

}  // namespace
}  // namespace motionkit

int main() {
  using namespace motionkit;

  std::array<FrameId, 3> frames{};
  FrameGraph graph = buildArm(frames);

  constexpr MotionLimits axis{2.0, 8.0, 40.0};
  const ScurveProfile profile = ScurveProfile::plan(0.0, 2.0, axis).value;

  const std::array<Scalar, 6> start{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  const std::array<Scalar, 6> goal{1.0, 0.2, -0.8, 0.4, 1.6, -0.3};
  const std::array<MotionLimits, 6> limits{axis, axis, axis, axis, axis, axis};
  const SynchronizedTrajectory arm =
      SynchronizedTrajectory::plan(start, goal, limits).value;
  std::array<MotionSample, 6> samples{};

  std::printf("motionkit micro-benchmarks (nanoseconds per call)\n");
  std::printf("  %-38s %8s %9s %9s   %8s\n", "operation", "min", "median", "max",
              "of 1 kHz");
  std::printf("  %s\n", std::string(78, '-').c_str());

  report("SO3 composition", measure([&](std::size_t) {
           const SO3 r = SO3::fromRPY(0.1, 0.2, 0.3) * SO3::fromRPY(0.3, 0.2, 0.1);
           g_sink = r.w();
         }));

  report("FrameGraph::lookup (tcp <- camera)", measure([&](std::size_t) {
           g_sink = graph.lookup(frames[0], frames[1]).value.translation().x;
         }));

  report("FrameGraph::lookup (tcp <- base)", measure([&](std::size_t) {
           g_sink = graph.lookup(frames[0], frames[2]).value.translation().z;
         }));

  report("ScurveProfile::sample", measure([&](std::size_t i) {
           const Scalar t = profile.duration() * static_cast<Scalar>(i % 1000) / 1000.0;
           g_sink = profile.sample(t).position;
         }));

  report("SynchronizedTrajectory::sample (6 axes)", measure([&](std::size_t i) {
           const Scalar t = arm.duration() * static_cast<Scalar>(i % 1000) / 1000.0;
           (void)arm.sample(t, samples);
           g_sink = samples[0].position;
         }));

  report("SynchronizedTrajectory::plan (6 axes)", measure([&](std::size_t i) {
           const Scalar factor = 0.5 + 0.0005 * static_cast<Scalar>(i % 1000);
           const std::array<MotionLimits, 6> scaled{
               limits[0].scaled(factor), limits[1].scaled(factor),
               limits[2].scaled(factor), limits[3].scaled(factor),
               limits[4].scaled(factor), limits[5].scaled(factor)};
           g_sink = SynchronizedTrajectory::plan(start, goal, scaled).value.duration();
         }));

  std::printf(
      "\n  Timings come from an ordinary desktop with no isolated cores and no\n"
      "  real-time scheduling, so the maximum column is dominated by whatever\n"
      "  else the machine was doing. It is reported anyway: a control loop is\n"
      "  sized by its worst cycle, not its median.\n");
  return 0;
}
