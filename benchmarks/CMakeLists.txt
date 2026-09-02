# Deliberately no google-benchmark. The library takes no third-party
# dependencies, and a benchmark that pulls one in cannot be built by someone who
# just cloned the repo -- which is when the numbers are wanted.
add_executable(motionkit_bench bench_core.cpp)
target_link_libraries(motionkit_bench PRIVATE motionkit::core motionkit::warnings)
set_target_properties(motionkit_bench PROPERTIES OUTPUT_NAME motionkit-bench)
