include(FetchContent)

FetchContent_Declare(googletest
  GIT_REPOSITORY https://github.com/google/googletest.git
  GIT_TAG        v1.17.0
  GIT_SHALLOW    TRUE
  EXCLUDE_FROM_ALL)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

include(GoogleTest)

add_executable(motionkit_tests
  test_vec3.cpp
  test_so3.cpp
  test_se3.cpp
  test_frame_graph.cpp
  test_frame_graph_realtime.cpp
  test_trajectory.cpp)
target_link_libraries(motionkit_tests
  PRIVATE motionkit::core motionkit::warnings GTest::gtest_main)
gtest_discover_tests(motionkit_tests
  PROPERTIES LABELS unit
  DISCOVERY_TIMEOUT 30)

if(MOTIONKIT_BUILD_ALLOCATION_TESTS)
  add_executable(motionkit_allocation_tests
    test_no_allocation.cpp)
  target_link_libraries(motionkit_allocation_tests
    PRIVATE motionkit::core motionkit::warnings GTest::gtest_main)

  # This target deliberately implements global new/delete with matching C
  # allocation functions so it can count allocations. GCC's local diagnostic
  # does not follow that replacement pair and reports the intentional free as
  # a mismatch. Keep the suppression on this one probe binary only.
  target_compile_options(motionkit_allocation_tests PRIVATE
    $<$<CXX_COMPILER_ID:GNU>:-Wno-mismatched-new-delete>)

  gtest_discover_tests(motionkit_allocation_tests
    PROPERTIES LABELS unit
    DISCOVERY_TIMEOUT 30)
endif()
