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
  test_se3.cpp)
target_link_libraries(motionkit_tests
  PRIVATE motionkit::core motionkit::warnings GTest::gtest_main)
gtest_discover_tests(motionkit_tests
  PROPERTIES LABELS unit
  DISCOVERY_TIMEOUT 30)
