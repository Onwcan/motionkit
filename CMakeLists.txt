cmake_minimum_required(VERSION 3.24)

project(motionkit
  VERSION 0.1.0
  DESCRIPTION "Robot kinematics, dynamics, trajectory and calibration core"
  LANGUAGES CXX)

# --- project-is-top-level detection -----------------------------------------
if(CMAKE_SOURCE_DIR STREQUAL PROJECT_SOURCE_DIR)
  set(MOTIONKIT_IS_TOP_LEVEL ON)
else()
  set(MOTIONKIT_IS_TOP_LEVEL OFF)
endif()

option(MOTIONKIT_BUILD_TESTS      "Build unit tests"           ${MOTIONKIT_IS_TOP_LEVEL})
option(MOTIONKIT_BUILD_ALLOCATION_TESTS
       "Build global-allocation probe tests"                   ON)
option(MOTIONKIT_BUILD_BENCHMARKS "Build micro-benchmarks"     OFF)
option(MOTIONKIT_ENABLE_WERROR    "Treat warnings as errors"   ON)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "" FORCE)
endif()

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

# --- warning interface -------------------------------------------------------
add_library(motionkit_warnings INTERFACE)
add_library(motionkit::warnings ALIAS motionkit_warnings)
target_compile_options(motionkit_warnings INTERFACE
  $<$<CXX_COMPILER_ID:GNU,Clang,AppleClang>:
    -Wall -Wextra -Wpedantic -Wshadow -Wconversion -Wsign-conversion
    -Wnon-virtual-dtor -Wold-style-cast -Wcast-align -Wunused
    -Woverloaded-virtual -Wdouble-promotion -Wformat=2 -Wnull-dereference
    $<$<BOOL:${MOTIONKIT_ENABLE_WERROR}>:-Werror>>
  $<$<CXX_COMPILER_ID:MSVC>:/W4 $<$<BOOL:${MOTIONKIT_ENABLE_WERROR}>:/WX>>)

# --- core library ------------------------------------------------------------
add_library(motionkit_core
  src/core/so3.cpp
  src/core/se3.cpp
  src/core/frame_graph.cpp
  src/core/trajectory.cpp)
add_library(motionkit::core ALIAS motionkit_core)

target_include_directories(motionkit_core
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_compile_features(motionkit_core PUBLIC cxx_std_20)
target_link_libraries(motionkit_core PRIVATE motionkit::warnings)
set_target_properties(motionkit_core PROPERTIES
  OUTPUT_NAME motionkit-core
  # Without EXPORT_NAME the installed target is motionkit::motionkit_core, while
  # in-tree consumers see the motionkit::core ALIAS above. The two names then
  # disagree and every downstream find_package() breaks -- which no unit test
  # would ever notice. The install-consumer CI job exists to catch exactly this.
  EXPORT_NAME core
  VERSION   ${PROJECT_VERSION}
  SOVERSION ${PROJECT_VERSION_MAJOR}
  CXX_VISIBILITY_PRESET hidden
  VISIBILITY_INLINES_HIDDEN ON)
set_target_properties(motionkit_warnings PROPERTIES EXPORT_NAME warnings)

# --- tests -------------------------------------------------------------------
if(MOTIONKIT_BUILD_TESTS)
  enable_testing()
  add_subdirectory(tests)
endif()

if(MOTIONKIT_BUILD_BENCHMARKS)
  add_subdirectory(benchmarks)
endif()

# --- install / export --------------------------------------------------------
install(TARGETS motionkit_core motionkit_warnings
        EXPORT motionkitTargets
        LIBRARY  DESTINATION ${CMAKE_INSTALL_LIBDIR}
        ARCHIVE  DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME  DESTINATION ${CMAKE_INSTALL_BINDIR})
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(EXPORT motionkitTargets
        FILE motionkitTargets.cmake
        NAMESPACE motionkit::
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/motionkit)

write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/motionkitConfigVersion.cmake"
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY SameMajorVersion)
configure_package_config_file(
  "${PROJECT_SOURCE_DIR}/cmake/motionkitConfig.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}/motionkitConfig.cmake"
  INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/motionkit)
install(FILES
  "${CMAKE_CURRENT_BINARY_DIR}/motionkitConfig.cmake"
  "${CMAKE_CURRENT_BINARY_DIR}/motionkitConfigVersion.cmake"
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/motionkit)
