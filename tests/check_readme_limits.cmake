file(READ "${PROJECT_SOURCE_DIR}/README.md" readme)

function(check_limit name header)
  file(READ "${PROJECT_SOURCE_DIR}/include/${header}" declaration)
  string(REGEX MATCH "${name} = ([0-9]+)" match "${declaration}")
  if(NOT match)
    message(FATAL_ERROR "Could not find ${name} in ${header}")
  endif()
  set(value "${CMAKE_MATCH_1}")
  string(FIND "${readme}" "| `${name}` | `${header}` | ${value} |" documented)
  if(documented EQUAL -1)
    message(FATAL_ERROR
      "README limit for ${name} does not match ${header} (${value})")
  endif()
endfunction()

check_limit(kMaxPathKnots motionkit/core/cartesian.hpp)
check_limit(kMaxWaypoints motionkit/core/cartesian.hpp)
check_limit(kMaxObstacles motionkit/core/collision.hpp)
