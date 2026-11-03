# GoogleTest setup
include_directories(BEFORE SYSTEM ${CROBOT_SOURCE_DIR}/unittests/gtest/include)
include_directories(BEFORE SYSTEM ${CROBOT_SOURCE_DIR}/unittests/gtest)
add_library(gtest STATIC gtest/src/gtest-all.cc)
add_library(gtest_main STATIC gtest/src/gtest_main.cc)
target_link_libraries(gtest_main gtest)
if(NOT WIN32)
  target_link_libraries(gtest pthread)
endif()
set_target_properties(
  gtest PROPERTIES
  ARCHIVE_OUTPUT_DIRECTORY ${CROBOT_BINARY_DIR}/lib
)

#===============================================================================
# This function uses following global properties:
# - CROBOT_UNITTESTS
# - CROBOT_${test_type}_TESTS
#
# Usage:
#   crobot_add_test("unit" test_UnitTestA) # assumed source is test_UnitTestA.cpp
#   crobot_add_test("unit" test_UnitTestB test_SourceB1.cpp)
#   crobot_add_test("unit" test_UnitTestA test_SourceC1.cpp test_SourceC2.cpp)
#===============================================================================
function(crobot_add_test test_type target_name) # ARGN for source files

  crobot_property_add(CROBOT_${test_type}_TESTS ${target_name})

  if(${ARGC} GREATER 2)
    set(sources ${ARGN})
  else()
    set(sources "${target_name}.cpp")
  endif()

  add_executable(${target_name} ${sources})
  add_test(${target_name} ${target_name})
  target_link_libraries(${target_name} crobot gtest gtest_main)

  crobot_format_add(${sources})

endfunction()

#===============================================================================
# Usage:
#   crobot_get_tests("comprehensive" compreshensive_tests)
#   foreach(test ${compreshensive_tests})
#     message(STATUS "Test: ${test})
#   endforeach()
#===============================================================================
function(crobot_get_tests output_var test_type)
  get_property(var GLOBAL PROPERTY CROBOT_${test_type}_TESTS)
  set(${output_var} ${var} PARENT_SCOPE)
endfunction()

include_directories(${CMAKE_CURRENT_SOURCE_DIR})
add_subdirectory(unit)

# Print tests
crobot_get_tests(unit_tests "unit")

if(CROBOT_VERBOSE)
  message(STATUS "")
  message(STATUS "[ Tests ]")
  foreach(test ${unit_tests})
    message(STATUS "Adding test: unit/${test}")
  endforeach()
else()
  list(LENGTH unit_tests unit_tests_len)
  math(
    EXPR tests_len
    "${unit_tests_len}"
  )
  message(STATUS "Adding ${tests_len} tests")
endif()

# Add custom target to build all the tests as a single target
add_custom_target(
  tests
  DEPENDS ${unit_tests}
)

# Add custom target to build all the tests and run the tests
add_custom_target(tests_and_run COMMAND ${CMAKE_CTEST_COMMAND}
  DEPENDS ${unit_tests}
)

crobot_format_add(gtest_utils.hpp)
