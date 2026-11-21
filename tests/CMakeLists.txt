# Test suites are grouped by intent and exposed to CTest with labels:
#   unit         fast, isolated API behaviour
#   numerical    conditioning, degenerate inputs, discontinuities, randomized properties
#   integration  several modules working together in simulation
#   regression   deterministic scenarios checked against stored expected results
#
#   ctest --preset debug -L unit
#
# Test sources are discovered with CONFIGURE_DEPENDS globbing so adding a test file
# does not require editing this list; library sources remain explicitly listed.

add_library(roboticscore_test_support INTERFACE)
target_include_directories(roboticscore_test_support INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/support)
target_link_libraries(roboticscore_test_support INTERFACE RoboticsCore::RoboticsCore GTest::gtest
                                                          GTest::gtest_main)
target_compile_definitions(roboticscore_test_support
                           INTERFACE ROBOTICSCORE_TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/data")

function(roboticscore_add_test_suite category)
  file(GLOB_RECURSE sources CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/${category}/*.cpp)
  if(NOT sources)
    return()
  endif()
  set(target roboticscore_${category}_tests)
  roboticscore_add_executable(${target} SOURCES ${sources} DEPS roboticscore_test_support)
  gtest_discover_tests(
    ${target}
    DISCOVERY_MODE PRE_TEST
    TEST_PREFIX "${category}."
    PROPERTIES LABELS ${category} TIMEOUT 300)
endfunction()

roboticscore_add_test_suite(unit)
roboticscore_add_test_suite(numerical)
roboticscore_add_test_suite(integration)
roboticscore_add_test_suite(regression)
