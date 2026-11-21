include(GNUInstallDirs)

# roboticscore_add_library(<name> SOURCES ... [PUBLIC_DEPS ...] [PRIVATE_DEPS ...])
#
# Creates the static library target `roboticscore_<name>` with the alias and
# exported name `RoboticsCore::<name>`, applies project warnings and C++20, and
# records the target for installation.
function(roboticscore_add_library name)
  cmake_parse_arguments(ARG "" "" "SOURCES;PUBLIC_DEPS;PRIVATE_DEPS" ${ARGN})
  set(target roboticscore_${name})

  add_library(${target} ${ARG_SOURCES})
  add_library(RoboticsCore::${name} ALIAS ${target})

  set_target_properties(
    ${target}
    PROPERTIES EXPORT_NAME ${name}
               OUTPUT_NAME roboticscore_${name}
               POSITION_INDEPENDENT_CODE ON)

  target_include_directories(
    ${target}
    PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
           $<BUILD_INTERFACE:${PROJECT_BINARY_DIR}/include>
           $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
  target_compile_features(${target} PUBLIC cxx_std_20)
  target_link_libraries(${target} PUBLIC ${ARG_PUBLIC_DEPS} PRIVATE ${ARG_PRIVATE_DEPS})

  roboticscore_set_warnings(${target})
  roboticscore_maybe_enable_clang_tidy(${target})

  set_property(GLOBAL APPEND PROPERTY ROBOTICSCORE_LIBRARY_TARGETS ${target})
endfunction()

# Applies clang-tidy to a project target when ROBOTICSCORE_ENABLE_CLANG_TIDY is ON.
function(roboticscore_maybe_enable_clang_tidy target)
  if(NOT ROBOTICSCORE_ENABLE_CLANG_TIDY)
    return()
  endif()
  find_program(ROBOTICSCORE_CLANG_TIDY_EXE NAMES clang-tidy REQUIRED)
  set_target_properties(${target} PROPERTIES CXX_CLANG_TIDY "${ROBOTICSCORE_CLANG_TIDY_EXE}")
endfunction()

# roboticscore_add_executable(<name> SOURCES ... [DEPS ...])
# Project-owned executable (demo, tool) with the project warning set.
function(roboticscore_add_executable name)
  cmake_parse_arguments(ARG "" "" "SOURCES;DEPS" ${ARGN})
  add_executable(${name} ${ARG_SOURCES})
  target_link_libraries(${name} PRIVATE ${ARG_DEPS})
  roboticscore_set_warnings(${name})
  roboticscore_maybe_enable_clang_tidy(${name})
endfunction()
