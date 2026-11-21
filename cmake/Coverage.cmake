# gcov/lcov-compatible coverage instrumentation for GCC and Clang (GNU driver).

function(roboticscore_configure_coverage)
  if(NOT ROBOTICSCORE_ENABLE_COVERAGE)
    return()
  endif()
  if(MSVC AND NOT CMAKE_CXX_COMPILER_FRONTEND_VARIANT STREQUAL "GNU")
    message(WARNING "Coverage instrumentation is not supported with cl.exe; ignored")
    return()
  endif()
  add_compile_options(--coverage -O0 -g)
  add_link_options(--coverage)
endfunction()
