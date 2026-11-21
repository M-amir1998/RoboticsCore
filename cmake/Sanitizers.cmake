# Sanitizers are applied to every target in the build (project code, tests and
# test frameworks) because mixing instrumented and uninstrumented code produces
# false positives, especially for AddressSanitizer container annotations.

function(roboticscore_configure_sanitizers)
  if(NOT ROBOTICSCORE_SANITIZERS)
    return()
  endif()

  if(MSVC AND NOT CMAKE_CXX_COMPILER_FRONTEND_VARIANT STREQUAL "GNU")
    # cl.exe supports AddressSanitizer only.
    foreach(san IN LISTS ROBOTICSCORE_SANITIZERS)
      if(san STREQUAL "address")
        add_compile_options(/fsanitize=address /Zi)
        add_link_options(/INCREMENTAL:NO)
        add_compile_definitions(_DISABLE_VECTOR_ANNOTATION _DISABLE_STRING_ANNOTATION)
      else()
        message(WARNING "Sanitizer '${san}' is not supported by MSVC cl.exe; ignored")
      endif()
    endforeach()
    return()
  endif()

  list(JOIN ROBOTICSCORE_SANITIZERS "," san_list)
  add_compile_options(-fsanitize=${san_list} -fno-omit-frame-pointer -fno-sanitize-recover=all -g)
  add_link_options(-fsanitize=${san_list})
  if(MSVC)
    # clang++ on Windows: the ASan runtime requires the release dynamic CRT and
    # STL container annotations must match across translation units.
    set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreadedDLL" PARENT_SCOPE)
    add_compile_definitions(_DISABLE_VECTOR_ANNOTATION _DISABLE_STRING_ANNOTATION)
  endif()
endfunction()
