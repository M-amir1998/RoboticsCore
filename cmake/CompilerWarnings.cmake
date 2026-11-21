# Strict warnings for project-owned targets only. Third-party code is consumed as
# SYSTEM includes so its warnings never leak into ours.

function(roboticscore_set_warnings target)
  if(MSVC AND NOT CMAKE_CXX_COMPILER_FRONTEND_VARIANT STREQUAL "GNU")
    set(warnings
        /W4
        /permissive-
        /w14242 # conversion, possible loss of data
        /w14254 # larger bit field to smaller bit field
        /w14263 # member function does not override base virtual
        /w14265 # class has virtual functions but non-virtual destructor
        /w14287 # unsigned/negative constant mismatch
        /w14296 # expression is always true/false
        /w14311 # pointer truncation
        /w14826 # sign-extended conversion
        /w14905 /w14906 # wide/narrow string literal cast
        /w14928 # illegal copy-initialization
    )
    if(ROBOTICSCORE_WARNINGS_AS_ERRORS)
      list(APPEND warnings /WX)
    endif()
    target_compile_options(${target} PRIVATE ${warnings} /utf-8 /Zc:__cplusplus /bigobj)
  else()
    set(warnings
        -Wall
        -Wextra
        -Wpedantic
        -Wshadow
        -Wnon-virtual-dtor
        -Wold-style-cast
        -Wcast-align
        -Wunused
        -Woverloaded-virtual
        -Wdouble-promotion
        -Wformat=2
        -Wimplicit-fallthrough
        -Wmissing-declarations)
    if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
      list(APPEND warnings -Wduplicated-cond -Wlogical-op)
      # -Wuseless-cast is omitted: casts such as uint64_t -> size_t are needed for portability
      # but are flagged on platforms where the two types coincide.
    endif()
    if(ROBOTICSCORE_WARNINGS_AS_ERRORS)
      list(APPEND warnings -Werror)
    endif()
    target_compile_options(${target} PRIVATE ${warnings})
    if(MSVC)
      # clang++ targeting the MSVC ABI: keep the CRT quiet about "unsafe" C functions.
      target_compile_definitions(${target} PRIVATE _CRT_SECURE_NO_WARNINGS)
    endif()
  endif()
endfunction()
