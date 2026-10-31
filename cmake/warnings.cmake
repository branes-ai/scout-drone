# cmake/warnings.cmake — strict warnings for scout-drone's own targets.
#
# Applied to the mission library, the harness executables, and the tests.
# Warnings are errors, as in the sibling repos.
include_guard(GLOBAL)

function(scout_set_warnings target)
    if(MSVC)
        target_compile_options(${target} PRIVATE /W4 /permissive-)
    else()
        target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic -Wshadow -Wconversion)
    endif()
    set_target_properties(${target} PROPERTIES COMPILE_WARNING_AS_ERROR ON)
endfunction()
