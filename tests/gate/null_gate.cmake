# The null baseline (docs/plan/bring-up/, acceptance 4): sitl_null runs the
# scenario to its duration with no error, and score_log scores it as timed
# out, the vehicle never having left the pad.
#
# Inputs: SITL, SCORE (executables), SCENARIO, WORK_DIR.

file(REMOVE_RECURSE ${WORK_DIR})
file(MAKE_DIRECTORY ${WORK_DIR})

function(run out_var)
    execute_process(COMMAND ${ARGN} RESULT_VARIABLE rc OUTPUT_VARIABLE out ERROR_VARIABLE err)
    message(STATUS "${out}${err}")
    if(NOT rc EQUAL 0)
        message(FATAL_ERROR "command failed (${rc}): ${ARGN}")
    endif()
    set(${out_var} "${out}" PARENT_SCOPE)
endfunction()

run(sitl_out ${SITL} --scenario ${SCENARIO} --out ${WORK_DIR}/null.brlog)
if(NOT sitl_out MATCHES ", ([0-9]+) ticks,")
    message(FATAL_ERROR "cannot read the tick count from: ${sitl_out}")
endif()
# 5 s at the null baseline's 100 Hz: the run went to its duration.
if(NOT CMAKE_MATCH_1 EQUAL 500)
    message(FATAL_ERROR "${CMAKE_MATCH_1} ticks, expected 500 (the scenario's 5 s at 100 Hz)")
endif()

run(score_out ${SCORE} --log ${WORK_DIR}/null.brlog)
foreach(expected "\"outcome\":\"timed_out\"" "\"path_length_m\":0.000000" "\"max_speed_m_s\":0.000000")
    string(FIND "${score_out}" "${expected}" at)
    if(at EQUAL -1)
        message(FATAL_ERROR "the score lacks ${expected}: ${score_out}")
    endif()
endforeach()

message(STATUS "gate null: passed")
