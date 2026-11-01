# trace_export writes a SITL log as a reflex-trace/1 replay with the field as
# its scene (apps/trace_export/main.cpp, sim/replay_trace.hpp): the vehicle's
# channels, every obstacle as a cylinder drawn as a tree, the goal and the
# geofence, and takeoff and the outcome as events; it refuses a log that
# wasn't recorded from the scenario it is given.
#
# Inputs: SITL, EXPORT (executables), SOURCE_DIR, WORK_DIR.

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

function(refused pattern)
    execute_process(COMMAND ${ARGN} RESULT_VARIABLE rc OUTPUT_VARIABLE out ERROR_VARIABLE err)
    if(NOT rc EQUAL 1)
        message(FATAL_ERROR "expected exit code 1, got ${rc}: ${ARGN}")
    endif()
    if(NOT err MATCHES "${pattern}")
        message(FATAL_ERROR "expected an error matching '${pattern}', got: ${err}")
    endif()
endfunction()

# s0-determinism flies 8 s into the forest at seed 1700, and collides.
set(scenario ${SOURCE_DIR}/scenarios/s0-determinism.yaml)
run(sitl_out ${SITL} --scenario ${scenario} --out ${WORK_DIR}/run.brlog)
run(out ${EXPORT} --log ${WORK_DIR}/run.brlog --scenario ${scenario} --out ${WORK_DIR}/a.json --commit abc1234)
if(NOT out MATCHES "trace_export: ([0-9]+) samples ->")
    message(FATAL_ERROR "cannot read the sample count from: ${out}")
endif()
# The collision settles the outcome early; the replay ends 2 s after it.
if(CMAKE_MATCH_1 LESS 20 OR CMAKE_MATCH_1 GREATER 161)
    message(FATAL_ERROR "expected the replay to end 2 s after the collision, got ${CMAKE_MATCH_1} samples")
endif()

file(READ ${WORK_DIR}/a.json json)
foreach(needle
        "\"schema\": \"reflex-trace/1\""
        "\"commit\": \"abc1234\""
        "\"block\": \"s0-gt-pid\""
        "\"seed\": 1700"
        "\"position_setpoint\": \\["
        "\"rotor_thrusts\": \\["
        "\"scene\": { \"entities\": \\["
        "{ \"id\": \"geofence\", \"role\": \"boundary\""
        "{ \"id\": \"obstacle-0\", \"role\": \"obstacle\", \"label\": \"obstacle 0\", \"shape\": { \"type\": \"cylinder\""
        "\"visual\": { \"kind\": \"tree\" }"
        "{ \"id\": \"goal\", \"role\": \"goal\""
        "\"label\": \"takeoff\""
        "\"label\": \"obstacle collision \\(obstacle [0-9]+\\)\"")
    if(NOT json MATCHES "${needle}")
        message(FATAL_ERROR "the replay lacks ${needle}")
    endif()
endforeach()
# One cylinder per obstacle of the field the log records: forest-60 grows 54.
string(REGEX MATCHALL "\"role\": \"obstacle\"" obstacles "${json}")
list(LENGTH obstacles n)
if(NOT n EQUAL 54)
    message(FATAL_ERROR "${n} obstacles in the scene, expected the forest's 54")
endif()

# A log recorded from another scenario is refused.
refused("was not recorded from" ${EXPORT} --log ${WORK_DIR}/run.brlog
    --scenario ${SOURCE_DIR}/scenarios/s0-open.yaml --out ${WORK_DIR}/b.json)

message(STATUS "trace_export: passed")
