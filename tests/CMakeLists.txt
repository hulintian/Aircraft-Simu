add_executable(closed_loop_test closed_loop_test.c)
target_link_libraries(closed_loop_test PRIVATE missile_common m)

add_test(
    NAME closed_loop_test
    COMMAND closed_loop_test
        $<TARGET_FILE:flight_control_sim>
        $<TARGET_FILE:environment_sim>
        ${CMAKE_SOURCE_DIR}/configs/baseline/flight_control.json
        ${CMAKE_SOURCE_DIR}/configs/baseline/scenario.json
)
set_tests_properties(closed_loop_test PROPERTIES TIMEOUT 15)
