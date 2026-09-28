add_executable(closed_loop_test closed_loop_test.c)
target_link_libraries(closed_loop_test PRIVATE missile_environment missile_flight_control missile_common m)

add_test(
    NAME closed_loop_test
    COMMAND closed_loop_test
        $<TARGET_FILE:flight_control_sim>
        $<TARGET_FILE:environment_sim>
        ${CMAKE_SOURCE_DIR}/configs/baseline/flight_control.json
        ${CMAKE_SOURCE_DIR}/configs/baseline/scenario.json
        $<TARGET_FILE:replay>
        $<TARGET_FILE:compare_logs>
        ${CMAKE_SOURCE_DIR}/tests/fixtures/dem/map_manifest.json
        ${CMAKE_SOURCE_DIR}/tests/fixtures/dem/everest_terrain_9x9.txt
        ${CMAKE_SOURCE_DIR}/configs/verification/log_compare_exact.json
)
set_tests_properties(closed_loop_test PROPERTIES TIMEOUT 15)

add_executable(instance_manager_test instance_manager_test.c)
add_test(
    NAME instance_manager_test
    COMMAND instance_manager_test
        $<TARGET_FILE:instance_manager>
        ${CMAKE_SOURCE_DIR}
        $<TARGET_FILE:environment_sim>
        $<TARGET_FILE:flight_control_sim>
)
set_tests_properties(instance_manager_test PROPERTIES TIMEOUT 30)

add_executable(log_convert_test log_convert_test.c)
target_link_libraries(log_convert_test PRIVATE missile_flight_control missile_common m)
add_test(
    NAME log_convert_test
    COMMAND log_convert_test
        $<TARGET_FILE:log_convert>
)
set_tests_properties(log_convert_test PROPERTIES TIMEOUT 10)

add_executable(p8_tools_test p8_tools_test.c)
target_link_libraries(p8_tools_test PRIVATE missile_common m)
add_test(
    NAME p8_tools_test
    COMMAND p8_tools_test
        $<TARGET_FILE:replay>
        $<TARGET_FILE:compare_logs>
        $<TARGET_FILE:batch_stats>
        ${CMAKE_SOURCE_DIR}/configs/baseline/flight_control.json
        ${CMAKE_SOURCE_DIR}/configs/verification/log_compare_tolerance.json
        ${CMAKE_SOURCE_DIR}/configs/verification/log_compare_exact.json
)
set_tests_properties(p8_tools_test PROPERTIES TIMEOUT 10)

add_executable(map_preprocess_test map_preprocess_test.c)
target_link_libraries(map_preprocess_test PRIVATE missile_environment missile_common m)
add_test(
    NAME map_preprocess_test
    COMMAND map_preprocess_test
        $<TARGET_FILE:map_preprocess>
        ${CMAKE_SOURCE_DIR}
)
set_tests_properties(map_preprocess_test PROPERTIES TIMEOUT 10)

add_executable(batch_runner_test batch_runner_test.c)
add_test(
    NAME batch_runner_test
    COMMAND batch_runner_test
        $<TARGET_FILE:batch_runner>
)
set_tests_properties(batch_runner_test PROPERTIES TIMEOUT 10)

add_executable(control_quality_test control_quality_test.c)
add_test(
    NAME control_quality_test
    COMMAND control_quality_test
        $<TARGET_FILE:control_quality_report>
        ${CMAKE_SOURCE_DIR}/configs/baseline/flight_control.json
        ${CMAKE_SOURCE_DIR}/configs/verification/control_quality.json
)
set_tests_properties(control_quality_test PROPERTIES TIMEOUT 10)

add_executable(campaign_pressure_test long_campaign_test.c)
add_test(
    NAME campaign_pressure_short_test
    COMMAND campaign_pressure_test
        $<TARGET_FILE:instance_manager>
        ${CMAKE_SOURCE_DIR}
        $<TARGET_FILE:environment_sim>
        $<TARGET_FILE:flight_control_sim>
        4
        2
        short
)
set_tests_properties(campaign_pressure_short_test PROPERTIES TIMEOUT 120)

if(MISSILE_ENABLE_MEDIUM_TESTS)
    add_test(
        NAME campaign_pressure_medium_test
        COMMAND campaign_pressure_test
            $<TARGET_FILE:instance_manager>
            ${CMAKE_SOURCE_DIR}
            $<TARGET_FILE:environment_sim>
            $<TARGET_FILE:flight_control_sim>
            16
            4
            medium
    )
    set_tests_properties(campaign_pressure_medium_test PROPERTIES TIMEOUT 300)
endif()

if(MISSILE_ENABLE_LONG_TESTS)
    add_test(
        NAME campaign_pressure_long_test
        COMMAND campaign_pressure_test
            $<TARGET_FILE:instance_manager>
            ${CMAKE_SOURCE_DIR}
            $<TARGET_FILE:environment_sim>
            $<TARGET_FILE:flight_control_sim>
            128
            8
            long
    )
    set_tests_properties(campaign_pressure_long_test PROPERTIES TIMEOUT 1500)
endif()
