set(FC_HEADERS
    include/fc/autopilot.h
    include/fc/command_manager.h
    include/fc/estimator.h
    include/fc/fc_app.h
    include/fc/fc_config.h
    include/fc/fc_context.h
    include/fc/fc_health.h
    include/fc/fc_modes.h
    include/fc/fc_scheduler.h
    include/fc/fc_state.h
    include/fc/guidance_manager.h
    include/fc/guidance_png.h
    include/fc/navigation.h
    include/fc/safety_monitor.h
)

set(FC_LIB_SOURCES
    src/autopilot.c
    src/command_manager.c
    src/estimator.c
    src/fc_health.c
    src/fc_modes.c
    src/fc_scheduler.c
    src/fc_state.c
    src/guidance_manager.c
    src/guidance_png.c
    src/navigation.c
    src/safety_monitor.c
)

set(FC_APP_SOURCES
    src/fc_app.c
    src/main.c
)

add_library(missile_flight_control STATIC ${FC_HEADERS} ${FC_LIB_SOURCES})
target_include_directories(missile_flight_control PUBLIC include)
target_link_libraries(missile_flight_control PUBLIC missile_common m)

add_executable(flight_control_sim ${FC_APP_SOURCES})
target_link_libraries(flight_control_sim PRIVATE missile_flight_control missile_common m)

add_executable(flight_control_tests tests/fc_tests.c)
target_link_libraries(flight_control_tests PRIVATE missile_flight_control missile_common m)
add_test(NAME flight_control_tests COMMAND flight_control_tests)
