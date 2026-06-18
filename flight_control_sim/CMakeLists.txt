set(FC_HEADERS
    include/fc/fc_app.h
    include/fc/fc_config.h
    include/fc/fc_context.h
)

set(FC_SOURCES
    src/fc_app.c
    src/guidance_png.c
    src/main.c
)

add_executable(flight_control_sim ${FC_HEADERS} ${FC_SOURCES})
target_include_directories(flight_control_sim PRIVATE include)
target_link_libraries(flight_control_sim PRIVATE missile_common m)
