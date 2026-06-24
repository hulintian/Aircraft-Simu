set(ENV_HEADERS
    include/env/env_app.h
    include/env/env_config.h
    include/env/env_context.h
)

set(ENV_MODEL_HEADERS
    include/env/actuator_model.h
    include/env/aero_model.h
    include/env/atmosphere_model.h
    include/env/earth_model.h
    include/env/environment_force_model.h
    include/env/fault_injection.h
    include/env/geo_coordinate.h
    include/env/gravity_model.h
    include/env/map_tile.h
    include/env/mass_model.h
    include/env/missile_plant_6dof.h
    include/env/propulsion_model.h
    include/env/sensor_accel.h
    include/env/sensor_imu.h
    include/env/sensor_noise.h
    include/env/sensor_seeker.h
    include/env/sensor_speed.h
    include/env/sensor_vector3.h
    include/env/terrain_model.h
)

set(ENV_MODEL_SOURCES
    src/actuator_model.c
    src/aero_model.c
    src/atmosphere_model.c
    src/earth_model.c
    src/environment_force_model.c
    src/fault_injection.c
    src/geo_coordinate.c
    src/gravity_model.c
    src/map_tile.c
    src/mass_model.c
    src/missile_plant_6dof.c
    src/propulsion_model.c
    src/sensor_accel.c
    src/sensor_imu.c
    src/sensor_noise.c
    src/sensor_seeker.c
    src/sensor_speed.c
    src/sensor_vector3.c
    src/terrain_model.c
)

set(ENV_SOURCES
    src/env_app.c
    src/main.c
)

add_library(missile_environment STATIC ${ENV_MODEL_HEADERS} ${ENV_MODEL_SOURCES})
target_include_directories(missile_environment PUBLIC include)
target_link_libraries(missile_environment PUBLIC missile_common m)

add_executable(environment_sim ${ENV_HEADERS} ${ENV_SOURCES})
target_include_directories(environment_sim PRIVATE include)
target_link_libraries(environment_sim PRIVATE missile_environment missile_common m)

add_executable(environment_tests tests/environment_tests.c)
target_link_libraries(environment_tests PRIVATE missile_environment missile_common m)
add_test(NAME environment_tests COMMAND environment_tests)
