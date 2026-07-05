add_executable(replay src/main.c)
target_link_libraries(replay PRIVATE missile_flight_control missile_common m)
