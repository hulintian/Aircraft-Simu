set(INSTANCE_MANAGER_SOURCES
    src/main.c
)

add_executable(instance_manager ${INSTANCE_MANAGER_SOURCES})
target_link_libraries(instance_manager PRIVATE missile_common m)
