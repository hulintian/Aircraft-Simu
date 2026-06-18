set(COMMON_HEADERS
    include/common/build_info.h
    include/common/config.h
    include/common/crc32.h
    include/common/logger.h
    include/common/math_constants.h
    include/common/matrix3.h
    include/common/packet.h
    include/common/protocol.h
    include/common/quaternion.h
    include/common/random.h
    include/common/ring_buffer.h
    include/common/sim_time.h
    include/common/status.h
    include/common/vec3.h
)

set(COMMON_SOURCES
    src/config.c
    src/crc32.c
    src/logger.c
    src/matrix3.c
    src/packet.c
    src/protocol.c
    src/quaternion.c
    src/random.c
    src/ring_buffer.c
    src/sim_time.c
    src/status.c
    src/vec3.c
)

add_library(missile_common STATIC ${COMMON_HEADERS} ${COMMON_SOURCES})
target_include_directories(missile_common PUBLIC include)

add_executable(common_tests tests/common_tests.c)
target_link_libraries(common_tests PRIVATE missile_common m)
add_test(NAME common_tests COMMAND common_tests)
