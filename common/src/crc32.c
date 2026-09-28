/** @file crc32.c
 *  @brief CRC32 校验实现。
 */
#include "common/crc32.h"

uint32_t crc32_init(void)
{
    return UINT32_C(0xFFFFFFFF);
}

uint32_t crc32_update(uint32_t state, const void *data, size_t size)
{
    const unsigned char *bytes = (const unsigned char *)data;
    size_t i;

    if (data == 0 && size != 0u) {
        return 0u;
    }

    for (i = 0; i < size; ++i) {
        uint32_t j;
        state ^= (uint32_t)bytes[i];
        for (j = 0; j < 8u; ++j) {
            uint32_t mask = 0u - (state & 1u);
            state = (state >> 1u) ^ (UINT32_C(0xEDB88320) & mask);
        }
    }

    return state;
}

uint32_t crc32_finalize(uint32_t state)
{
    return ~state;
}

/** @brief 计算缓冲区 CRC32。 */
uint32_t crc32_compute(const void *data, size_t size)
{
    if (data == 0 && size != 0u) {
        return 0u;
    }

    return crc32_finalize(crc32_update(crc32_init(), data, size));
}
