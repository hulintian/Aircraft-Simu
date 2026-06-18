/** @file crc32.c
 *  @brief CRC32 校验实现。
 */
#include "common/crc32.h"

/** @brief 计算缓冲区 CRC32。 */
uint32_t crc32_compute(const void *data, size_t size)
{
    const unsigned char *bytes = (const unsigned char *)data;
    uint32_t crc = 0xFFFFFFFFu;
    size_t i;

    if (data == 0 && size != 0u) {
        return 0u;
    }

    for (i = 0; i < size; ++i) {
        uint32_t j;
        crc ^= (uint32_t)bytes[i];
        for (j = 0; j < 8u; ++j) {
            uint32_t mask = 0u - (crc & 1u);
            crc = (crc >> 1u) ^ (0xEDB88320u & mask);
        }
    }

    return ~crc;
}
