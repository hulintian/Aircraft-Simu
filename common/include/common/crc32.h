/** @file crc32.h
 *  @brief CRC32 数据完整性校验接口。
 *
 *  通信报文和地形瓦片使用同一实现校验载荷。函数不保存全局状态，
 *  因此相同字节序列始终得到相同结果。
 */
#ifndef COMMON_CRC32_H
#define COMMON_CRC32_H

#include <stddef.h>
#include <stdint.h>

/** @brief 初始化可增量更新的 CRC32 状态。 */
uint32_t crc32_init(void);

/** @brief 向 CRC32 状态追加一段字节。
 *
 *  @param state 由 crc32_init 或前一次 crc32_update 返回的未取反状态。
 *  @param data 数据起始地址；当 @p size 为零时允许为空。
 *  @param size 数据长度，单位字节。
 *  @return 更新后的未取反状态；参数非法时返回 0。
 */
uint32_t crc32_update(uint32_t state, const void *data, size_t size);

/** @brief 完成增量 CRC32 计算并返回标准 CRC32 值。 */
uint32_t crc32_finalize(uint32_t state);

/** @brief 计算连续字节数据的 CRC32。
 *
 *  @param data 数据起始地址；当 @p size 为零时允许为空。
 *  @param size 数据长度，单位字节。
 *  @return CRC32 校验值。
 */
uint32_t crc32_compute(const void *data, size_t size);

#endif
