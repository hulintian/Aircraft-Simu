/** @file provenance.h
 *  @brief 运行证据与 JSON 清单的公共小型接口。
 */
#ifndef COMMON_PROVENANCE_H
#define COMMON_PROVENANCE_H

#include "common/status.h"

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

/** @brief 以固定内存开销计算文件 CRC32 和字节数。
 *
 *  @param path 输入文件路径。
 *  @param crc_out 返回完整文件的 CRC32。
 *  @param size_out 返回文件大小，单位字节。
 *  @return SIM_OK，或文件读取错误。
 */
SimStatus provenance_file_crc32(
    const char *path,
    uint32_t *crc_out,
    uint64_t *size_out);

/** @brief 向 JSON 文件写入一个包含引号的转义字符串值。
 *
 *  控制字符按 JSON 规则转义；输入必须是以 NUL 结尾的 UTF-8/ASCII 字符串。
 */
SimStatus provenance_write_json_string(FILE *file, const char *value);

/** @brief 格式化当前 UTC 墙钟；墙钟只用于 provenance，不得参与仿真计算。 */
void provenance_format_wall_clock_utc(char *out, size_t out_size);

#endif
