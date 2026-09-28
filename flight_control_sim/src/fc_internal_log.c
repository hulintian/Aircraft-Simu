/** @file fc_internal_log.c
 *  @brief 飞控内部状态日志固定线格式编解码。
 */
#include "fc/fc_internal_log.h"

#include <stdint.h>
#include <string.h>

/** @brief 写入小端 32 位无符号整数。 */
static void write_u32_le(unsigned char *out, uint32_t value)
{
    out[0] = (unsigned char)(value & UINT32_C(0xff));
    out[1] = (unsigned char)((value >> 8u) & UINT32_C(0xff));
    out[2] = (unsigned char)((value >> 16u) & UINT32_C(0xff));
    out[3] = (unsigned char)((value >> 24u) & UINT32_C(0xff));
}

/** @brief 读取小端 32 位无符号整数。 */
static uint32_t read_u32_le(const unsigned char *data)
{
    return (uint32_t)data[0] |
        ((uint32_t)data[1] << 8u) |
        ((uint32_t)data[2] << 16u) |
        ((uint32_t)data[3] << 24u);
}

/** @brief 写入小端双精度浮点。 */
static void write_f64_le(unsigned char *out, double value)
{
    uint64_t bits;
    unsigned int index;

    (void)memcpy(&bits, &value, sizeof(bits));
    for (index = 0u; index < 8u; ++index) {
        out[index] = (unsigned char)((bits >> (8u * index)) & UINT64_C(0xff));
    }
}

/** @brief 读取小端双精度浮点。 */
static double read_f64_le(const unsigned char *data)
{
    uint64_t bits = UINT64_C(0);
    unsigned int index;
    double value;

    for (index = 0u; index < 8u; ++index) {
        bits |= (uint64_t)data[index] << (8u * index);
    }
    (void)memcpy(&value, &bits, sizeof(value));
    return value;
}

SimStatus fc_internal_log_encode(
    const ControlCommand *command,
    unsigned char *out,
    size_t out_size)
{
    size_t offset = 0u;

    if (command == 0 || out == 0) {
        return SIM_ERR_INVALID_ARG;
    }
    if (out_size < FC_INTERNAL_LOG_WIRE_SIZE) {
        return SIM_ERR_OUT_OF_RANGE;
    }
    write_u32_le(&out[offset], command->seq);
    offset += 4u;
    write_f64_le(&out[offset], command->sim_time);
    offset += 8u;
    write_u32_le(&out[offset], command->command_mode);
    offset += 4u;
    write_u32_le(&out[offset], command->command_status);
    offset += 4u;
    write_f64_le(&out[offset], command->accel_cmd_ecef.x);
    offset += 8u;
    write_f64_le(&out[offset], command->accel_cmd_ecef.y);
    offset += 8u;
    write_f64_le(&out[offset], command->accel_cmd_ecef.z);
    offset += 8u;
    write_f64_le(&out[offset], command->attitude_cmd.x);
    offset += 8u;
    write_f64_le(&out[offset], command->attitude_cmd.y);
    offset += 8u;
    write_f64_le(&out[offset], command->attitude_cmd.z);
    offset += 8u;
    write_f64_le(&out[offset], command->body_rate_cmd.x);
    offset += 8u;
    write_f64_le(&out[offset], command->body_rate_cmd.y);
    offset += 8u;
    write_f64_le(&out[offset], command->body_rate_cmd.z);
    offset += 8u;
    return offset == FC_INTERNAL_LOG_WIRE_SIZE ? SIM_OK : SIM_ERR_INTERNAL;
}

SimStatus fc_internal_log_decode(
    const unsigned char *data,
    size_t data_size,
    ControlCommand *out)
{
    size_t offset = 0u;

    if (data == 0 || out == 0) {
        return SIM_ERR_INVALID_ARG;
    }
    if (data_size != FC_INTERNAL_LOG_WIRE_SIZE) {
        return SIM_ERR_BAD_PACKET;
    }
    (void)memset(out, 0, sizeof(*out));
    out->seq = read_u32_le(&data[offset]);
    offset += 4u;
    out->sim_time = read_f64_le(&data[offset]);
    offset += 8u;
    out->command_mode = read_u32_le(&data[offset]);
    offset += 4u;
    out->command_status = read_u32_le(&data[offset]);
    offset += 4u;
    out->accel_cmd_ecef.x = read_f64_le(&data[offset]);
    offset += 8u;
    out->accel_cmd_ecef.y = read_f64_le(&data[offset]);
    offset += 8u;
    out->accel_cmd_ecef.z = read_f64_le(&data[offset]);
    offset += 8u;
    out->attitude_cmd.x = read_f64_le(&data[offset]);
    offset += 8u;
    out->attitude_cmd.y = read_f64_le(&data[offset]);
    offset += 8u;
    out->attitude_cmd.z = read_f64_le(&data[offset]);
    offset += 8u;
    out->body_rate_cmd.x = read_f64_le(&data[offset]);
    offset += 8u;
    out->body_rate_cmd.y = read_f64_le(&data[offset]);
    offset += 8u;
    out->body_rate_cmd.z = read_f64_le(&data[offset]);
    offset += 8u;
    return offset == FC_INTERNAL_LOG_WIRE_SIZE ? SIM_OK : SIM_ERR_INTERNAL;
}
