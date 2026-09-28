/** @file fc_internal_log.h
 *  @brief 飞控内部状态日志的固定小端线格式。
 */
#ifndef FC_FC_INTERNAL_LOG_H
#define FC_FC_INTERNAL_LOG_H

#include "common/packet.h"
#include "common/status.h"

#include <stddef.h>

/** @brief 单条飞控内部日志记录的固定字节数。 */
#define FC_INTERNAL_LOG_WIRE_SIZE 92u

/**
 * @brief 将控制链内部状态编码为固定小端记录。
 *
 * 当前记录保存序号、仿真时间、模式、状态位、加速度指令、姿态指令和
 * 角速度指令。调用方拥有输入和输出缓冲区；输出缓冲区至少为
 * `FC_INTERNAL_LOG_WIRE_SIZE` 字节。
 */
SimStatus fc_internal_log_encode(
    const ControlCommand *command,
    unsigned char *out,
    size_t out_size);

/**
 * @brief 从固定小端记录解码飞控内部状态。
 *
 * 输入必须恰好包含 `FC_INTERNAL_LOG_WIRE_SIZE` 字节。未写入内部日志的
 * 执行机构字段在输出中置零。
 */
SimStatus fc_internal_log_decode(
    const unsigned char *data,
    size_t data_size,
    ControlCommand *out);

#endif
