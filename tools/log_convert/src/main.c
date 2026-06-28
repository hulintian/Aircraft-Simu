/** @file main.c
 *  @brief 二进制协议日志到 CSV 的转换工具。
 *
 *  该工具读取 environment_sim 写出的 sensor_log.bin 或 command_log.bin。
 *  输入文件由连续的固定线格式协议报文组成，工具逐帧解码、校验实例号和
 *  CRC，并输出便于绘图、审计和回放前处理的 CSV 文本。
 */
#include "common/packet.h"
#include "common/status.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef enum LogKind {
    LOG_KIND_SENSOR = 0,
    LOG_KIND_COMMAND = 1
} LogKind;

typedef struct ConvertOptions {
    /** @brief 输入二进制日志路径。 */
    const char *input_path;
    /** @brief 输出 CSV 路径；为空时输出到 stdout。 */
    const char *output_path;
    /** @brief 期望实例编号。 */
    uint32_t instance_id;
    /** @brief 日志类型。 */
    LogKind kind;
} ConvertOptions;

/** @brief 打印命令行帮助。 */
static void print_usage(const char *argv0)
{
    (void)printf(
        "usage: %s --type sensor|command --instance-id N --input PATH [--output PATH]\n",
        argv0);
}

/** @brief 解析命令行参数。 */
static SimStatus parse_args(int argc, char **argv, ConvertOptions *out)
{
    int i;

    if (out == 0) {
        return SIM_ERR_INVALID_ARG;
    }
    (void)memset(out, 0, sizeof(*out));
    out->kind = LOG_KIND_SENSOR;
    for (i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "--help") == 0) {
            print_usage(argv[0]);
            exit(0);
        }
        if (strcmp(argv[i], "--type") == 0 && (i + 1) < argc) {
            const char *kind = argv[++i];

            if (strcmp(kind, "sensor") == 0) {
                out->kind = LOG_KIND_SENSOR;
            } else if (strcmp(kind, "command") == 0) {
                out->kind = LOG_KIND_COMMAND;
            } else {
                return SIM_ERR_CONFIG;
            }
            continue;
        }
        if (strcmp(argv[i], "--instance-id") == 0 && (i + 1) < argc) {
            out->instance_id = (uint32_t)strtoul(argv[++i], 0, 10);
            continue;
        }
        if (strcmp(argv[i], "--input") == 0 && (i + 1) < argc) {
            out->input_path = argv[++i];
            continue;
        }
        if (strcmp(argv[i], "--output") == 0 && (i + 1) < argc) {
            out->output_path = argv[++i];
            continue;
        }
        return SIM_ERR_CONFIG;
    }
    return out->input_path != 0 ? SIM_OK : SIM_ERR_CONFIG;
}

/** @brief 写出传感器 CSV 表头。 */
static void write_sensor_header(FILE *out)
{
    (void)fprintf(
        out,
        "seq,sim_time,dt,"
        "vel_x,vel_y,vel_z,accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z,"
        "lat_rad,lon_rad,height_m,height_agl_m,range_m,"
        "los_x,los_y,los_z,los_rate_x,los_rate_y,los_rate_z,"
        "closing_velocity,valid_flags,fault_flags\n");
}

/** @brief 写出一帧传感器 CSV 数据。 */
static void write_sensor_row(FILE *out, const SensorFrame *sensor)
{
    (void)fprintf(
        out,
        "%u,%.9f,%.9f,"
        "%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,"
        "%.12f,%.12f,%.9f,%.9f,%.9f,"
        "%.12f,%.12f,%.12f,%.12f,%.12f,%.12f,"
        "%.9f,%u,%u\n",
        sensor->seq,
        sensor->sim_time,
        sensor->dt,
        sensor->missile_vel_ecef_meas.x,
        sensor->missile_vel_ecef_meas.y,
        sensor->missile_vel_ecef_meas.z,
        sensor->missile_accel_ecef_meas.x,
        sensor->missile_accel_ecef_meas.y,
        sensor->missile_accel_ecef_meas.z,
        sensor->missile_gyro_b_meas.x,
        sensor->missile_gyro_b_meas.y,
        sensor->missile_gyro_b_meas.z,
        sensor->missile_lat_rad_meas,
        sensor->missile_lon_rad_meas,
        sensor->missile_height_m_meas,
        sensor->missile_height_agl_m_meas,
        sensor->target_range_meas,
        sensor->target_los_unit_ecef_meas.x,
        sensor->target_los_unit_ecef_meas.y,
        sensor->target_los_unit_ecef_meas.z,
        sensor->target_los_rate_ecef_meas.x,
        sensor->target_los_rate_ecef_meas.y,
        sensor->target_los_rate_ecef_meas.z,
        sensor->target_closing_velocity_meas,
        sensor->sensor_valid_flags,
        sensor->sensor_fault_flags);
}

/** @brief 写出控制命令 CSV 表头。 */
static void write_command_header(FILE *out)
{
    (void)fprintf(
        out,
        "seq,sim_time,"
        "accel_x,accel_y,accel_z,attitude_x,attitude_y,attitude_z,"
        "body_rate_x,body_rate_y,body_rate_z,"
        "actuator_0,actuator_1,actuator_2,actuator_3,"
        "actuator_4,actuator_5,actuator_6,actuator_7,"
        "command_mode,command_status\n");
}

/** @brief 写出一帧控制命令 CSV 数据。 */
static void write_command_row(FILE *out, const ControlCommand *command)
{
    (void)fprintf(
        out,
        "%u,%.9f,"
        "%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,"
        "%.9f,%.9f,%.9f,"
        "%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,"
        "%u,%u\n",
        command->seq,
        command->sim_time,
        command->accel_cmd_ecef.x,
        command->accel_cmd_ecef.y,
        command->accel_cmd_ecef.z,
        command->attitude_cmd.x,
        command->attitude_cmd.y,
        command->attitude_cmd.z,
        command->body_rate_cmd.x,
        command->body_rate_cmd.y,
        command->body_rate_cmd.z,
        command->actuator_cmd[0],
        command->actuator_cmd[1],
        command->actuator_cmd[2],
        command->actuator_cmd[3],
        command->actuator_cmd[4],
        command->actuator_cmd[5],
        command->actuator_cmd[6],
        command->actuator_cmd[7],
        command->command_mode,
        command->command_status);
}

/** @brief 转换传感器日志。 */
static SimStatus convert_sensor_log(FILE *input, FILE *out, uint32_t instance_id)
{
    unsigned char packet[SIM_SENSOR_PACKET_WIRE_SIZE];

    write_sensor_header(out);
    for (;;) {
        const size_t got = fread(packet, 1u, sizeof(packet), input);
        SensorFrame sensor;

        if (got == 0u) {
            return ferror(input) == 0 ? SIM_OK : SIM_ERR_IO;
        }
        if (got != sizeof(packet)) {
            return SIM_ERR_BAD_PACKET;
        }
        if (packet_decode_sensor_frame(packet, sizeof(packet), instance_id, &sensor) != SIM_OK) {
            return SIM_ERR_BAD_PACKET;
        }
        write_sensor_row(out, &sensor);
    }
}

/** @brief 转换控制命令日志。 */
static SimStatus convert_command_log(FILE *input, FILE *out, uint32_t instance_id)
{
    unsigned char packet[SIM_CONTROL_PACKET_WIRE_SIZE];

    write_command_header(out);
    for (;;) {
        const size_t got = fread(packet, 1u, sizeof(packet), input);
        ControlCommand command;

        if (got == 0u) {
            return ferror(input) == 0 ? SIM_OK : SIM_ERR_IO;
        }
        if (got != sizeof(packet)) {
            return SIM_ERR_BAD_PACKET;
        }
        if (packet_decode_control_command(packet, sizeof(packet), instance_id, &command) != SIM_OK) {
            return SIM_ERR_BAD_PACKET;
        }
        write_command_row(out, &command);
    }
}

/** @brief 工具入口。 */
int main(int argc, char **argv)
{
    ConvertOptions options;
    FILE *input;
    FILE *output = stdout;
    SimStatus status;

    status = parse_args(argc, argv, &options);
    if (status != SIM_OK) {
        print_usage(argv[0]);
        return 2;
    }
    input = fopen(options.input_path, "rb");
    if (input == 0) {
        (void)fprintf(stderr, "log_convert: cannot open input %s\n", options.input_path);
        return 1;
    }
    if (options.output_path != 0) {
        output = fopen(options.output_path, "wb");
        if (output == 0) {
            (void)fprintf(stderr, "log_convert: cannot open output %s\n", options.output_path);
            (void)fclose(input);
            return 1;
        }
    }
    status = options.kind == LOG_KIND_SENSOR ?
        convert_sensor_log(input, output, options.instance_id) :
        convert_command_log(input, output, options.instance_id);
    (void)fclose(input);
    if (options.output_path != 0) {
        (void)fclose(output);
    }
    if (status != SIM_OK) {
        (void)fprintf(stderr, "log_convert: conversion failed: %s\n", sim_status_to_string(status));
        return 1;
    }
    return 0;
}
