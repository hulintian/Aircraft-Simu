/** @file main.c
 *  @brief 二进制协议日志到 CSV 的转换工具。
 *
 *  该工具读取 environment_sim 写出的 sensor_log.bin 或 command_log.bin。
 *  输入文件由连续的固定线格式协议报文组成，工具逐帧解码、校验实例号和
 *  CRC，并输出便于绘图、审计和回放前处理的 CSV 文本。
 */
#include "common/build_info.h"
#include "common/packet.h"
#include "common/provenance.h"
#include "common/status.h"
#include "fc/fc_internal_log.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef enum LogKind {
    LOG_KIND_SENSOR = 0,
    LOG_KIND_COMMAND = 1,
    LOG_KIND_FC_INTERNAL = 2
} LogKind;

typedef struct ConvertOptions {
    /** @brief 输入二进制日志路径。 */
    const char *input_path;
    /** @brief 输出 CSV 路径；为空时输出到 stdout。 */
    const char *output_path;
    /** @brief 被动回放运行清单路径。 */
    const char *manifest_path;
    /** @brief 自动生成的清单路径存储。 */
    char default_manifest_path[1024];
    /** @brief 期望实例编号。 */
    uint32_t instance_id;
    /** @brief 日志类型。 */
    LogKind kind;
} ConvertOptions;

/** @brief 打印命令行帮助。 */
static void print_usage(const char *argv0)
{
    (void)printf(
        "usage: %s --type sensor|command|fc-internal --instance-id N --input PATH "
        "[--output PATH] [--manifest PATH]\n",
        argv0);
}

static int parse_uint32_arg(const char *text, uint32_t *out)
{
    char *end = 0;
    unsigned long value;

    if (text == 0 || out == 0 || text[0] == '\0' || text[0] == '-') {
        return 0;
    }
    value = strtoul(text, &end, 10);
    if (end == text || *end != '\0' || value > UINT32_MAX) {
        return 0;
    }
    *out = (uint32_t)value;
    return 1;
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
            } else if (strcmp(kind, "fc-internal") == 0) {
                out->kind = LOG_KIND_FC_INTERNAL;
            } else {
                return SIM_ERR_CONFIG;
            }
            continue;
        }
        if (strcmp(argv[i], "--instance-id") == 0 && (i + 1) < argc) {
            if (!parse_uint32_arg(argv[++i], &out->instance_id)) {
                return SIM_ERR_CONFIG;
            }
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
        if (strcmp(argv[i], "--manifest") == 0 && (i + 1) < argc) {
            out->manifest_path = argv[++i];
            continue;
        }
        return SIM_ERR_CONFIG;
    }
    if (out->input_path == 0) {
        return SIM_ERR_CONFIG;
    }
    if (out->manifest_path == 0) {
        const char *base_path = out->output_path != 0 ? out->output_path : out->input_path;
        const char *suffix = out->output_path != 0 ? ".run_manifest.json" :
            ".replay_passive.run_manifest.json";
        const int written = snprintf(
            out->default_manifest_path,
            sizeof(out->default_manifest_path),
            "%s%s",
            base_path,
            suffix);

        if (written < 0 || (size_t)written >= sizeof(out->default_manifest_path)) {
            return SIM_ERR_OUT_OF_RANGE;
        }
        out->manifest_path = out->default_manifest_path;
    }
    return SIM_OK;
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
static SimStatus convert_sensor_log(
    FILE *input,
    FILE *out,
    uint32_t instance_id,
    uint64_t *record_count)
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
        ++*record_count;
    }
}

/** @brief 转换控制命令日志。 */
static SimStatus convert_command_log(
    FILE *input,
    FILE *out,
    uint32_t instance_id,
    uint64_t *record_count)
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
        ++*record_count;
    }
}

/** @brief 转换飞控内部状态日志。 */
static SimStatus convert_fc_internal_log(FILE *input, FILE *out, uint64_t *record_count)
{
    unsigned char record[FC_INTERNAL_LOG_WIRE_SIZE];

    write_command_header(out);
    for (;;) {
        const size_t got = fread(record, 1u, sizeof(record), input);
        ControlCommand command;

        if (got == 0u) {
            return ferror(input) == 0 ? SIM_OK : SIM_ERR_IO;
        }
        if (got != sizeof(record) ||
            fc_internal_log_decode(record, sizeof(record), &command) != SIM_OK) {
            return SIM_ERR_BAD_PACKET;
        }
        write_command_row(out, &command);
        ++*record_count;
    }
}

static const char *log_kind_name(LogKind kind)
{
    if (kind == LOG_KIND_COMMAND) {
        return "command";
    }
    if (kind == LOG_KIND_FC_INTERNAL) {
        return "fc-internal";
    }
    return "sensor";
}

static SimStatus write_passive_manifest(
    const ConvertOptions *options,
    uint64_t record_count,
    SimStatus conversion_status)
{
    uint32_t input_crc = 0u;
    uint32_t output_crc = 0u;
    uint64_t input_size = 0u;
    uint64_t output_size = 0u;
    char wall_clock[32];
    FILE *file;
    SimStatus status;
    int close_status;

    if (options == 0) {
        return SIM_ERR_INVALID_ARG;
    }
    status = provenance_file_crc32(options->input_path, &input_crc, &input_size);
    if (status == SIM_OK && conversion_status == SIM_OK && options->output_path != 0) {
        status = provenance_file_crc32(options->output_path, &output_crc, &output_size);
    }
    if (status != SIM_OK) {
        return status;
    }
    provenance_format_wall_clock_utc(wall_clock, sizeof(wall_clock));
    file = fopen(options->manifest_path, "wb");
    if (file == 0) {
        return SIM_ERR_IO;
    }
    (void)fprintf(file, "{\n");
    (void)fprintf(file, "  \"schema_version\": 1,\n");
    (void)fprintf(file, "  \"run_mode\": \"REPLAY_PASSIVE\",\n");
    (void)fprintf(file, "  \"status\": ");
    (void)provenance_write_json_string(file, sim_status_to_string(conversion_status));
    (void)fprintf(file, ",\n  \"instance_id\": %u,\n", options->instance_id);
    (void)fprintf(file, "  \"log_type\": ");
    (void)provenance_write_json_string(file, log_kind_name(options->kind));
    (void)fprintf(file, ",\n  \"configuration\": null,\n");
    (void)fprintf(file, "  \"random_seed\": null,\n");
    (void)fprintf(file, "  \"random_seed_provenance\": \"preserved_in_historical_log_only\",\n");
    (void)fprintf(
        file,
        "  \"program_version\": \"log_convert %d.%d.%d\",\n",
        MISSILE_SIM_VERSION_MAJOR,
        MISSILE_SIM_VERSION_MINOR,
        MISSILE_SIM_VERSION_PATCH);
    (void)fprintf(file, "  \"git_commit\": ");
    (void)provenance_write_json_string(file, MISSILE_SIM_GIT_COMMIT);
    (void)fprintf(
        file,
        ",\n  \"git_worktree_dirty\": %s,\n",
        MISSILE_SIM_GIT_DIRTY != 0 ? "true" : "false");
    (void)fprintf(file, "  \"build_time\": ");
    (void)provenance_write_json_string(file, MISSILE_SIM_BUILD_TIME);
    (void)fprintf(file, ",\n  \"compiler\": ");
    (void)provenance_write_json_string(file, MISSILE_SIM_COMPILER);
    (void)fprintf(file, ",\n  \"start_time_wall_clock\": ");
    (void)provenance_write_json_string(file, wall_clock);
    (void)fprintf(
        file,
        ",\n  \"protocol_version\": \"%d.%d\",\n",
        MISSILE_SIM_PROTOCOL_VERSION_MAJOR,
        MISSILE_SIM_PROTOCOL_VERSION_MINOR);
    (void)fprintf(file, "  \"log_files\": {\n    \"input\": { \"path\": ");
    (void)provenance_write_json_string(file, options->input_path);
    (void)fprintf(
        file,
        ", \"crc32\": \"0x%08x\", \"size_bytes\": %llu },\n",
        input_crc,
        (unsigned long long)input_size);
    (void)fprintf(file, "    \"output\": { \"path\": ");
    (void)provenance_write_json_string(
        file,
        options->output_path != 0 ? options->output_path : "stdout");
    if (conversion_status == SIM_OK && options->output_path != 0) {
        (void)fprintf(
            file,
            ", \"crc32\": \"0x%08x\", \"size_bytes\": %llu }\n",
            output_crc,
            (unsigned long long)output_size);
    } else {
        (void)fprintf(file, ", \"crc32\": null, \"size_bytes\": null }\n");
    }
    (void)fprintf(file, "  },\n");
    (void)fprintf(file, "  \"record_count\": %llu\n", (unsigned long long)record_count);
    (void)fprintf(file, "}\n");
    status = ferror(file) == 0 ? SIM_OK : SIM_ERR_IO;
    close_status = fclose(file);
    return status == SIM_OK && close_status == 0 ? SIM_OK : SIM_ERR_IO;
}

/** @brief 工具入口。 */
int main(int argc, char **argv)
{
    ConvertOptions options;
    FILE *input;
    FILE *output = stdout;
    SimStatus status;
    SimStatus manifest_status;
    uint64_t record_count = 0u;
    int close_status = 0;
    int output_error;

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
    if (options.kind == LOG_KIND_SENSOR) {
        status = convert_sensor_log(input, output, options.instance_id, &record_count);
    } else if (options.kind == LOG_KIND_COMMAND) {
        status = convert_command_log(input, output, options.instance_id, &record_count);
    } else {
        status = convert_fc_internal_log(input, output, &record_count);
    }
    if (fclose(input) != 0 && status == SIM_OK) {
        status = SIM_ERR_IO;
    }
    output_error = ferror(output);
    if (options.output_path != 0) {
        close_status = fclose(output);
    } else if (fflush(output) != 0) {
        close_status = EOF;
    }
    if ((output_error != 0 || close_status != 0) && status == SIM_OK) {
        status = SIM_ERR_IO;
    }
    manifest_status = write_passive_manifest(&options, record_count, status);
    if (manifest_status != SIM_OK) {
        (void)fprintf(
            stderr,
            "log_convert: cannot write manifest: %s\n",
            sim_status_to_string(manifest_status));
        return 1;
    }
    if (status != SIM_OK) {
        (void)fprintf(stderr, "log_convert: conversion failed: %s\n", sim_status_to_string(status));
        return 1;
    }
    return 0;
}
