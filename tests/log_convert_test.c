/** @file log_convert_test.c
 *  @brief P8 二进制日志转 CSV 工具集成测试。
 */
#define _POSIX_C_SOURCE 200809L

#include "common/packet.h"
#include "fc/fc_internal_log.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/wait.h>
#include <unistd.h>

/** @brief 判断小文本文件是否包含指定片段。 */
static int text_file_contains(const char *path, const char *expected)
{
    char buffer[4096];
    FILE *file = fopen(path, "rb");
    size_t size;

    if (file == 0 || expected == 0) {
        return 0;
    }
    size = fread(buffer, 1u, sizeof(buffer) - 1u, file);
    (void)fclose(file);
    buffer[size] = '\0';
    return strstr(buffer, expected) != 0;
}

/** @brief 写出一条固定控制命令二进制日志。 */
static int write_command_log(const char *path)
{
    ControlCommand command;
    unsigned char packet[SIM_CONTROL_PACKET_WIRE_SIZE];
    size_t packet_size = 0u;
    FILE *file = fopen(path, "wb");

    if (file == 0) {
        return -1;
    }
    (void)memset(&command, 0, sizeof(command));
    command.seq = 7u;
    command.sim_time = 0.25;
    command.accel_cmd_ecef = vec3_make(1.0, 2.0, 3.0);
    command.attitude_cmd = vec3_make(0.1, 0.2, 0.3);
    command.body_rate_cmd = vec3_make(0.4, 0.5, 0.6);
    command.actuator_cmd[0] = 0.12;
    command.actuator_cmd[1] = -0.34;
    command.command_mode = 4u;
    command.command_status = 8u;
    if (packet_encode_control_command(0u, &command, packet, sizeof(packet), &packet_size) != SIM_OK ||
        fwrite(packet, 1u, packet_size, file) != packet_size) {
        (void)fclose(file);
        return -1;
    }
    return fclose(file);
}

/** @brief 运行 log_convert 子进程。 */
static int run_log_convert(
    const char *program,
    const char *kind,
    const char *input_path,
    const char *output_path)
{
    pid_t pid = fork();
    int status = 0;

    if (pid == 0) {
        execl(
            program,
            program,
            "--type",
            kind,
            "--instance-id",
            "0",
            "--input",
            input_path,
            "--output",
            output_path,
            (char *)0);
        _exit(127);
    }
    if (pid <= 0 || waitpid(pid, &status, 0) != pid) {
        return -1;
    }
    return WIFEXITED(status) && WEXITSTATUS(status) == 0 ? 0 : -1;
}

/** @brief 写出一条飞控内部状态日志。 */
static int write_fc_internal_log(const char *path)
{
    ControlCommand command;
    unsigned char record[FC_INTERNAL_LOG_WIRE_SIZE];
    FILE *file = fopen(path, "wb");

    if (file == 0) {
        return -1;
    }
    (void)memset(&command, 0, sizeof(command));
    command.seq = 9u;
    command.sim_time = 0.5;
    command.command_mode = 6u;
    command.command_status = 16u;
    command.accel_cmd_ecef = vec3_make(4.0, 5.0, 6.0);
    command.attitude_cmd = vec3_make(0.11, 0.22, 0.33);
    command.body_rate_cmd = vec3_make(0.44, 0.55, 0.66);
    if (fc_internal_log_encode(&command, record, sizeof(record)) != SIM_OK ||
        fwrite(record, 1u, sizeof(record), file) != sizeof(record)) {
        (void)fclose(file);
        return -1;
    }
    return fclose(file);
}

int main(int argc, char **argv)
{
    char input_path[256];
    char output_path[256];
    char internal_input_path[256];
    char internal_output_path[256];
    char output_manifest_path[320];
    char internal_manifest_path[320];

    if (argc != 2) {
        return 2;
    }
    (void)snprintf(input_path, sizeof(input_path), "/tmp/missile_log_convert_%ld.bin", (long)getpid());
    (void)snprintf(output_path, sizeof(output_path), "/tmp/missile_log_convert_%ld.csv", (long)getpid());
    (void)snprintf(
        internal_input_path,
        sizeof(internal_input_path),
        "/tmp/missile_fc_internal_%ld.bin",
        (long)getpid());
    (void)snprintf(
        internal_output_path,
        sizeof(internal_output_path),
        "/tmp/missile_fc_internal_%ld.csv",
        (long)getpid());
    (void)snprintf(
        output_manifest_path,
        sizeof(output_manifest_path),
        "%s.run_manifest.json",
        output_path);
    (void)snprintf(
        internal_manifest_path,
        sizeof(internal_manifest_path),
        "%s.run_manifest.json",
        internal_output_path);
    if (write_command_log(input_path) != 0) {
        return 1;
    }
    if (run_log_convert(argv[1], "command", input_path, output_path) != 0) {
        return 1;
    }
    if (write_fc_internal_log(internal_input_path) != 0 ||
        run_log_convert(argv[1], "fc-internal", internal_input_path, internal_output_path) != 0 ||
        !text_file_contains(internal_output_path, "9,0.500000000,") ||
        !text_file_contains(internal_output_path, "4.000000000,5.000000000,6.000000000") ||
        !text_file_contains(internal_output_path, ",6,16") ||
        !text_file_contains(internal_manifest_path, "\"run_mode\": \"REPLAY_PASSIVE\"") ||
        !text_file_contains(internal_manifest_path, "\"log_type\": \"fc-internal\"") ||
        !text_file_contains(internal_manifest_path, "\"record_count\": 1")) {
        return 1;
    }
    if (!text_file_contains(output_path, "seq,sim_time,") ||
        !text_file_contains(output_path, "7,0.250000000,") ||
        !text_file_contains(output_path, "0.120000000,-0.340000000") ||
        !text_file_contains(output_path, ",4,8") ||
        !text_file_contains(output_manifest_path, "\"run_mode\": \"REPLAY_PASSIVE\"") ||
        !text_file_contains(output_manifest_path, "\"log_type\": \"command\"") ||
        !text_file_contains(output_manifest_path, "\"record_count\": 1") ||
        !text_file_contains(output_manifest_path, "\"crc32\": \"0x")) {
        return 1;
    }
    (void)unlink(input_path);
    (void)unlink(output_path);
    (void)unlink(internal_input_path);
    (void)unlink(internal_output_path);
    (void)unlink(output_manifest_path);
    (void)unlink(internal_manifest_path);
    return 0;
}
