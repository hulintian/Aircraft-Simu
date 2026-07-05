/** @file closed_loop_test.c
 *  @brief 环境与飞控双进程锁步闭环集成测试。
 */
#define _POSIX_C_SOURCE 200809L

#include "common/packet.h"
#include "common/protocol.h"
#include "env/aero_database.h"
#include "env/map_tile.h"
#include "env/terrain_model.h"
#include "fc/fc_health.h"
#include "fc/fc_modes.h"

#include <arpa/inet.h>
#include <ctype.h>
#include <errno.h>
#include <netinet/in.h>
#include <signal.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <time.h>
#include <unistd.h>

/** @brief 一次闭环回归运行的隔离路径集合。 */
typedef struct ClosedLoopRun {
    char output_dir[256];
    char instance_dir[320];
    char runtime_path[320];
    char faults_path[320];
} ClosedLoopRun;

/** @brief 闭环产物校验时的场景特定期望。 */
typedef struct ClosedLoopExpectation {
    int expect_los_occlusion;
    int expect_hit;
} ClosedLoopExpectation;

/** @brief 让内核临时分配一个可用的本地 UDP 端口。 */
static int allocate_udp_port(unsigned int *out)
{
    int socket_fd;
    struct sockaddr_in address;
    socklen_t address_size = sizeof(address);

    socket_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (socket_fd < 0) {
        return -1;
    }
    (void)memset(&address, 0, sizeof(address));
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    address.sin_port = 0;
    if (bind(socket_fd, (struct sockaddr *)&address, sizeof(address)) != 0 ||
        getsockname(socket_fd, (struct sockaddr *)&address, &address_size) != 0) {
        (void)close(socket_fd);
        return -1;
    }
    *out = (unsigned int)ntohs(address.sin_port);
    (void)close(socket_fd);
    return 0;
}

/** @brief 为测试实例生成隔离端口和输出目录的最小运行配置。 */
static int write_runtime(
    const char *path,
    const char *output_dir,
    unsigned int environment_port,
    unsigned int flight_control_port)
{
    FILE *file = fopen(path, "wb");

    if (file == 0) {
        return -1;
    }
    (void)fprintf(file, "{\n");
    (void)fprintf(file, "  \"schema_version\": 1,\n");
    (void)fprintf(
        file,
        "  \"campaign\": {"
        "\"instance_count\": 1, "
        "\"max_parallel_instances\": 1, "
        "\"base_random_seed\": 424242},\n");
    (void)fprintf(file, "  \"network\": {\n");
    (void)fprintf(file, "    \"environment_base_port\": %u,\n", environment_port);
    (void)fprintf(file, "    \"flight_control_base_port\": %u,\n", flight_control_port);
    (void)fprintf(file, "    \"host\": \"127.0.0.1\"\n");
    (void)fprintf(file, "  },\n");
    (void)fprintf(file, "  \"logging\": {\n");
    (void)fprintf(file, "    \"output_dir\": \"%s\",\n", output_dir);
    (void)fprintf(file, "    \"instance_dir_template\": \"instance_${instance_id}\",\n");
    (void)fprintf(file, "    \"binary_logs\": true,\n");
    (void)fprintf(file, "    \"event_log\": true,\n");
    (void)fprintf(file, "    \"flush_every_steps\": 100\n");
    (void)fprintf(file, "  }\n");
    (void)fprintf(file, "}\n");
    return fclose(file);
}

/** @brief 为闭环测试生成一个不影响前三帧传感器校验的故障脚本。 */
static int write_faults(const char *path)
{
    FILE *file = fopen(path, "wb");

    if (file == 0) {
        return -1;
    }
    (void)fprintf(file, "{\n");
    (void)fprintf(file, "  \"schema_version\": 1,\n");
    (void)fprintf(file, "  \"faults\": [\n");
    (void)fprintf(file, "    {\n");
    (void)fprintf(file, "      \"id\": \"closed_loop_speed_dropout\",\n");
    (void)fprintf(file, "      \"enabled\": true,\n");
    (void)fprintf(file, "      \"start_time_s\": 0.05,\n");
    (void)fprintf(file, "      \"duration_s\": 0.02,\n");
    (void)fprintf(file, "      \"target\": \"sensor.speedometer\",\n");
    (void)fprintf(file, "      \"type\": \"DROPOUT\"\n");
    (void)fprintf(file, "    },\n");
    (void)fprintf(file, "    {\n");
    (void)fprintf(file, "      \"id\": \"closed_loop_comm_delay\",\n");
    (void)fprintf(file, "      \"enabled\": true,\n");
    (void)fprintf(file, "      \"start_time_s\": 0.08,\n");
    (void)fprintf(file, "      \"duration_s\": 0.03,\n");
      (void)fprintf(file, "      \"target\": \"sensor.frame\",\n");
      (void)fprintf(file, "      \"type\": \"COMMUNICATION_DELAY\",\n");
      (void)fprintf(file, "      \"value\": 2,\n");
      (void)fprintf(file, "      \"recovery_hold_s\": 0.02\n");
      (void)fprintf(file, "    },\n");
    (void)fprintf(file, "    {\n");
    (void)fprintf(file, "      \"id\": \"closed_loop_comm_reorder\",\n");
    (void)fprintf(file, "      \"enabled\": true,\n");
    (void)fprintf(file, "      \"start_time_s\": 0.12,\n");
    (void)fprintf(file, "      \"duration_s\": 0.02,\n");
    (void)fprintf(file, "      \"target\": \"sensor.frame\",\n");
    (void)fprintf(file, "      \"type\": \"COMMUNICATION_REORDER\"\n");
    (void)fprintf(file, "    }\n");
    (void)fprintf(file, "  ]\n");
    (void)fprintf(file, "}\n");
    return fclose(file);
}

/** @brief 为闭环测试写一个覆盖 baseline 场景的常值地形瓦片。 */
static int write_test_tile(const char *path, int16_t height_m)
{
    TerrainTileHeader header = {
        TERRAIN_TILE_MAGIC,
        TERRAIN_TILE_VERSION,
        3u,
        3u,
        29.9 * 0.017453292519943295769236907684886,
        30.2 * 0.017453292519943295769236907684886,
        119.9 * 0.017453292519943295769236907684886,
        120.2 * 0.017453292519943295769236907684886,
        1.0,
        0.0,
        0u
    };
    int16_t samples[9];
    TerrainTile tile;
    size_t index;

    for (index = 0u; index < 9u; ++index) {
        samples[index] = height_m;
    }
    if (map_tile_bind(&tile, &header, samples, 9u) != SIM_OK) {
        return -1;
    }
    return map_tile_write_file(path, &tile) == SIM_OK ? 0 : -1;
}

/** @brief 为闭环测试写一个端点低、中间视线高的遮挡山脊瓦片。 */
static int write_test_occluding_tile(const char *path)
{
    TerrainTileHeader header = {
        TERRAIN_TILE_MAGIC,
        TERRAIN_TILE_VERSION,
        9u,
        3u,
        29.9 * 0.017453292519943295769236907684886,
        30.2 * 0.017453292519943295769236907684886,
        119.9 * 0.017453292519943295769236907684886,
        120.2 * 0.017453292519943295769236907684886,
        1.0,
        0.0,
        0u
    };
    int16_t samples[27];
    TerrainTile tile;
    size_t row;
    size_t column;

    for (row = 0u; row < 3u; ++row) {
        for (column = 0u; column < 9u; ++column) {
            samples[(row * 9u) + column] = column == 4u ? 30000 : 0;
        }
    }
    if (map_tile_bind(&tile, &header, samples, 27u) != SIM_OK) {
        return -1;
    }
    return map_tile_write_file(path, &tile) == SIM_OK ? 0 : -1;
}

/** @brief 按小端字节序写入 16 位整数。 */
static int write_u16_le(FILE *file, uint16_t value)
{
    unsigned char bytes[2];

    bytes[0] = (unsigned char)(value & UINT16_C(0xff));
    bytes[1] = (unsigned char)((value >> 8u) & UINT16_C(0xff));
    return fwrite(bytes, 1u, sizeof(bytes), file) == sizeof(bytes) ? 0 : -1;
}

/** @brief 按小端字节序写入 32 位整数。 */
static int write_u32_le(FILE *file, uint32_t value)
{
    unsigned char bytes[4];
    size_t index;

    for (index = 0u; index < sizeof(bytes); ++index) {
        bytes[index] = (unsigned char)((value >> (8u * index)) & UINT32_C(0xff));
    }
    return fwrite(bytes, 1u, sizeof(bytes), file) == sizeof(bytes) ? 0 : -1;
}

/** @brief 按小端字节序写入 IEEE-754 双精度值。 */
static int write_double_le(FILE *file, double value)
{
    uint64_t bits = 0u;
    unsigned char bytes[8];
    size_t index;

    (void)memcpy(&bits, &value, sizeof(bits));
    for (index = 0u; index < sizeof(bytes); ++index) {
        bytes[index] = (unsigned char)((bits >> (8u * index)) & UINT64_C(0xff));
    }
    return fwrite(bytes, 1u, sizeof(bytes), file) == sizeof(bytes) ? 0 : -1;
}

/** @brief 为闭环测试写一个单瓦片二进制空间索引。 */
static int write_test_tile_index(const char *path, const char *tile_path)
{
    FILE *file = fopen(path, "wb");
    const uint16_t path_size = tile_path == 0 ? 0u : (uint16_t)strlen(tile_path);

    if (file == 0 || tile_path == 0 ||
        path_size == 0u ||
        (size_t)path_size != strlen(tile_path)) {
        if (file != 0) {
            (void)fclose(file);
        }
        return -1;
    }
    if (write_u32_le(file, TERRAIN_TILE_INDEX_MAGIC) != 0 ||
        write_u16_le(file, (uint16_t)TERRAIN_TILE_INDEX_VERSION) != 0 ||
        write_u16_le(file, 1u) != 0 ||
        write_double_le(file, 29.9 * 0.017453292519943295769236907684886) != 0 ||
        write_double_le(file, 30.2 * 0.017453292519943295769236907684886) != 0 ||
        write_double_le(file, 119.9 * 0.017453292519943295769236907684886) != 0 ||
        write_double_le(file, 120.2 * 0.017453292519943295769236907684886) != 0 ||
        write_u16_le(file, path_size) != 0 ||
        fwrite(tile_path, 1u, path_size, file) != path_size) {
        (void)fclose(file);
        return -1;
    }
    return fclose(file) == 0 ? 0 : -1;
}

/** @brief 写一个闭环测试使用的零系数气动表。 */
static int write_test_aero_table(const char *path)
{
    const AeroTableSample samples[1] = {
        { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 }
    };

    return aero_database_write_file(
        path,
        samples,
        1u,
        AERO_DB_EXTRAPOLATION_CLAMP_AND_WARN) == SIM_OK ? 0 : -1;
}

/** @brief 写一个闭环测试使用的零系数线性代理模型。 */
static int write_test_aero_surrogate(const char *path)
{
    FILE *file = fopen(path, "wb");

    if (file == 0) {
        return -1;
    }
    if (fprintf(file, "MISSILE_AERO_SURROGATE_LINEAR_V1\n") < 0 ||
        fprintf(file, "cx 0.0 0.0 0.0 0.0\n") < 0 ||
        fprintf(file, "cy 0.0 0.0 0.0 0.0\n") < 0 ||
        fprintf(file, "cz 0.0 0.0 0.0 0.0\n") < 0 ||
        fprintf(file, "cl 0.0 0.0 0.0 0.0\n") < 0 ||
        fprintf(file, "cm 0.0 0.0 0.0 0.0\n") < 0 ||
        fprintf(file, "cn 0.0 0.0 0.0 0.0\n") < 0) {
        (void)fclose(file);
        return -1;
    }
    return fclose(file) == 0 ? 0 : -1;
}

/** @brief 从 baseline 场景派生一个配置了单瓦片和气动表路径的临时场景。 */
static int write_scenario_with_resource_paths(
    const char *source_path,
    const char *dest_path,
    const char *tile_index_path,
    const char *aero_table_path,
    double max_time_override_s)
{
    const char map_marker[] = "\"map\": {";
    const char aero_marker[] = "\"aerodynamics\": {";
    const char max_time_marker[] = "\"max_time\":";
    FILE *source = fopen(source_path, "rb");
    FILE *dest;
    char *buffer;
    char *map_position;
    char *aero_position;
    char *max_time_position;
    long size;
    size_t bytes_read;
    size_t map_prefix_size;
    size_t aero_prefix_size;
    const char *write_start;

    if (source == 0) {
        return -1;
    }
    if (fseek(source, 0, SEEK_END) != 0 ||
        (size = ftell(source)) < 0 ||
        fseek(source, 0, SEEK_SET) != 0) {
        (void)fclose(source);
        return -1;
    }
    buffer = (char *)malloc((size_t)size + 1u);
    if (buffer == 0) {
        (void)fclose(source);
        return -1;
    }
    bytes_read = fread(buffer, 1u, (size_t)size, source);
    (void)fclose(source);
    if (bytes_read != (size_t)size) {
        free(buffer);
        return -1;
    }
    buffer[(size_t)size] = '\0';
    map_position = strstr(buffer, map_marker);
    aero_position = strstr(buffer, aero_marker);
    max_time_position = strstr(buffer, max_time_marker);
    if (map_position == 0 || aero_position == 0 || aero_position <= map_position) {
        free(buffer);
        return -1;
    }
    dest = fopen(dest_path, "wb");
    if (dest == 0) {
        free(buffer);
        return -1;
    }
    map_prefix_size = (size_t)(map_position - buffer) + sizeof(map_marker) - 1u;
    aero_prefix_size = (size_t)(aero_position - buffer) + sizeof(aero_marker) - 1u;
    write_start = buffer;
    if (max_time_override_s > 0.0 && max_time_position != 0 && max_time_position < map_position) {
        char *max_time_value_end = max_time_position + sizeof(max_time_marker) - 1u;
        size_t prefix_size;

        while (*max_time_value_end != '\0' && isspace((unsigned char)*max_time_value_end) != 0) {
            ++max_time_value_end;
        }
        while (*max_time_value_end != '\0' &&
            *max_time_value_end != ',' &&
            *max_time_value_end != '\n' &&
            *max_time_value_end != '\r') {
            ++max_time_value_end;
        }
        prefix_size = (size_t)(max_time_position - buffer) + sizeof(max_time_marker) - 1u;
        if (fwrite(buffer, 1u, prefix_size, dest) != prefix_size ||
            fprintf(dest, " %.17g", max_time_override_s) < 0) {
            (void)fclose(dest);
            free(buffer);
            return -1;
        }
        write_start = max_time_value_end;
    }
    if (fwrite(write_start, 1u, (size_t)(map_position - write_start) + sizeof(map_marker) - 1u, dest) !=
            (size_t)(map_position - write_start) + sizeof(map_marker) - 1u ||
        fprintf(dest, "\n    \"tile_index_path\": \"%s\",", tile_index_path) < 0 ||
        fwrite(
            map_position + sizeof(map_marker) - 1u,
            1u,
            aero_prefix_size - map_prefix_size,
            dest) != aero_prefix_size - map_prefix_size ||
        fprintf(
            dest,
            "\n    \"table_path\": \"%s\",\n"
            "    \"table_extrapolation_policy\": \"CLAMP_AND_WARN\",\n"
            "    \"table_height_min_m\": 0.0,\n"
            "    \"table_height_max_m\": 5000.0,\n"
            "    \"table_actuator_min_rad\": -0.2,\n"
            "    \"table_actuator_max_rad\": 0.2,\n"
            "    \"surrogate_model_path\": \"%s.surrogate\",\n"
            "    \"surrogate_model_version\": \"surrogate-test-v1\",\n"
            "    \"surrogate_training_data_version\": \"training-data-test-v1\",",
            aero_table_path,
            aero_table_path) < 0 ||
        fwrite(
            aero_position + sizeof(aero_marker) - 1u,
            1u,
            (size_t)size - aero_prefix_size,
            dest) != (size_t)size - aero_prefix_size) {
        (void)fclose(dest);
        free(buffer);
        return -1;
    }
    free(buffer);
    return fclose(dest) == 0 ? 0 : -1;
}

/** @brief fork/exec 启动飞控测试进程。 */
static pid_t launch_flight_control(
    const char *program,
    const char *config_path,
    const char *runtime_path)
{
    pid_t pid = fork();

    if (pid == 0) {
        execl(
            program,
            program,
            "--instance-id",
            "0",
            "--config",
            config_path,
            "--runtime",
            runtime_path,
            (char *)0);
        _exit(127);
    }
    return pid;
}

/** @brief fork/exec 启动环境测试进程。 */
static pid_t launch_environment(
    const char *program,
    const char *scenario_path,
    const char *runtime_path,
    const char *faults_path)
{
    pid_t pid = fork();

    if (pid == 0) {
        execl(
            program,
            program,
            "--instance-id",
            "0",
            "--scenario",
            scenario_path,
            "--runtime",
            runtime_path,
            "--faults",
            faults_path,
            (char *)0);
        _exit(127);
    }
    return pid;
}

/** @brief 判断 waitpid 状态是否代表正常零退出。 */
static int process_succeeded(int status)
{
    return WIFEXITED(status) && WEXITSTATUS(status) == 0;
}

/** @brief 等待工具进程零退出。 */
static int wait_for_tool(pid_t pid)
{
    int status = 0;

    if (pid <= 0 || waitpid(pid, &status, 0) != pid) {
        return -1;
    }
    return process_succeeded(status) ? 0 : -1;
}

/** @brief 运行 replay 工具生成重放控制日志。 */
static int run_replay_tool(
    const char *program,
    const char *config_path,
    const char *sensor_log_path,
    const char *output_path)
{
    pid_t pid = fork();

    if (pid == 0) {
        execl(
            program,
            program,
            "--instance-id",
            "0",
            "--config",
            config_path,
            "--input",
            sensor_log_path,
            "--output",
            output_path,
            (char *)0);
        _exit(127);
    }
    return wait_for_tool(pid);
}

/** @brief 运行 compare_logs 工具对比控制日志。 */
static int run_compare_tool(
    const char *program,
    const char *left_path,
    const char *right_path,
    const char *output_path)
{
    pid_t pid = fork();

    if (pid == 0) {
        execl(
            program,
            program,
            "--type",
            "command",
            "--instance-id",
            "0",
            "--left",
            left_path,
            "--right",
            right_path,
            "--output",
            output_path,
            (char *)0);
        _exit(127);
    }
    return wait_for_tool(pid);
}

/** @brief 在有限时间内轮询两个子进程并在超时后终止。 */
static int wait_for_children(pid_t fc_pid, pid_t env_pid, int *fc_status, int *env_status)
{
    struct timespec delay = { 0, 10000000L };
    unsigned int attempts;
    int fc_done = 0;
    int env_done = 0;

    for (attempts = 0u; attempts < 1000u && (fc_done == 0 || env_done == 0); ++attempts) {
        if (fc_done == 0) {
            pid_t result = waitpid(fc_pid, fc_status, WNOHANG);
            if (result == fc_pid) {
                fc_done = 1;
            }
        }
        if (env_done == 0) {
            pid_t result = waitpid(env_pid, env_status, WNOHANG);
            if (result == env_pid) {
                env_done = 1;
            }
        }
        if (fc_done == 0 || env_done == 0) {
            (void)nanosleep(&delay, 0);
        }
    }

    if (fc_done == 0) {
        (void)kill(fc_pid, SIGTERM);
        (void)waitpid(fc_pid, fc_status, 0);
    }
    if (env_done == 0) {
        (void)kill(env_pid, SIGTERM);
        (void)waitpid(env_pid, env_status, 0);
    }
    return fc_done != 0 && env_done != 0 ? 0 : -1;
}

/** @brief 判断交付文件是否存在且包含数据。 */
static int file_exists_and_nonempty(const char *path)
{
    struct stat info;
    return stat(path, &info) == 0 && info.st_size > 0;
}

/** @brief 检查小型文本文件是否包含指定稳定字段。 */
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

/** @brief 解码前三帧传感器日志并验证延迟预热、LOS 遮挡与各通道有效位。 */
static int sensor_log_reports_expected_pipeline(const char *path, int expect_los_occlusion)
{
    unsigned char packet[SIM_SENSOR_PACKET_WIRE_SIZE];
    SensorFrame frame;
    FILE *file = fopen(path, "rb");
    unsigned int index;

    if (file == 0) {
        return 0;
    }
    for (index = 0u; index < 3u; ++index) {
        if (fread(packet, 1u, sizeof(packet), file) != sizeof(packet) ||
            packet_decode_sensor_frame(packet, sizeof(packet), 0u, &frame) != SIM_OK) {
            (void)fclose(file);
            return 0;
        }
        if ((frame.sensor_valid_flags & SIM_SENSOR_VALID_IMU_GYRO) == 0u ||
            (frame.sensor_valid_flags & SIM_SENSOR_VALID_ACCEL) == 0u ||
            (frame.sensor_valid_flags & SIM_SENSOR_VALID_SPEED) == 0u ||
            (frame.sensor_valid_flags & SIM_SENSOR_VALID_GEODETIC) == 0u) {
            (void)fclose(file);
            return 0;
        }
        if (index < 2u) {
            if ((frame.sensor_valid_flags & SIM_SENSOR_VALID_SEEKER) != 0u ||
                (frame.sensor_fault_flags & SIM_SENSOR_FAULT_DELAY_WARMUP) == 0u) {
                (void)fclose(file);
                return 0;
            }
        } else if (expect_los_occlusion == 0) {
            if ((frame.sensor_valid_flags & SIM_SENSOR_VALID_SEEKER) == 0u) {
                (void)fclose(file);
                return 0;
            }
        } else {
            if ((frame.sensor_valid_flags & SIM_SENSOR_VALID_SEEKER) != 0u ||
                (frame.sensor_fault_flags & SIM_SENSOR_FAULT_LOS_OCCLUDED) == 0u) {
                (void)fclose(file);
                return 0;
            }
        }
    }
    (void)fclose(file);
    return 1;
}

/** @brief 解码前三条控制命令并验证 P6 飞控模式和保护位。 */
static int command_log_reports_expected_protection(const char *path, int expect_los_occlusion)
{
    unsigned char packet[SIM_CONTROL_PACKET_WIRE_SIZE];
    ControlCommand command;
    FILE *file = fopen(path, "rb");
    unsigned int index;

    if (file == 0) {
        return 0;
    }
    for (index = 0u; index < 3u; ++index) {
        if (fread(packet, 1u, sizeof(packet), file) != sizeof(packet) ||
            packet_decode_control_command(packet, sizeof(packet), 0u, &command) != SIM_OK) {
            (void)fclose(file);
            return 0;
        }
        if (index < 2u) {
            if (command.command_mode != (uint32_t)FC_COMMAND_HOLD ||
                (command.command_status & FC_HEALTH_WARNING_MEASUREMENT_INVALID) == 0u ||
                (command.command_status & FC_HEALTH_WARNING_COMMAND_HELD) == 0u) {
                (void)fclose(file);
                return 0;
            }
        } else if (expect_los_occlusion == 0) {
            if (command.command_mode != (uint32_t)FC_GUIDANCE_ACTIVE ||
                (command.command_status & FC_HEALTH_WARNING_RATE_LIMITED) == 0u) {
                (void)fclose(file);
                return 0;
            }
        } else {
            if (command.command_mode != (uint32_t)FC_COMMAND_HOLD ||
                (command.command_status & FC_HEALTH_WARNING_MEASUREMENT_INVALID) == 0u ||
                (command.command_status & FC_HEALTH_WARNING_COMMAND_HELD) == 0u) {
                (void)fclose(file);
                return 0;
            }
        }
    }
    (void)fclose(file);
    return 1;
}

/** @brief 逐字节比较两个文件是否完全一致。 */
static int files_equal(const char *left_path, const char *right_path)
{
    unsigned char left_buffer[4096];
    unsigned char right_buffer[4096];
    FILE *left = fopen(left_path, "rb");
    FILE *right = fopen(right_path, "rb");

    if (left == 0 || right == 0) {
        if (left != 0) {
            (void)fclose(left);
        }
        if (right != 0) {
            (void)fclose(right);
        }
        return 0;
    }
    for (;;) {
        size_t left_size = fread(left_buffer, 1u, sizeof(left_buffer), left);
        size_t right_size = fread(right_buffer, 1u, sizeof(right_buffer), right);

        if (left_size != right_size ||
            (left_size > 0u && memcmp(left_buffer, right_buffer, left_size) != 0)) {
            (void)fclose(left);
            (void)fclose(right);
            return 0;
        }
        if (left_size < sizeof(left_buffer)) {
            const int done = feof(left) != 0 && feof(right) != 0;

            (void)fclose(left);
            (void)fclose(right);
            return done;
        }
    }
}

/** @brief 校验闭环运行产物和故障统计字段。 */
static int validate_closed_loop_outputs(
    const ClosedLoopRun *run,
    const ClosedLoopExpectation *expectation)
{
    char path[512];

    if (run == 0 || expectation == 0) {
        return 0;
    }
    (void)snprintf(path, sizeof(path), "%s/run_manifest.json", run->instance_dir);
    if (!file_exists_and_nonempty(path) ||
        !text_file_contains(path, "\"faults_path\":") ||
        !text_file_contains(path, "\"random_seed\": 424242") ||
        !text_file_contains(path, "\"gravity_enabled\": true") ||
        !text_file_contains(path, "\"aerodynamics_enabled\": true") ||
        !text_file_contains(path, "\"aero_table_enabled\": true") ||
        !text_file_contains(path, "\"aero_table_file_version\": 1") ||
        !text_file_contains(path, "\"aero_table_extrapolation_policy\": \"CLAMP_AND_WARN\"") ||
        !text_file_contains(path, "\"aero_table_height_envelope_enabled\": true") ||
        !text_file_contains(path, "\"aero_table_height_max_m\": 5000") ||
        !text_file_contains(path, "\"aero_table_actuator_envelope_enabled\": true") ||
        !text_file_contains(path, "\"aero_table_actuator_max_rad\": 0.20000000000000001") ||
        !text_file_contains(path, "\"aero_surrogate_enabled\": true") ||
        !text_file_contains(path, "\"aero_surrogate_model_version\": \"surrogate-test-v1\"") ||
        !text_file_contains(path, "\"aero_surrogate_training_data_version\": \"training-data-test-v1\"") ||
        !text_file_contains(path, "\"earth_rotation_enabled\": true")) {
        return 0;
    }
    if (!text_file_contains(path, "\"terrain_enabled\": true") ||
        !text_file_contains(path, "\"terrain_los_occlusion_enabled\": true") ||
        !text_file_contains(path, "\"terrain_missing_policy\": \"FLAT_FILL\"") ||
        !text_file_contains(path, "\"terrain_cache_tile_count\": 16") ||
        !text_file_contains(path, "\"terrain_tile_index_path_enabled\": true") ||
        !text_file_contains(path, "\"terrain_tile_index_path\":") ||
        !text_file_contains(path, "\"terrain_tile_path_count\": 1") ||
        !text_file_contains(path, "\"terrain_tile_paths\": [")) {
        return 0;
    }
    (void)snprintf(path, sizeof(path), "%s/event_log.txt", run->instance_dir);
    if (!file_exists_and_nonempty(path) ||
        !text_file_contains(path, "FAULT_START") ||
        !text_file_contains(path, "FAULT_END")) {
        return 0;
    }
    (void)snprintf(path, sizeof(path), "%s/sensor_log.bin", run->instance_dir);
    if (!file_exists_and_nonempty(path) ||
        !sensor_log_reports_expected_pipeline(path, expectation->expect_los_occlusion)) {
        return 0;
    }
    (void)snprintf(path, sizeof(path), "%s/command_log.bin", run->instance_dir);
    if (!file_exists_and_nonempty(path) ||
        !command_log_reports_expected_protection(path, expectation->expect_los_occlusion)) {
        return 0;
    }
    (void)snprintf(path, sizeof(path), "%s/fc_internal_log.bin", run->instance_dir);
    if (!file_exists_and_nonempty(path)) {
        return 0;
    }
    (void)snprintf(path, sizeof(path), "%s/trajectory.csv", run->instance_dir);
    if (!file_exists_and_nonempty(path) ||
        !text_file_contains(path, "missile_mass_kg") ||
        !text_file_contains(path, "force_b_x_n")) {
        return 0;
    }
    (void)snprintf(path, sizeof(path), "%s/trajectory_diagnostics.csv", run->instance_dir);
    if (!file_exists_and_nonempty(path) ||
        !text_file_contains(path, "quat_norm_error") ||
        !text_file_contains(path, "dcm_orthogonality_error") ||
        !text_file_contains(path, "model_degradation_flags")) {
        return 0;
    }
    (void)snprintf(path, sizeof(path), "%s/summary.json", run->instance_dir);
    if (!text_file_contains(
            path,
            expectation->expect_hit != 0 ? "\"hit_flag\": true" : "\"hit_flag\": false") ||
        !text_file_contains(path, "\"fault_configured_count\": 3") ||
        !text_file_contains(path, "\"fault_start_count\": 3") ||
        !text_file_contains(path, "\"fault_end_count\": 3") ||
        !text_file_contains(path, "\"fault_sensor_affected_step_count\":") ||
        !text_file_contains(path, "\"terrain_cache_path_count\": 1") ||
        !text_file_contains(path, "\"terrain_cache_capacity\":") ||
        !text_file_contains(path, "\"terrain_cache_load_count\":")) {
        return 0;
    }
    if (!text_file_contains(path, "\"diagnostic_sample_count\":") ||
        !text_file_contains(path, "\"max_quat_norm_error\":") ||
        !text_file_contains(path, "\"max_dcm_orthogonality_error\":") ||
        !text_file_contains(path, "\"min_mass_kg\":") ||
        !text_file_contains(path, "\"min_inertia_diag_kgm2\":") ||
        !text_file_contains(path, "\"aero_model_flags_or\":") ||
        !text_file_contains(path, "\"model_degradation_flags_or\":") ||
        !text_file_contains(path, "\"aero_extrapolated_sample_count\":")) {
        return 0;
    }
    return 1;
}

/** @brief 使用真实闭环 sensor_log 回放飞控并比较控制日志。 */
static int validate_replay_compare(
    const ClosedLoopRun *run,
    const char *flight_control_config,
    const char *replay_program,
    const char *compare_program)
{
    char sensor_log_path[512];
    char command_log_path[512];
    char replayed_command_path[512];
    char compare_output_path[512];

    if (run == 0 || flight_control_config == 0 || replay_program == 0 || compare_program == 0) {
        return 0;
    }
    (void)snprintf(sensor_log_path, sizeof(sensor_log_path), "%s/sensor_log.bin", run->instance_dir);
    (void)snprintf(command_log_path, sizeof(command_log_path), "%s/command_log.bin", run->instance_dir);
    (void)snprintf(
        replayed_command_path,
        sizeof(replayed_command_path),
        "%s/replayed_command_log.bin",
        run->instance_dir);
    (void)snprintf(
        compare_output_path,
        sizeof(compare_output_path),
        "%s/command_compare.json",
        run->instance_dir);
    if (run_replay_tool(
            replay_program,
            flight_control_config,
            sensor_log_path,
            replayed_command_path) != 0) {
        (void)fprintf(stderr, "closed_loop_test: replay failed\n");
        return 0;
    }
    if (!file_exists_and_nonempty(replayed_command_path)) {
        return 0;
    }
    if (run_compare_tool(
            compare_program,
            command_log_path,
            replayed_command_path,
            compare_output_path) != 0) {
        (void)fprintf(stderr, "closed_loop_test: compare_logs failed\n");
        return 0;
    }
    return file_exists_and_nonempty(compare_output_path) &&
        text_file_contains(compare_output_path, "\"verdict\": \"PASS\"");
}

/** @brief 启动一次闭环回归运行并等待两个进程正常退出。 */
static int run_closed_loop_once(
    const char *flight_control_program,
    const char *environment_program,
    const char *flight_control_config,
    const char *scenario_config,
    const char *label,
    ClosedLoopRun *run)
{
    unsigned int environment_port;
    unsigned int flight_control_port;
    pid_t fc_pid;
    pid_t env_pid;
    int fc_status = 0;
    int env_status = 0;
    struct timespec startup_delay = { 0, 100000000L };

    if (run == 0 || label == 0) {
        return -1;
    }
    (void)memset(run, 0, sizeof(*run));
    if (allocate_udp_port(&environment_port) != 0 ||
        allocate_udp_port(&flight_control_port) != 0 ||
        environment_port == flight_control_port) {
        (void)fprintf(stderr, "closed_loop_test: cannot allocate UDP ports\n");
        return -1;
    }

    (void)snprintf(
        run->output_dir,
        sizeof(run->output_dir),
        "/tmp/missile_closed_loop_%ld_%s",
        (long)getpid(),
        label);
    (void)snprintf(
        run->instance_dir,
        sizeof(run->instance_dir),
        "%s/instance_0000",
        run->output_dir);
    (void)snprintf(
        run->runtime_path,
        sizeof(run->runtime_path),
        "%s_runtime.json",
        run->output_dir);
    (void)snprintf(
        run->faults_path,
        sizeof(run->faults_path),
        "%s_faults.json",
        run->output_dir);
    if (write_runtime(run->runtime_path, run->output_dir, environment_port, flight_control_port) != 0) {
        (void)fprintf(stderr, "closed_loop_test: cannot write runtime config\n");
        return -1;
    }
    if (write_faults(run->faults_path) != 0) {
        (void)fprintf(stderr, "closed_loop_test: cannot write faults config\n");
        (void)unlink(run->runtime_path);
        return -1;
    }

    fc_pid = launch_flight_control(
        flight_control_program,
        flight_control_config,
        run->runtime_path);
    if (fc_pid <= 0) {
        (void)unlink(run->runtime_path);
        (void)unlink(run->faults_path);
        return -1;
    }
    (void)nanosleep(&startup_delay, 0);
    env_pid = launch_environment(
        environment_program,
        scenario_config,
        run->runtime_path,
        run->faults_path);
    if (env_pid <= 0) {
        (void)kill(fc_pid, SIGTERM);
        (void)waitpid(fc_pid, &fc_status, 0);
        (void)unlink(run->runtime_path);
        (void)unlink(run->faults_path);
        return -1;
    }

    if (wait_for_children(fc_pid, env_pid, &fc_status, &env_status) != 0 ||
        !process_succeeded(fc_status) ||
        !process_succeeded(env_status)) {
        (void)fprintf(
            stderr,
            "closed_loop_test: child failure fc=%d env=%d errno=%d\n",
            fc_status,
            env_status,
            errno);
        return -1;
    }
    return 0;
}

/** @brief 启动双进程闭环并校验全部 P3 运行产物。 */
int main(int argc, char **argv)
{
    ClosedLoopRun first;
    ClosedLoopRun second;
    ClosedLoopRun occluded;
    const ClosedLoopExpectation normal_expectation = { 0, 1 };
    const ClosedLoopExpectation occluded_expectation = { 1, 0 };
    char first_path[512];
    char second_path[512];
    char tile_path[256];
    char tile_relative_path[128];
    char tile_index_path[256];
    char occluded_tile_path[256];
    char occluded_tile_relative_path[128];
    char occluded_tile_index_path[256];
    char aero_table_path[256];
    char aero_surrogate_path[280];
    char scenario_with_tile_path[256];
    char occluded_scenario_path[256];

    if (argc != 7) {
        (void)fprintf(stderr, "closed_loop_test: invalid arguments\n");
        return 2;
    }
    (void)snprintf(
        tile_relative_path,
        sizeof(tile_relative_path),
        "missile_closed_loop_tile_%ld.bin",
        (long)getpid());
    (void)snprintf(tile_path, sizeof(tile_path), "/tmp/%s", tile_relative_path);
    (void)snprintf(
        tile_index_path,
        sizeof(tile_index_path),
        "/tmp/missile_closed_loop_tile_index_%ld.bin",
        (long)getpid());
    (void)snprintf(
        occluded_tile_relative_path,
        sizeof(occluded_tile_relative_path),
        "missile_closed_loop_occluded_tile_%ld.bin",
        (long)getpid());
    (void)snprintf(occluded_tile_path, sizeof(occluded_tile_path), "/tmp/%s", occluded_tile_relative_path);
    (void)snprintf(
        occluded_tile_index_path,
        sizeof(occluded_tile_index_path),
        "/tmp/missile_closed_loop_occluded_tile_index_%ld.bin",
        (long)getpid());
    (void)snprintf(
        scenario_with_tile_path,
        sizeof(scenario_with_tile_path),
        "/tmp/missile_closed_loop_scenario_%ld.json",
        (long)getpid());
    (void)snprintf(
        occluded_scenario_path,
        sizeof(occluded_scenario_path),
        "/tmp/missile_closed_loop_occluded_scenario_%ld.json",
        (long)getpid());
    (void)snprintf(
        aero_table_path,
        sizeof(aero_table_path),
        "/tmp/missile_closed_loop_aero_%ld.bin",
        (long)getpid());
    (void)snprintf(aero_surrogate_path, sizeof(aero_surrogate_path), "%s.surrogate", aero_table_path);
    if (write_test_tile(tile_path, 0) != 0 ||
        write_test_tile_index(tile_index_path, tile_relative_path) != 0 ||
        write_test_occluding_tile(occluded_tile_path) != 0 ||
        write_test_tile_index(occluded_tile_index_path, occluded_tile_relative_path) != 0 ||
        write_test_aero_table(aero_table_path) != 0 ||
        write_test_aero_surrogate(aero_surrogate_path) != 0) {
        return 1;
    }
    if (
        write_scenario_with_resource_paths(
            argv[4],
            scenario_with_tile_path,
            tile_index_path,
            aero_table_path,
            0.0) != 0 ||
        write_scenario_with_resource_paths(
            argv[4],
            occluded_scenario_path,
            occluded_tile_index_path,
            aero_table_path,
            0.25) != 0) {
        return 1;
    }
    if (run_closed_loop_once(argv[1], argv[2], argv[3], scenario_with_tile_path, "a", &first) != 0) {
        return 1;
    }
    if (!validate_closed_loop_outputs(&first, &normal_expectation)) {
        return 1;
    }
    if (!validate_replay_compare(&first, argv[3], argv[5], argv[6])) {
        return 1;
    }
    if (run_closed_loop_once(argv[1], argv[2], argv[3], scenario_with_tile_path, "b", &second) != 0) {
        return 1;
    }
    if (!validate_closed_loop_outputs(&second, &normal_expectation)) {
        return 1;
    }
    (void)snprintf(first_path, sizeof(first_path), "%s/summary.json", first.instance_dir);
    (void)snprintf(second_path, sizeof(second_path), "%s/summary.json", second.instance_dir);
    if (!files_equal(first_path, second_path)) {
        (void)fprintf(stderr, "closed_loop_test: summary repeatability mismatch\n");
        return 1;
    }
    (void)snprintf(first_path, sizeof(first_path), "%s/sensor_log.bin", first.instance_dir);
    (void)snprintf(second_path, sizeof(second_path), "%s/sensor_log.bin", second.instance_dir);
    if (!files_equal(first_path, second_path)) {
        (void)fprintf(stderr, "closed_loop_test: sensor log repeatability mismatch\n");
        return 1;
    }
    if (run_closed_loop_once(
            argv[1],
            argv[2],
            argv[3],
            occluded_scenario_path,
            "los",
            &occluded) != 0) {
        return 1;
    }
    if (!validate_closed_loop_outputs(&occluded, &occluded_expectation)) {
        return 1;
    }

    (void)unlink(first.runtime_path);
    (void)unlink(first.faults_path);
    (void)unlink(second.runtime_path);
    (void)unlink(second.faults_path);
    (void)unlink(occluded.runtime_path);
    (void)unlink(occluded.faults_path);
    (void)unlink(tile_path);
    (void)unlink(tile_index_path);
    (void)unlink(occluded_tile_path);
    (void)unlink(occluded_tile_index_path);
    (void)unlink(aero_table_path);
    (void)unlink(aero_surrogate_path);
    (void)unlink(scenario_with_tile_path);
    (void)unlink(occluded_scenario_path);
    return 0;
}
