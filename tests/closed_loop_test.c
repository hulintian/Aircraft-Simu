/** @file closed_loop_test.c
 *  @brief 环境与飞控双进程锁步闭环集成测试。
 */
#define _POSIX_C_SOURCE 200809L

#include "common/packet.h"
#include "common/protocol.h"

#include <arpa/inet.h>
#include <errno.h>
#include <netinet/in.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <time.h>
#include <unistd.h>

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
    const char *runtime_path)
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

/** @brief 解码前三帧传感器日志并验证延迟预热与各通道有效位。 */
static int sensor_log_reports_expected_pipeline(const char *path)
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
        } else if ((frame.sensor_valid_flags & SIM_SENSOR_VALID_SEEKER) == 0u) {
            (void)fclose(file);
            return 0;
        }
    }
    (void)fclose(file);
    return 1;
}

/** @brief 启动双进程闭环并校验全部 P3 运行产物。 */
int main(int argc, char **argv)
{
    char output_dir[256];
    char instance_dir[320];
    char runtime_path[320];
    char path[512];
    unsigned int environment_port;
    unsigned int flight_control_port;
    pid_t fc_pid;
    pid_t env_pid;
    int fc_status = 0;
    int env_status = 0;
    struct timespec startup_delay = { 0, 100000000L };

    if (argc != 5) {
        (void)fprintf(stderr, "closed_loop_test: invalid arguments\n");
        return 2;
    }
    if (allocate_udp_port(&environment_port) != 0 ||
        allocate_udp_port(&flight_control_port) != 0 ||
        environment_port == flight_control_port) {
        (void)fprintf(stderr, "closed_loop_test: cannot allocate UDP ports\n");
        return 1;
    }

    (void)snprintf(output_dir, sizeof(output_dir), "/tmp/missile_closed_loop_%ld", (long)getpid());
    (void)snprintf(instance_dir, sizeof(instance_dir), "%s/instance_0000", output_dir);
    (void)snprintf(runtime_path, sizeof(runtime_path), "%s_runtime.json", output_dir);
    if (write_runtime(runtime_path, output_dir, environment_port, flight_control_port) != 0) {
        (void)fprintf(stderr, "closed_loop_test: cannot write runtime config\n");
        return 1;
    }

    fc_pid = launch_flight_control(argv[1], argv[3], runtime_path);
    if (fc_pid <= 0) {
        (void)unlink(runtime_path);
        return 1;
    }
    (void)nanosleep(&startup_delay, 0);
    env_pid = launch_environment(argv[2], argv[4], runtime_path);
    if (env_pid <= 0) {
        (void)kill(fc_pid, SIGTERM);
        (void)waitpid(fc_pid, &fc_status, 0);
        (void)unlink(runtime_path);
        return 1;
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
        return 1;
    }

    (void)snprintf(path, sizeof(path), "%s/run_manifest.json", instance_dir);
    if (!file_exists_and_nonempty(path) ||
        !text_file_contains(path, "\"random_seed\": 424242") ||
        !text_file_contains(path, "\"gravity_enabled\": true") ||
        !text_file_contains(path, "\"aerodynamics_enabled\": true") ||
        !text_file_contains(path, "\"earth_rotation_enabled\": true")) {
        return 1;
    }
    (void)snprintf(path, sizeof(path), "%s/event_log.txt", instance_dir);
    if (!file_exists_and_nonempty(path)) {
        return 1;
    }
    (void)snprintf(path, sizeof(path), "%s/sensor_log.bin", instance_dir);
    if (!file_exists_and_nonempty(path) ||
        !sensor_log_reports_expected_pipeline(path)) {
        return 1;
    }
    (void)snprintf(path, sizeof(path), "%s/command_log.bin", instance_dir);
    if (!file_exists_and_nonempty(path)) {
        return 1;
    }
    (void)snprintf(path, sizeof(path), "%s/trajectory.csv", instance_dir);
    if (!file_exists_and_nonempty(path) ||
        !text_file_contains(path, "missile_mass_kg") ||
        !text_file_contains(path, "force_b_x_n")) {
        return 1;
    }
    (void)snprintf(path, sizeof(path), "%s/summary.json", instance_dir);
    if (!text_file_contains(path, "\"hit_flag\": true")) {
        return 1;
    }

    return 0;
}
