/** @file long_campaign_test.c
 *  @brief 可选多实例长运行压力测试。
 */
#define _POSIX_C_SOURCE 200809L

#include <arpa/inet.h>
#include <errno.h>
#include <netinet/in.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <time.h>
#include <unistd.h>

enum {
    LONG_CAMPAIGN_INSTANCE_COUNT = 6,
    LONG_CAMPAIGN_MAX_PARALLEL = 3
};

/** @brief 尝试绑定一个 UDP 端口并持有套接字。 */
static int bind_udp_port(unsigned int port, int *socket_out)
{
    int socket_fd;
    struct sockaddr_in address;

    if (socket_out == 0 || port > 65535u) {
        return -1;
    }
    socket_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (socket_fd < 0) {
        return -1;
    }
    (void)memset(&address, 0, sizeof(address));
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = htonl(INADDR_ANY);
    address.sin_port = htons((uint16_t)port);
    if (bind(socket_fd, (struct sockaddr *)&address, sizeof(address)) != 0) {
        (void)close(socket_fd);
        return -1;
    }
    *socket_out = socket_fd;
    return 0;
}

/** @brief 为压力测试寻找一段连续可用端口。 */
static int allocate_port_block(unsigned int *environment_base, unsigned int *flight_control_base)
{
    const unsigned int required_ports = 2u * (unsigned int)LONG_CAMPAIGN_INSTANCE_COUNT;
    const unsigned int first = 28000u;
    const unsigned int last = 62000u;
    const unsigned int span = (last - first - required_ports) / required_ports;
    const unsigned int rotate = (unsigned int)((unsigned long)getpid() % (unsigned long)span);
    unsigned int attempt;

    for (attempt = 0u; attempt < span; ++attempt) {
        unsigned int base = first + (required_ports * ((attempt + rotate) % span));
        int sockets[2u * LONG_CAMPAIGN_INSTANCE_COUNT];
        size_t index;
        int ok = 1;

        for (index = 0u; index < (size_t)required_ports; ++index) {
            sockets[index] = -1;
        }
        for (index = 0u; index < (size_t)required_ports; ++index) {
            if (bind_udp_port(base + (unsigned int)index, &sockets[index]) != 0) {
                ok = 0;
                break;
            }
        }
        for (index = 0u; index < (size_t)required_ports; ++index) {
            if (sockets[index] >= 0) {
                (void)close(sockets[index]);
            }
        }
        if (ok != 0) {
            *environment_base = base;
            *flight_control_base = base + 1u;
            return 0;
        }
    }
    return -1;
}

/** @brief 写出多实例压力测试 runtime.json。 */
static int write_runtime(
    const char *path,
    const char *source_dir,
    const char *output_dir,
    const char *environment_program,
    const char *flight_control_program,
    unsigned int environment_base,
    unsigned int flight_control_base)
{
    FILE *file = fopen(path, "wb");
    unsigned int index;

    if (file == 0) {
        return -1;
    }
    (void)fprintf(file, "{\n");
    (void)fprintf(file, "  \"schema_version\": 1,\n");
    (void)fprintf(file, "  \"campaign\": {\n");
    (void)fprintf(file, "    \"campaign_id\": \"long_campaign_pressure_test\",\n");
    (void)fprintf(file, "    \"instance_count\": %u,\n", (unsigned int)LONG_CAMPAIGN_INSTANCE_COUNT);
    (void)fprintf(file, "    \"schedule\": \"PARALLEL\",\n");
    (void)fprintf(file, "    \"max_parallel_instances\": %u,\n", (unsigned int)LONG_CAMPAIGN_MAX_PARALLEL);
    (void)fprintf(file, "    \"base_random_seed\": 10000,\n");
    (void)fprintf(file, "    \"failure_strategy\": \"CONTINUE_ON_FAILURE\"\n");
    (void)fprintf(file, "  },\n");
    (void)fprintf(file, "  \"network\": {\n");
    (void)fprintf(file, "    \"environment_base_port\": %u,\n", environment_base);
    (void)fprintf(file, "    \"flight_control_base_port\": %u,\n", flight_control_base);
    (void)fprintf(file, "    \"host\": \"127.0.0.1\"\n");
    (void)fprintf(file, "  },\n");
    (void)fprintf(file, "  \"logging\": {\n");
    (void)fprintf(file, "    \"output_dir\": \"%s\",\n", output_dir);
    (void)fprintf(file, "    \"instance_dir_template\": \"instance_${instance_id}\",\n");
    (void)fprintf(file, "    \"binary_logs\": true,\n");
    (void)fprintf(file, "    \"event_log\": true,\n");
    (void)fprintf(file, "    \"flush_every_steps\": 200\n");
    (void)fprintf(file, "  },\n");
    (void)fprintf(file, "  \"tools\": {\n");
    (void)fprintf(file, "    \"environment_program\": \"%s\",\n", environment_program);
    (void)fprintf(file, "    \"flight_control_program\": \"%s\"\n", flight_control_program);
    (void)fprintf(file, "  },\n");
    (void)fprintf(file, "  \"instances\": [\n");
    for (index = 0u; index < (unsigned int)LONG_CAMPAIGN_INSTANCE_COUNT; ++index) {
        (void)fprintf(file, "    {\n");
        (void)fprintf(file, "      \"enabled\": true,\n");
        (void)fprintf(file, "      \"instance_id\": %u,\n", index);
        (void)fprintf(file, "      \"scenario\": \"%s/configs/baseline/scenario.json\",\n", source_dir);
        (void)fprintf(
            file,
            "      \"flight_control\": \"%s/configs/baseline/flight_control.json\",\n",
            source_dir);
        (void)fprintf(file, "      \"faults\": \"%s/configs/baseline/faults.json\",\n", source_dir);
        (void)fprintf(file, "      \"random_seed\": %u\n", 10001u + index);
        (void)fprintf(
            file,
            "    }%s\n",
            index + 1u == (unsigned int)LONG_CAMPAIGN_INSTANCE_COUNT ? "" : ",");
    }
    (void)fprintf(file, "  ]\n");
    (void)fprintf(file, "}\n");
    return fclose(file);
}

/** @brief 判断小文本文件是否包含指定片段。 */
static int text_file_contains(const char *path, const char *expected)
{
    char buffer[16384];
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

/** @brief 启动管理器并等待其在限定时间内退出。 */
static int run_manager(const char *manager_program, const char *runtime_path)
{
    pid_t pid = fork();
    int status = 0;
    unsigned int attempt;
    struct timespec delay = { 0, 10000000L };

    if (pid == 0) {
        (void)setpgid(0, 0);
        if (chdir("/tmp") != 0) {
            _exit(126);
        }
        execl(manager_program, manager_program, "--runtime", runtime_path, (char *)0);
        _exit(127);
    }
    if (pid <= 0) {
        return -1;
    }
    for (attempt = 0u; attempt < 9000u; ++attempt) {
        pid_t result = waitpid(pid, &status, WNOHANG);

        if (result == pid) {
            return WIFEXITED(status) && WEXITSTATUS(status) == 0 ? 0 : -1;
        }
        if (result < 0 && errno != EINTR) {
            return -1;
        }
        (void)nanosleep(&delay, 0);
    }
    (void)kill(-pid, SIGTERM);
    (void)waitpid(pid, &status, 0);
    return -1;
}

int main(int argc, char **argv)
{
    unsigned int environment_base;
    unsigned int flight_control_base;
    char output_dir[256];
    char runtime_path[320];
    char path[512];

    if (argc != 5) {
        (void)fprintf(stderr, "long_campaign_test: invalid arguments\n");
        return 2;
    }
    if (allocate_port_block(&environment_base, &flight_control_base) != 0) {
        (void)fprintf(stderr, "long_campaign_test: cannot allocate ports\n");
        return 1;
    }

    (void)snprintf(output_dir, sizeof(output_dir), "/tmp/missile_long_campaign_%ld", (long)getpid());
    (void)snprintf(runtime_path, sizeof(runtime_path), "%s_runtime.json", output_dir);
    if (write_runtime(
            runtime_path,
            argv[2],
            output_dir,
            argv[3],
            argv[4],
            environment_base,
            flight_control_base) != 0) {
        (void)fprintf(stderr, "long_campaign_test: cannot write runtime\n");
        return 1;
    }
    if (run_manager(argv[1], runtime_path) != 0) {
        return 1;
    }

    (void)snprintf(path, sizeof(path), "%s/campaign_summary.json", output_dir);
    if (!text_file_contains(path, "\"instance_count\": 6") ||
        !text_file_contains(path, "\"schedule\": \"PARALLEL\"") ||
        !text_file_contains(path, "\"campaign_wall_time_s\":") ||
        !text_file_contains(path, "\"total_instance_wall_time_s\":") ||
        !text_file_contains(path, "\"max_instance_wall_time_s\":") ||
        !text_file_contains(path, "\"completed_count\": 6") ||
        !text_file_contains(path, "\"failed_count\": 0") ||
        !text_file_contains(path, "\"summary_available_count\": 6") ||
        !text_file_contains(path, "\"total_diagnostic_sample_count\":") ||
        !text_file_contains(path, "\"random_seed\": 10001") ||
        !text_file_contains(path, "\"random_seed\": 10006")) {
        return 1;
    }

    (void)snprintf(path, sizeof(path), "%s/instance_0005/summary.json", output_dir);
    if (!text_file_contains(path, "\"exit_reason\":")) {
        return 1;
    }
    (void)unlink(runtime_path);
    return 0;
}
