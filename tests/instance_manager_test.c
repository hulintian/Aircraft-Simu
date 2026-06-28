/** @file instance_manager_test.c
 *  @brief 多实例管理器逐实例计划和输出隔离集成测试。
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
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <time.h>
#include <unistd.h>

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

/** @brief 为两个实例寻找四个连续可用端口。 */
static int allocate_port_block(unsigned int *environment_base, unsigned int *flight_control_base)
{
    const unsigned int first = 30000u;
    const unsigned int last = 62000u;
    const unsigned int span = (last - first) / 4u;
    const unsigned int rotate = (unsigned int)((unsigned long)getpid() % (unsigned long)span);
    unsigned int attempt;

    for (attempt = 0u; attempt < span; ++attempt) {
        unsigned int base = first + (4u * ((attempt + rotate) % span));
        int sockets[4] = { -1, -1, -1, -1 };
        size_t index;
        int ok = 1;

        for (index = 0u; index < 4u; ++index) {
            if (bind_udp_port(base + (unsigned int)index, &sockets[index]) != 0) {
                ok = 0;
                break;
            }
        }
        for (index = 0u; index < 4u; ++index) {
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

/** @brief 写出测试用 runtime.json。 */
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

    if (file == 0) {
        return -1;
    }
    (void)fprintf(file, "{\n");
    (void)fprintf(file, "  \"schema_version\": 1,\n");
    (void)fprintf(file, "  \"campaign\": {\n");
    (void)fprintf(file, "    \"campaign_id\": \"manager_test\",\n");
    (void)fprintf(file, "    \"instance_count\": 2,\n");
    (void)fprintf(file, "    \"schedule\": \"PARALLEL\",\n");
    (void)fprintf(file, "    \"max_parallel_instances\": 2,\n");
    (void)fprintf(file, "    \"base_random_seed\": 9000,\n");
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
    (void)fprintf(file, "    \"flush_every_steps\": 100\n");
    (void)fprintf(file, "  },\n");
    (void)fprintf(file, "  \"tools\": {\n");
    (void)fprintf(file, "    \"environment_program\": \"%s\",\n", environment_program);
    (void)fprintf(file, "    \"flight_control_program\": \"%s\"\n", flight_control_program);
    (void)fprintf(file, "  },\n");
    (void)fprintf(file, "  \"instances\": [\n");
    (void)fprintf(file, "    {\n");
    (void)fprintf(file, "      \"enabled\": true,\n");
    (void)fprintf(file, "      \"instance_id\": 0,\n");
    (void)fprintf(file, "      \"scenario\": \"%s/configs/baseline/scenario.json\",\n", source_dir);
    (void)fprintf(file, "      \"flight_control\": \"%s/configs/baseline/flight_control.json\",\n", source_dir);
    (void)fprintf(file, "      \"faults\": \"%s/configs/baseline/faults.json\",\n", source_dir);
    (void)fprintf(file, "      \"random_seed\": 9001\n");
    (void)fprintf(file, "    },\n");
    (void)fprintf(file, "    {\n");
    (void)fprintf(file, "      \"enabled\": true,\n");
    (void)fprintf(file, "      \"instance_id\": 1,\n");
    (void)fprintf(file, "      \"scenario\": \"%s/configs/baseline/scenario.json\",\n", source_dir);
    (void)fprintf(file, "      \"flight_control\": \"%s/configs/baseline/flight_control.json\",\n", source_dir);
    (void)fprintf(file, "      \"faults\": \"%s/configs/baseline/faults.json\",\n", source_dir);
    (void)fprintf(file, "      \"random_seed\": 9002\n");
    (void)fprintf(file, "    }\n");
    (void)fprintf(file, "  ]\n");
    (void)fprintf(file, "}\n");
    return fclose(file);
}

/** @brief 写出一个飞控配置路径无效的单实例运行配置。 */
static int write_bad_flight_control_runtime(
    const char *path,
    const char *source_dir,
    const char *output_dir,
    const char *environment_program,
    const char *flight_control_program,
    unsigned int environment_base,
    unsigned int flight_control_base)
{
    FILE *file = fopen(path, "wb");

    if (file == 0) {
        return -1;
    }
    (void)fprintf(file, "{\n");
    (void)fprintf(file, "  \"schema_version\": 1,\n");
    (void)fprintf(file, "  \"campaign\": {\n");
    (void)fprintf(file, "    \"campaign_id\": \"manager_bad_fc_test\",\n");
    (void)fprintf(file, "    \"instance_count\": 1,\n");
    (void)fprintf(file, "    \"schedule\": \"SEQUENTIAL\",\n");
    (void)fprintf(file, "    \"max_parallel_instances\": 1,\n");
    (void)fprintf(file, "    \"base_random_seed\": 9100,\n");
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
    (void)fprintf(file, "    \"flush_every_steps\": 100\n");
    (void)fprintf(file, "  },\n");
    (void)fprintf(file, "  \"tools\": {\n");
    (void)fprintf(file, "    \"environment_program\": \"%s\",\n", environment_program);
    (void)fprintf(file, "    \"flight_control_program\": \"%s\"\n", flight_control_program);
    (void)fprintf(file, "  },\n");
    (void)fprintf(file, "  \"instances\": [\n");
    (void)fprintf(file, "    {\n");
    (void)fprintf(file, "      \"enabled\": true,\n");
    (void)fprintf(file, "      \"instance_id\": 0,\n");
    (void)fprintf(file, "      \"scenario\": \"%s/configs/baseline/scenario.json\",\n", source_dir);
    (void)fprintf(file, "      \"flight_control\": \"%s/configs/baseline/missing_flight_control.json\",\n", source_dir);
    (void)fprintf(file, "      \"faults\": \"%s/configs/baseline/faults.json\",\n", source_dir);
    (void)fprintf(file, "      \"random_seed\": 9101\n");
    (void)fprintf(file, "    }\n");
    (void)fprintf(file, "  ]\n");
    (void)fprintf(file, "}\n");
    return fclose(file);
}

/** @brief 写出 STOP_ON_FAILURE 场景，首实例失败后第二实例应被跳过。 */
static int write_stop_on_failure_runtime(
    const char *path,
    const char *source_dir,
    const char *output_dir,
    const char *environment_program,
    const char *flight_control_program,
    unsigned int environment_base,
    unsigned int flight_control_base)
{
    FILE *file = fopen(path, "wb");

    if (file == 0) {
        return -1;
    }
    (void)fprintf(file, "{\n");
    (void)fprintf(file, "  \"schema_version\": 1,\n");
    (void)fprintf(file, "  \"campaign\": {\n");
    (void)fprintf(file, "    \"campaign_id\": \"manager_stop_test\",\n");
    (void)fprintf(file, "    \"instance_count\": 2,\n");
    (void)fprintf(file, "    \"schedule\": \"PARALLEL\",\n");
    (void)fprintf(file, "    \"max_parallel_instances\": 2,\n");
    (void)fprintf(file, "    \"base_random_seed\": 9200,\n");
    (void)fprintf(file, "    \"failure_strategy\": \"STOP_ON_FAILURE\"\n");
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
    (void)fprintf(file, "    \"flush_every_steps\": 100\n");
    (void)fprintf(file, "  },\n");
    (void)fprintf(file, "  \"tools\": {\n");
    (void)fprintf(file, "    \"environment_program\": \"%s\",\n", environment_program);
    (void)fprintf(file, "    \"flight_control_program\": \"%s\"\n", flight_control_program);
    (void)fprintf(file, "  },\n");
    (void)fprintf(file, "  \"instances\": [\n");
    (void)fprintf(file, "    {\n");
    (void)fprintf(file, "      \"enabled\": true,\n");
    (void)fprintf(file, "      \"instance_id\": 0,\n");
    (void)fprintf(file, "      \"scenario\": \"%s/configs/baseline/scenario.json\",\n", source_dir);
    (void)fprintf(file, "      \"flight_control\": \"%s/configs/baseline/missing_flight_control.json\",\n", source_dir);
    (void)fprintf(file, "      \"faults\": \"%s/configs/baseline/faults.json\",\n", source_dir);
    (void)fprintf(file, "      \"random_seed\": 9201\n");
    (void)fprintf(file, "    },\n");
    (void)fprintf(file, "    {\n");
    (void)fprintf(file, "      \"enabled\": true,\n");
    (void)fprintf(file, "      \"instance_id\": 1,\n");
    (void)fprintf(file, "      \"scenario\": \"%s/configs/baseline/scenario.json\",\n", source_dir);
    (void)fprintf(file, "      \"flight_control\": \"%s/configs/baseline/flight_control.json\",\n", source_dir);
    (void)fprintf(file, "      \"faults\": \"%s/configs/baseline/faults.json\",\n", source_dir);
    (void)fprintf(file, "      \"random_seed\": 9202\n");
    (void)fprintf(file, "    }\n");
    (void)fprintf(file, "  ]\n");
    (void)fprintf(file, "}\n");
    return fclose(file);
}

/** @brief 判断小文本文件是否包含指定片段。 */
static int text_file_contains(const char *path, const char *expected)
{
    char buffer[8192];
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
static int run_manager(const char *manager_program, const char *source_dir, const char *runtime_path)
{
    pid_t pid = fork();
    int status = 0;
    unsigned int attempt;
    struct timespec delay = { 0, 10000000L };

    (void)source_dir;
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
    for (attempt = 0u; attempt < 3000u; ++attempt) {
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

/** @brief 运行管理器并要求其失败，用于验证错误路径摘要。 */
static int run_manager_expect_failure(
    const char *manager_program,
    const char *source_dir,
    const char *runtime_path)
{
    pid_t pid = fork();
    int status = 0;
    unsigned int attempt;
    struct timespec delay = { 0, 10000000L };

    (void)source_dir;
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
    for (attempt = 0u; attempt < 3000u; ++attempt) {
        pid_t result = waitpid(pid, &status, WNOHANG);

        if (result == pid) {
            return WIFEXITED(status) && WEXITSTATUS(status) != 0 ? 0 : -1;
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
    char bad_output_dir[256];
    char bad_runtime_path[320];
    char port_output_dir[256];
    char port_runtime_path[320];
    char stop_output_dir[256];
    char stop_runtime_path[320];
    char path[512];
    int held_socket = -1;

    if (argc != 5) {
        (void)fprintf(stderr, "instance_manager_test: invalid arguments\n");
        return 2;
    }
    if (allocate_port_block(&environment_base, &flight_control_base) != 0) {
        (void)fprintf(stderr, "instance_manager_test: cannot allocate ports\n");
        return 1;
    }
    (void)snprintf(output_dir, sizeof(output_dir), "/tmp/missile_manager_%ld", (long)getpid());
    (void)snprintf(runtime_path, sizeof(runtime_path), "%s_runtime.json", output_dir);
    if (write_runtime(
            runtime_path,
            argv[2],
            output_dir,
            argv[3],
            argv[4],
            environment_base,
            flight_control_base) != 0) {
        (void)fprintf(stderr, "instance_manager_test: cannot write runtime\n");
        return 1;
    }
    if (run_manager(argv[1], argv[2], runtime_path) != 0) {
        return 1;
    }

    (void)snprintf(path, sizeof(path), "%s/campaign_summary.json", output_dir);
    if (!text_file_contains(path, "\"completed_count\": 2") ||
        !text_file_contains(path, "\"failed_count\": 0") ||
        !text_file_contains(path, "\"schedule\": \"PARALLEL\"") ||
        !text_file_contains(path, "\"random_seed\": 9001") ||
        !text_file_contains(path, "\"random_seed\": 9002")) {
        return 1;
    }
    (void)snprintf(path, sizeof(path), "%s/instance_0000/run_manifest.json", output_dir);
    if (!text_file_contains(path, "\"random_seed\": 9001") ||
        !text_file_contains(path, "\"scenario_path\": \"/")) {
        return 1;
    }
    (void)snprintf(path, sizeof(path), "%s/instance_0001/run_manifest.json", output_dir);
    if (!text_file_contains(path, "\"random_seed\": 9002") ||
        !text_file_contains(path, "\"faults_path\": \"/")) {
        return 1;
    }

    if (allocate_port_block(&environment_base, &flight_control_base) != 0) {
        return 1;
    }
    (void)snprintf(bad_output_dir, sizeof(bad_output_dir), "/tmp/missile_manager_bad_%ld", (long)getpid());
    (void)snprintf(bad_runtime_path, sizeof(bad_runtime_path), "%s_runtime.json", bad_output_dir);
    if (write_bad_flight_control_runtime(
            bad_runtime_path,
            argv[2],
            bad_output_dir,
            argv[3],
            argv[4],
            environment_base,
            flight_control_base) != 0) {
        return 1;
    }
    if (run_manager_expect_failure(argv[1], argv[2], bad_runtime_path) != 0) {
        return 1;
    }
    (void)snprintf(path, sizeof(path), "%s/campaign_summary.json", bad_output_dir);
    if (!text_file_contains(path, "\"failed_count\": 1") ||
        !text_file_contains(path, "fc_exited_before_ready")) {
        return 1;
    }

    if (allocate_port_block(&environment_base, &flight_control_base) != 0) {
        return 1;
    }
    if (bind_udp_port(flight_control_base, &held_socket) != 0) {
        return 1;
    }
    (void)snprintf(port_output_dir, sizeof(port_output_dir), "/tmp/missile_manager_port_%ld", (long)getpid());
    (void)snprintf(port_runtime_path, sizeof(port_runtime_path), "%s_runtime.json", port_output_dir);
    if (write_runtime(
            port_runtime_path,
            argv[2],
            port_output_dir,
            argv[3],
            argv[4],
            environment_base,
            flight_control_base) != 0) {
        (void)close(held_socket);
        return 1;
    }
    if (run_manager_expect_failure(argv[1], argv[2], port_runtime_path) != 0) {
        (void)close(held_socket);
        return 1;
    }
    (void)close(held_socket);
    held_socket = -1;

    if (allocate_port_block(&environment_base, &flight_control_base) != 0) {
        return 1;
    }
    (void)snprintf(stop_output_dir, sizeof(stop_output_dir), "/tmp/missile_manager_stop_%ld", (long)getpid());
    (void)snprintf(stop_runtime_path, sizeof(stop_runtime_path), "%s_runtime.json", stop_output_dir);
    if (write_stop_on_failure_runtime(
            stop_runtime_path,
            argv[2],
            stop_output_dir,
            argv[3],
            argv[4],
            environment_base,
            flight_control_base) != 0) {
        return 1;
    }
    if (run_manager_expect_failure(argv[1], argv[2], stop_runtime_path) != 0) {
        return 1;
    }
    (void)snprintf(path, sizeof(path), "%s/campaign_summary.json", stop_output_dir);
    if (!text_file_contains(path, "\"failed_count\": 2") ||
        !text_file_contains(path, "fc_exited_before_ready") ||
        !text_file_contains(path, "skipped_after_failure")) {
        return 1;
    }
    (void)unlink(runtime_path);
    (void)unlink(bad_runtime_path);
    (void)unlink(port_runtime_path);
    (void)unlink(stop_runtime_path);
    return 0;
}
