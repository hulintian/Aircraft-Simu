/** @file control_quality_test.c
 *  @brief 标准机动控制品质报告集成测试。
 */
#define _POSIX_C_SOURCE 200809L

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/wait.h>
#include <unistd.h>

/** @brief 判断报告是否包含全部通过标志和三个工况。 */
static int report_is_complete(const char *path)
{
    char buffer[8192];
    FILE *file = fopen(path, "rb");
    size_t size;

    if (file == 0) {
        return 0;
    }
    size = fread(buffer, 1u, sizeof(buffer) - 1u, file);
    (void)fclose(file);
    buffer[size] = '\0';
    return strstr(buffer, "\"pass\": true") != 0 &&
        strstr(buffer, "\"acceleration_step\"") != 0 &&
        strstr(buffer, "\"command_reversal\"") != 0 &&
        strstr(buffer, "\"dropout_recovery\"") != 0 &&
        strstr(buffer, "engineering_sil_baseline_not_model_fidelity_validation") != 0;
}

int main(int argc, char **argv)
{
    char output_path[256];
    char command[1536];
    int status;

    if (argc != 4) {
        return 2;
    }
    (void)snprintf(output_path, sizeof(output_path), "/tmp/missile_control_quality_%ld.json", (long)getpid());
    (void)snprintf(
        command,
        sizeof(command),
        "%s --flight-control %s --criteria %s --output %s",
        argv[1],
        argv[2],
        argv[3],
        output_path);
    status = system(command);
    if (status == -1 || !WIFEXITED(status) || WEXITSTATUS(status) != 0 ||
        !report_is_complete(output_path)) {
        return 1;
    }
    (void)unlink(output_path);
    return 0;
}
