/** @file main.c
 *  @brief 环境仿真进程入口。
 */
#include "env/env_app.h"
#include "env/env_config.h"

#include "common/status.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static void print_usage(const char *argv0)
{
    (void)printf(
        "usage: %s [--instance-id N] [--scenario PATH] [--runtime PATH] [--faults PATH] [--random-seed N]\n",
        argv0);
}

/** @brief 解析环境进程参数并运行指定实例。 */
int main(int argc, char **argv)
{
    EnvContext ctx = {
        0u,
        ENV_DEFAULT_SCENARIO_CONFIG,
        ENV_DEFAULT_RUNTIME_CONFIG,
        ENV_DEFAULT_FAULTS_CONFIG,
        0,
        0u
    };
    SimStatus status;
    int i;

    for (i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "--help") == 0) {
            print_usage(argv[0]);
            return 0;
        }
        if (strcmp(argv[i], "--instance-id") == 0 && (i + 1) < argc) {
            ctx.instance_id = (uint32_t)strtoul(argv[++i], 0, 10);
            continue;
        }
        if (strcmp(argv[i], "--scenario") == 0 && (i + 1) < argc) {
            ctx.scenario_path = argv[++i];
            continue;
        }
        if (strcmp(argv[i], "--runtime") == 0 && (i + 1) < argc) {
            ctx.runtime_path = argv[++i];
            continue;
        }
        if (strcmp(argv[i], "--faults") == 0 && (i + 1) < argc) {
            ctx.faults_path = argv[++i];
            continue;
        }
        if (strcmp(argv[i], "--random-seed") == 0 && (i + 1) < argc) {
            ctx.random_seed_override = (uint64_t)strtoull(argv[++i], 0, 10);
            ctx.has_random_seed_override = 1;
            continue;
        }
        print_usage(argv[0]);
        return 2;
    }

    status = env_app_run(&ctx);
    return status == SIM_OK ? 0 : 1;
}
