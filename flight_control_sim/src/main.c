/** @file main.c
 *  @brief 飞控仿真进程入口。
 */
#include "fc/fc_app.h"
#include "fc/fc_config.h"

#include "common/status.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static void print_usage(const char *argv0)
{
    (void)printf("usage: %s [--instance-id N] [--config PATH] [--runtime PATH]\n", argv0);
}

/** @brief 解析飞控进程参数并运行指定实例。 */
int main(int argc, char **argv)
{
    FcContext ctx = {
        0u,
        FC_DEFAULT_CONFIG,
        FC_DEFAULT_RUNTIME_CONFIG
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
        if (strcmp(argv[i], "--config") == 0 && (i + 1) < argc) {
            ctx.flight_control_path = argv[++i];
            continue;
        }
        if (strcmp(argv[i], "--runtime") == 0 && (i + 1) < argc) {
            ctx.runtime_path = argv[++i];
            continue;
        }
        print_usage(argv[0]);
        return 2;
    }

    status = fc_app_run(&ctx);
    return status == SIM_OK ? 0 : 1;
}
