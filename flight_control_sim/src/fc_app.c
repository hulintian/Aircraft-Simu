/** @file fc_app.c
 *  @brief 飞控主循环实现。
 *
 *  该模块负责加载配置、接收传感器帧、执行 PNG 制导并向环境程序发出
 *  控制指令。
 */
#include "fc/fc_app.h"

#include "common/config.h"
#include "common/logger.h"
#include "common/packet.h"
#include "common/protocol.h"
#include "common/status.h"
#include "fc/guidance_png.h"

#include <arpa/inet.h>
#include <errno.h>
#include <netinet/in.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

#define FC_PACKET_BUFFER_SIZE 1024u

typedef struct FcRuntimeConfig {
    /** @brief 环境进程 UDP 基础端口。 */
    unsigned int env_base_port;
    /** @brief 飞控进程 UDP 基础端口。 */
    unsigned int fc_base_port;
    /** @brief 环境目标 IPv4 地址。 */
    char host[64];
} FcRuntimeConfig;

/** @brief 从运行时配置中读取网络参数。 */
static SimStatus load_runtime_config(const ConfigTree *runtime, FcRuntimeConfig *out)
{
    SimStatus status;

    if (runtime == 0 || out == 0) {
        return SIM_ERR_INVALID_ARG;
    }

    status = config_get_uint32(runtime, "network.environment_base_port", &out->env_base_port);
    if (status != SIM_OK) {
        return status;
    }
    status = config_get_uint32(runtime, "network.flight_control_base_port", &out->fc_base_port);
    if (status != SIM_OK) {
        return status;
    }
    status = config_get_string(runtime, "network.host", out->host, sizeof(out->host));
    return status;
}

/** @brief 从飞控配置中读取 PNG 参数。 */
static SimStatus load_guidance_config(const ConfigTree *config, GuidancePngConfig *out)
{
    SimStatus status;

    if (config == 0 || out == 0) {
        return SIM_ERR_INVALID_ARG;
    }
    status = config_get_double(config, "guidance.navigation_constant", &out->navigation_constant);
    if (status != SIM_OK) {
        return status;
    }
    status = config_get_double(config, "guidance.max_accel_mps2", &out->max_accel_mps2);
    if (status != SIM_OK) {
        return status;
    }
    status = config_get_double(config, "guidance.max_accel_rate_mps3", &out->max_accel_rate_mps3);
    return status;
}

/** @brief 绑定 UDP 监听套接字。 */
static SimStatus bind_udp_socket(unsigned int port, int *sock_out)
{
    int sock;
    struct sockaddr_in addr;
    struct timeval timeout;

    if (sock_out == 0 || port > 65535u) {
        return SIM_ERR_INVALID_ARG;
    }

    sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
        return SIM_ERR_IO;
    }

    {
        int reuse = 1;
        (void)setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
    }
    timeout.tv_sec = 2;
    timeout.tv_usec = 0;
    (void)setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));

    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons((uint16_t)port);
    if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) != 0) {
        (void)close(sock);
        return SIM_ERR_IO;
    }

    *sock_out = sock;
    return SIM_OK;
}

/** @brief 序列化并发送控制指令。 */
static SimStatus send_control_command(
    int sock,
    const struct sockaddr_in *peer,
    uint32_t instance_id,
    const ControlCommand *command)
{
    unsigned char buffer[SIM_CONTROL_PACKET_WIRE_SIZE];
    size_t packet_size;
    SimStatus status;
    ssize_t sent;

    if (peer == 0 || command == 0) {
        return SIM_ERR_INVALID_ARG;
    }

    status = packet_encode_control_command(
        instance_id,
        command,
        buffer,
        sizeof(buffer),
        &packet_size);
    if (status != SIM_OK) {
        return status;
    }
    sent = sendto(sock, buffer, packet_size, 0, (const struct sockaddr *)peer, sizeof(*peer));
    if (sent != (ssize_t)packet_size) {
        return SIM_ERR_IO;
    }
    return SIM_OK;
}

/** @brief 运行飞控仿真主循环。 */
SimStatus fc_app_run(const FcContext *ctx)
{
    ConfigTree fc_tree;
    ConfigTree runtime_tree;
    FcRuntimeConfig runtime_cfg;
    GuidancePngConfig guidance_cfg;
    Logger logger;
    SimStatus status;
    int sock = -1;
    unsigned int fc_port;
    unsigned int env_port;
    uint32_t last_seq = 0u;
    int have_seq = 0;

    if (ctx == 0 || ctx->flight_control_path == 0 || ctx->runtime_path == 0) {
        return SIM_ERR_INVALID_ARG;
    }

    memset(&fc_tree, 0, sizeof(fc_tree));
    memset(&runtime_tree, 0, sizeof(runtime_tree));
    status = logger_open_stdout(&logger);
    if (status != SIM_OK) {
        return status;
    }
    status = config_load_file(ctx->flight_control_path, &fc_tree);
    if (status != SIM_OK) {
        (void)fprintf(stderr, "flight_control_sim: failed to load %s: %s\n",
            ctx->flight_control_path,
            sim_status_to_string(status));
        return status;
    }
    status = config_load_file(ctx->runtime_path, &runtime_tree);
    if (status != SIM_OK) {
        (void)fprintf(stderr, "flight_control_sim: failed to load %s: %s\n",
            ctx->runtime_path,
            sim_status_to_string(status));
        config_free(&fc_tree);
        return status;
    }
    status = config_validate_schema(&fc_tree, 1u);
    if (status == SIM_OK) {
        status = config_require_section(&fc_tree, "guidance");
    }
    if (status == SIM_OK) {
        status = config_require_section(&fc_tree, "safety");
    }
    if (status == SIM_OK) {
        status = config_validate_schema(&runtime_tree, 1u);
    }
    if (status == SIM_OK) {
        status = config_require_section(&runtime_tree, "network");
    }
    if (status != SIM_OK) {
        (void)fprintf(stderr, "flight_control_sim: schema validation failed: %s\n",
            sim_status_to_string(status));
        config_free(&fc_tree);
        config_free(&runtime_tree);
        return status;
    }
    status = load_runtime_config(&runtime_tree, &runtime_cfg);
    if (status != SIM_OK) {
        (void)fprintf(stderr, "flight_control_sim: invalid runtime config: %s\n",
            sim_status_to_string(status));
        config_free(&fc_tree);
        config_free(&runtime_tree);
        return status;
    }
    status = load_guidance_config(&fc_tree, &guidance_cfg);
    if (status != SIM_OK) {
        (void)fprintf(stderr, "flight_control_sim: invalid guidance config: %s\n",
            sim_status_to_string(status));
        config_free(&fc_tree);
        config_free(&runtime_tree);
        return status;
    }

    fc_port = runtime_cfg.fc_base_port + (2u * ctx->instance_id);
    env_port = runtime_cfg.env_base_port + (2u * ctx->instance_id);
    status = bind_udp_socket(fc_port, &sock);
    if (status != SIM_OK) {
        (void)fprintf(stderr, "flight_control_sim: failed to bind UDP port %u: %s\n",
            fc_port,
            sim_status_to_string(status));
        config_free(&fc_tree);
        config_free(&runtime_tree);
        return status;
    }

    (void)logger_info(&logger, "flight_control_sim UDP loop started");
    (void)printf("instance_id=%u fc_port=%u env_port=%u\n", ctx->instance_id, fc_port, env_port);

    for (;;) {
        unsigned char buffer[FC_PACKET_BUFFER_SIZE];
        struct sockaddr_in from;
        socklen_t from_len = sizeof(from);
        ssize_t got = recvfrom(sock, buffer, sizeof(buffer), 0, (struct sockaddr *)&from, &from_len);
        SensorFrame sensor;
        GuidancePngInput guidance_in;
        GuidancePngOutput guidance_out;
        ControlCommand command;

        if (got < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                break;
            }
            status = SIM_ERR_IO;
            break;
        }
        if ((size_t)got != SIM_SENSOR_PACKET_WIRE_SIZE) {
            (void)fprintf(stderr, "flight_control_sim: dropped packet size=%ld expected=%zu\n",
                (long)got,
                (size_t)SIM_SENSOR_PACKET_WIRE_SIZE);
            continue;
        }
        status = packet_decode_sensor_frame(buffer, (size_t)got, ctx->instance_id, &sensor);
        if (status != SIM_OK) {
            (void)fprintf(stderr, "flight_control_sim: dropped packet status=%s\n",
                sim_status_to_string(status));
            continue;
        }
        if (have_seq != 0 && sensor.seq <= last_seq) {
            continue;
        }
        have_seq = 1;
        last_seq = sensor.seq;

        memset(&command, 0, sizeof(command));
        command.seq = sensor.seq;
        command.sim_time = sensor.sim_time;
        if ((sensor.sensor_valid_flags & SIM_SENSOR_VALID_SEEKER) != 0u) {
            guidance_in.range_m = sensor.target_range_meas;
            guidance_in.closing_velocity_mps =
                sensor.target_closing_velocity_meas;
            guidance_in.los_unit_ecef = sensor.target_los_unit_ecef_meas;
            guidance_in.los_rate_ecef = sensor.target_los_rate_ecef_meas;
            status = guidance_png_update(
                &guidance_cfg,
                &guidance_in,
                &guidance_out);
            if (status == SIM_OK) {
                command.accel_cmd_ecef = guidance_out.accel_cmd_ecef;
                command.command_status = 0u;
            } else {
                command.command_status = 1u;
            }
        } else {
            /*
             * LOCKSTEP 仍需对每个传感器帧返回一条命令。导引头延迟预热、
             * 丢包或遮挡期间输出受控零加速度，避免使用无效测量。
             */
            command.command_status = 2u;
            status = SIM_OK;
        }
        from.sin_port = htons((uint16_t)env_port);
        status = send_control_command(sock, &from, ctx->instance_id, &command);
        if (status != SIM_OK) {
            break;
        }
    }

    if (sock >= 0) {
        (void)close(sock);
    }
    config_free(&fc_tree);
    config_free(&runtime_tree);
    (void)logger_info(&logger, "flight_control_sim stopped");
    return status == SIM_ERR_TIMEOUT ? SIM_OK : status;
}
