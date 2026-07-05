/** @file p8_tools_test.c
 *  @brief P8 回放、日志比较和批量统计工具集成测试。
 */
#define _POSIX_C_SOURCE 200809L

#include "common/packet.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/wait.h>
#include <unistd.h>

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

static SensorFrame make_sensor(uint32_t seq, double sim_time)
{
    SensorFrame sensor;

    (void)memset(&sensor, 0, sizeof(sensor));
    sensor.seq = seq;
    sensor.sim_time = sim_time;
    sensor.dt = 0.01;
    sensor.missile_vel_ecef_meas = vec3_make(300.0, 0.0, 0.0);
    sensor.missile_accel_ecef_meas = vec3_make(0.0, 0.0, 0.0);
    sensor.missile_gyro_b_meas = vec3_make(0.0, 0.0, 0.0);
    sensor.missile_lat_rad_meas = 0.5;
    sensor.missile_lon_rad_meas = 2.0;
    sensor.missile_height_m_meas = 1000.0;
    sensor.missile_height_agl_m_meas = 1000.0;
    sensor.target_range_meas = 1000.0;
    sensor.target_los_unit_ecef_meas = vec3_make(1.0, 0.0, 0.0);
    sensor.target_los_rate_ecef_meas = vec3_make(0.0, 0.0, 0.1);
    sensor.target_closing_velocity_meas = 100.0;
    sensor.sensor_valid_flags =
        SIM_SENSOR_VALID_SEEKER |
        SIM_SENSOR_VALID_IMU_GYRO |
        SIM_SENSOR_VALID_ACCEL |
        SIM_SENSOR_VALID_SPEED |
        SIM_SENSOR_VALID_GEODETIC;
    return sensor;
}

static int write_sensor_log(const char *path)
{
    FILE *file = fopen(path, "wb");
    uint32_t seq;

    if (file == 0) {
        return -1;
    }
    for (seq = 1u; seq <= 3u; ++seq) {
        SensorFrame sensor = make_sensor(seq, 0.01 * (double)seq);
        unsigned char packet[SIM_SENSOR_PACKET_WIRE_SIZE];
        size_t packet_size = 0u;

        if (packet_encode_sensor_frame(0u, &sensor, packet, sizeof(packet), &packet_size) != SIM_OK ||
            fwrite(packet, 1u, packet_size, file) != packet_size) {
            (void)fclose(file);
            return -1;
        }
    }
    return fclose(file);
}

static int write_summary(const char *path, int hit, double miss_distance)
{
    FILE *file = fopen(path, "wb");

    if (file == 0) {
        return -1;
    }
    (void)fprintf(file, "{\n");
    (void)fprintf(file, "  \"hit_flag\": %s,\n", hit != 0 ? "true" : "false");
    (void)fprintf(file, "  \"miss_distance\": %.6f,\n", miss_distance);
    (void)fprintf(file, "  \"time_of_closest_approach\": 1.000000,\n");
    (void)fprintf(file, "  \"simulation_steps\": 10,\n");
    (void)fprintf(file, "  \"exit_reason\": \"%s\",\n", hit != 0 ? "hit" : "timeout");
    (void)fprintf(file, "  \"fault_start_count\": 1,\n");
    (void)fprintf(file, "  \"fault_end_count\": 1,\n");
    (void)fprintf(file, "  \"fault_sensor_affected_step_count\": 2,\n");
    (void)fprintf(file, "  \"fault_actuator_affected_step_count\": 3,\n");
    (void)fprintf(file, "  \"diagnostic_sample_count\": 4,\n");
    (void)fprintf(file, "  \"max_quat_norm_error\": %.12e,\n", hit != 0 ? 1.0e-9 : 2.0e-9);
    (void)fprintf(file, "  \"max_dcm_orthogonality_error\": %.12e,\n", hit != 0 ? 3.0e-9 : 1.0e-9);
    (void)fprintf(file, "  \"min_mass_kg\": %.9f,\n", hit != 0 ? 90.0 : 80.0);
    (void)fprintf(file, "  \"min_inertia_diag_kgm2\": %.9f,\n", hit != 0 ? 4.0 : 3.0);
    (void)fprintf(file, "  \"aero_model_flags_or\": %u,\n", hit != 0 ? 1u : 2u);
    (void)fprintf(file, "  \"model_degradation_flags_or\": %u,\n", hit != 0 ? 4u : 8u);
    (void)fprintf(file, "  \"aero_extrapolated_sample_count\": %u\n", hit != 0 ? 2u : 5u);
    (void)fprintf(file, "}\n");
    return fclose(file);
}

static int run_replay(
    const char *program,
    const char *config_path,
    const char *input_path,
    const char *output_path)
{
    pid_t pid = fork();
    int status = 0;

    if (pid == 0) {
        execl(
            program,
            program,
            "--instance-id",
            "0",
            "--config",
            config_path,
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

static int run_compare(
    const char *program,
    const char *left_path,
    const char *right_path,
    const char *output_path,
    int expect_success)
{
    pid_t pid = fork();
    int status = 0;

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
    if (pid <= 0 || waitpid(pid, &status, 0) != pid || !WIFEXITED(status)) {
        return -1;
    }
    if (expect_success != 0) {
        return WEXITSTATUS(status) == 0 ? 0 : -1;
    }
    return WEXITSTATUS(status) != 0 ? 0 : -1;
}

static int run_batch_stats(
    const char *program,
    const char *left_summary,
    const char *right_summary,
    const char *output_path)
{
    pid_t pid = fork();
    int status = 0;

    if (pid == 0) {
        execl(
            program,
            program,
            "--input",
            left_summary,
            "--input",
            right_summary,
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

int main(int argc, char **argv)
{
    char sensor_log[256];
    char command_log[256];
    char compare_pass[256];
    char compare_fail[256];
    char truncated_log[256];
    char summary_a[256];
    char summary_b[256];
    char stats_json[256];
    FILE *source;
    FILE *truncated;
    unsigned char packet[SIM_CONTROL_PACKET_WIRE_SIZE];

    if (argc != 5) {
        return 2;
    }
    (void)snprintf(sensor_log, sizeof(sensor_log), "/tmp/missile_p8_sensor_%ld.bin", (long)getpid());
    (void)snprintf(command_log, sizeof(command_log), "/tmp/missile_p8_command_%ld.bin", (long)getpid());
    (void)snprintf(compare_pass, sizeof(compare_pass), "/tmp/missile_p8_compare_pass_%ld.json", (long)getpid());
    (void)snprintf(compare_fail, sizeof(compare_fail), "/tmp/missile_p8_compare_fail_%ld.json", (long)getpid());
    (void)snprintf(truncated_log, sizeof(truncated_log), "/tmp/missile_p8_command_bad_%ld.bin", (long)getpid());
    (void)snprintf(summary_a, sizeof(summary_a), "/tmp/missile_p8_summary_a_%ld.json", (long)getpid());
    (void)snprintf(summary_b, sizeof(summary_b), "/tmp/missile_p8_summary_b_%ld.json", (long)getpid());
    (void)snprintf(stats_json, sizeof(stats_json), "/tmp/missile_p8_stats_%ld.json", (long)getpid());

    if (write_sensor_log(sensor_log) != 0) {
        return 1;
    }
    if (run_replay(argv[1], argv[4], sensor_log, command_log) != 0) {
        return 1;
    }
    if (run_compare(argv[2], command_log, command_log, compare_pass, 1) != 0 ||
        !text_file_contains(compare_pass, "\"verdict\": \"PASS\"")) {
        return 1;
    }

    source = fopen(command_log, "rb");
    truncated = fopen(truncated_log, "wb");
    if (source == 0 || truncated == 0) {
        return 1;
    }
    if (fread(packet, 1u, sizeof(packet), source) != sizeof(packet) ||
        fwrite(packet, 1u, sizeof(packet) - 1u, truncated) != sizeof(packet) - 1u) {
        (void)fclose(source);
        (void)fclose(truncated);
        return 1;
    }
    (void)fclose(source);
    (void)fclose(truncated);
    if (run_compare(argv[2], command_log, truncated_log, compare_fail, 0) != 0 ||
        !text_file_contains(compare_fail, "\"verdict\": \"FAIL\"")) {
        return 1;
    }

    if (write_summary(summary_a, 1, 1.0) != 0 ||
        write_summary(summary_b, 0, 3.0) != 0 ||
        run_batch_stats(argv[3], summary_a, summary_b, stats_json) != 0) {
        return 1;
    }
    if (!text_file_contains(stats_json, "\"run_count\": 2") ||
        !text_file_contains(stats_json, "\"hit_count\": 1") ||
        !text_file_contains(stats_json, "\"miss_distance_mean\": 2.000000000") ||
        !text_file_contains(stats_json, "\"total_fault_sensor_affected_step_count\": 4") ||
        !text_file_contains(stats_json, "\"total_diagnostic_sample_count\": 8") ||
        !text_file_contains(stats_json, "\"max_quat_norm_error\": 2.000000000000e-09") ||
        !text_file_contains(stats_json, "\"min_mass_kg\": 80.000000000") ||
        !text_file_contains(stats_json, "\"aero_model_flags_or\": 3") ||
        !text_file_contains(stats_json, "\"model_degradation_flags_or\": 12") ||
        !text_file_contains(stats_json, "\"total_aero_extrapolated_sample_count\": 7") ||
        !text_file_contains(stats_json, "\"total_campaign_wall_time_s\": 0.000000") ||
        !text_file_contains(stats_json, "\"max_instance_wall_time_s\": 0.000000")) {
        return 1;
    }

    (void)unlink(sensor_log);
    (void)unlink(command_log);
    (void)unlink(compare_pass);
    (void)unlink(compare_fail);
    (void)unlink(truncated_log);
    (void)unlink(summary_a);
    (void)unlink(summary_b);
    (void)unlink(stats_json);
    return 0;
}
