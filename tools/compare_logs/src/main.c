/** @file main.c
 *  @brief 比较协议二进制日志并输出回归判定。
 */
#include "common/config.h"
#include "common/packet.h"
#include "common/status.h"
#include "common/vec3.h"

#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef enum CompareKind {
    COMPARE_COMMAND = 0,
    COMPARE_SENSOR = 1,
    COMPARE_TRAJECTORY = 2
} CompareKind;

enum {
    COMPARE_CSV_LINE_SIZE = 8192,
    COMPARE_CSV_MAX_COLUMNS = 128
};

typedef struct CompareOptions {
    const char *left_path;
    const char *right_path;
    const char *output_path;
    const char *tolerance_config_path;
    uint32_t instance_id;
    double abs_tol;
    double rel_tol;
    CompareKind kind;
} CompareOptions;

typedef struct CompareStats {
    uint32_t frame_count_left;
    uint32_t frame_count_right;
    uint32_t first_divergent_frame;
    uint32_t status_bit_mismatch_count;
    uint32_t mode_mismatch_count;
    double max_abs_error;
    double max_rel_error;
    int diverged;
} CompareStats;

static void print_usage(const char *argv0)
{
    (void)fprintf(
        stderr,
        "usage: %s --type command|sensor|trajectory --instance-id N --left A --right B "
        "[--tolerance-config PATH] [--abs-tol X] [--rel-tol X] [--output result.json]\n",
        argv0);
}

/** @brief 读取版本化日志比较容差。 */
static SimStatus load_tolerance_config(const char *path, double *abs_tol, double *rel_tol)
{
    ConfigTree tree;
    SimStatus status;

    if (path == 0 || abs_tol == 0 || rel_tol == 0) {
        return SIM_ERR_INVALID_ARG;
    }
    status = config_load_file(path, &tree);
    if (status != SIM_OK) {
        return status;
    }
    status = config_validate_json(&tree);
    if (status == SIM_OK) {
        status = config_validate_schema(&tree, 1u);
    }
    if (status == SIM_OK) {
        status = config_get_double(&tree, "compare_logs.abs_tol", abs_tol);
    }
    if (status == SIM_OK) {
        status = config_get_double(&tree, "compare_logs.rel_tol", rel_tol);
    }
    config_free(&tree);
    if (status == SIM_OK &&
        (!isfinite(*abs_tol) || !isfinite(*rel_tol) || *abs_tol < 0.0 || *rel_tol < 0.0)) {
        return SIM_ERR_OUT_OF_RANGE;
    }
    return status;
}

static SimStatus parse_args(int argc, char **argv, CompareOptions *out)
{
    int i;

    if (out == 0) {
        return SIM_ERR_INVALID_ARG;
    }
    (void)memset(out, 0, sizeof(*out));
    out->kind = COMPARE_COMMAND;
    out->abs_tol = 1.0e-9;
    out->rel_tol = 1.0e-9;
    for (i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "--tolerance-config") == 0 && (i + 1) < argc) {
            out->tolerance_config_path = argv[++i];
        }
    }
    if (out->tolerance_config_path != 0 &&
        load_tolerance_config(out->tolerance_config_path, &out->abs_tol, &out->rel_tol) != SIM_OK) {
        return SIM_ERR_CONFIG;
    }
    for (i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "--help") == 0) {
            print_usage(argv[0]);
            exit(0);
        }
        if (strcmp(argv[i], "--type") == 0 && (i + 1) < argc) {
            const char *kind = argv[++i];

            if (strcmp(kind, "command") == 0) {
                out->kind = COMPARE_COMMAND;
            } else if (strcmp(kind, "sensor") == 0) {
                out->kind = COMPARE_SENSOR;
            } else if (strcmp(kind, "trajectory") == 0) {
                out->kind = COMPARE_TRAJECTORY;
            } else {
                return SIM_ERR_CONFIG;
            }
            continue;
        }
        if (strcmp(argv[i], "--instance-id") == 0 && (i + 1) < argc) {
            out->instance_id = (uint32_t)strtoul(argv[++i], 0, 10);
            continue;
        }
        if (strcmp(argv[i], "--left") == 0 && (i + 1) < argc) {
            out->left_path = argv[++i];
            continue;
        }
        if (strcmp(argv[i], "--right") == 0 && (i + 1) < argc) {
            out->right_path = argv[++i];
            continue;
        }
        if (strcmp(argv[i], "--output") == 0 && (i + 1) < argc) {
            out->output_path = argv[++i];
            continue;
        }
        if (strcmp(argv[i], "--tolerance-config") == 0 && (i + 1) < argc) {
            ++i;
            continue;
        }
        if (strcmp(argv[i], "--abs-tol") == 0 && (i + 1) < argc) {
            out->abs_tol = strtod(argv[++i], 0);
            continue;
        }
        if (strcmp(argv[i], "--rel-tol") == 0 && (i + 1) < argc) {
            out->rel_tol = strtod(argv[++i], 0);
            continue;
        }
        return SIM_ERR_CONFIG;
    }
    return out->left_path != 0 && out->right_path != 0 &&
            isfinite(out->abs_tol) && out->abs_tol >= 0.0 &&
            isfinite(out->rel_tol) && out->rel_tol >= 0.0 ?
        SIM_OK :
        SIM_ERR_CONFIG;
}

static double max_double(double a, double b)
{
    return a > b ? a : b;
}

static int compare_double(
    double left,
    double right,
    const CompareOptions *options,
    CompareStats *stats)
{
    const double abs_error = fabs(left - right);
    const double scale = max_double(fabs(left), fabs(right));
    const double rel_error = scale > 0.0 ? abs_error / scale : abs_error;
    const double allowed = options->abs_tol + (options->rel_tol * scale);

    if (abs_error > stats->max_abs_error) {
        stats->max_abs_error = abs_error;
    }
    if (rel_error > stats->max_rel_error) {
        stats->max_rel_error = rel_error;
    }
    return abs_error <= allowed;
}

static int compare_vec3(
    Vec3 left,
    Vec3 right,
    const CompareOptions *options,
    CompareStats *stats)
{
    int ok = 1;

    ok = compare_double(left.x, right.x, options, stats) && ok;
    ok = compare_double(left.y, right.y, options, stats) && ok;
    ok = compare_double(left.z, right.z, options, stats) && ok;
    return ok;
}

static int compare_command(
    const ControlCommand *left,
    const ControlCommand *right,
    const CompareOptions *options,
    CompareStats *stats)
{
    size_t index;
    int ok = 1;

    if (left->seq != right->seq) {
        ok = 0;
    }
    ok = compare_double(left->sim_time, right->sim_time, options, stats) && ok;
    ok = compare_vec3(left->accel_cmd_ecef, right->accel_cmd_ecef, options, stats) && ok;
    ok = compare_vec3(left->attitude_cmd, right->attitude_cmd, options, stats) && ok;
    ok = compare_vec3(left->body_rate_cmd, right->body_rate_cmd, options, stats) && ok;
    for (index = 0u; index < SIM_MAX_ACTUATORS; ++index) {
        ok = compare_double(left->actuator_cmd[index], right->actuator_cmd[index], options, stats) && ok;
    }
    if (left->command_mode != right->command_mode) {
        ++stats->mode_mismatch_count;
        ok = 0;
    }
    if (left->command_status != right->command_status) {
        ++stats->status_bit_mismatch_count;
        ok = 0;
    }
    return ok;
}

static int compare_sensor(
    const SensorFrame *left,
    const SensorFrame *right,
    const CompareOptions *options,
    CompareStats *stats)
{
    int ok = 1;

    if (left->seq != right->seq) {
        ok = 0;
    }
    ok = compare_double(left->sim_time, right->sim_time, options, stats) && ok;
    ok = compare_double(left->dt, right->dt, options, stats) && ok;
    ok = compare_vec3(left->missile_vel_ecef_meas, right->missile_vel_ecef_meas, options, stats) && ok;
    ok = compare_vec3(left->missile_accel_ecef_meas, right->missile_accel_ecef_meas, options, stats) && ok;
    ok = compare_vec3(left->missile_gyro_b_meas, right->missile_gyro_b_meas, options, stats) && ok;
    ok = compare_double(left->missile_lat_rad_meas, right->missile_lat_rad_meas, options, stats) && ok;
    ok = compare_double(left->missile_lon_rad_meas, right->missile_lon_rad_meas, options, stats) && ok;
    ok = compare_double(left->missile_height_m_meas, right->missile_height_m_meas, options, stats) && ok;
    ok = compare_double(left->missile_height_agl_m_meas, right->missile_height_agl_m_meas, options, stats) && ok;
    ok = compare_double(left->target_range_meas, right->target_range_meas, options, stats) && ok;
    ok = compare_vec3(left->target_los_unit_ecef_meas, right->target_los_unit_ecef_meas, options, stats) && ok;
    ok = compare_vec3(left->target_los_rate_ecef_meas, right->target_los_rate_ecef_meas, options, stats) && ok;
    ok = compare_double(
             left->target_closing_velocity_meas,
             right->target_closing_velocity_meas,
             options,
             stats) &&
        ok;
    if (left->sensor_valid_flags != right->sensor_valid_flags ||
        left->sensor_fault_flags != right->sensor_fault_flags) {
        ++stats->status_bit_mismatch_count;
        ok = 0;
    }
    return ok;
}

static SimStatus compare_binary_logs(const CompareOptions *options, CompareStats *stats)
{
    FILE *left;
    FILE *right;
    SimStatus status = SIM_OK;
    const size_t packet_size = options->kind == COMPARE_COMMAND ?
        SIM_CONTROL_PACKET_WIRE_SIZE :
        SIM_SENSOR_PACKET_WIRE_SIZE;
    unsigned char left_packet[SIM_SENSOR_PACKET_WIRE_SIZE];
    unsigned char right_packet[SIM_SENSOR_PACKET_WIRE_SIZE];

    (void)memset(stats, 0, sizeof(*stats));
    stats->first_divergent_frame = UINT32_MAX;
    left = fopen(options->left_path, "rb");
    if (left == 0) {
        return SIM_ERR_IO;
    }
    right = fopen(options->right_path, "rb");
    if (right == 0) {
        (void)fclose(left);
        return SIM_ERR_IO;
    }
    for (;;) {
        const size_t left_got = fread(left_packet, 1u, packet_size, left);
        const size_t right_got = fread(right_packet, 1u, packet_size, right);
        int frames_match = 1;

        if (left_got == 0u && right_got == 0u) {
            if (ferror(left) != 0 || ferror(right) != 0) {
                status = SIM_ERR_IO;
            }
            break;
        }
        if (left_got == packet_size) {
            ++stats->frame_count_left;
        }
        if (right_got == packet_size) {
            ++stats->frame_count_right;
        }
        if (left_got != packet_size || right_got != packet_size) {
            stats->diverged = 1;
            if (stats->first_divergent_frame == UINT32_MAX) {
                stats->first_divergent_frame = max_double(
                    (double)stats->frame_count_left,
                    (double)stats->frame_count_right) > 0.0 ?
                    (uint32_t)max_double(
                        (double)stats->frame_count_left,
                        (double)stats->frame_count_right) :
                    0u;
            }
            status = SIM_ERR_BAD_PACKET;
            break;
        }
        if (options->kind == COMPARE_COMMAND) {
            ControlCommand left_command;
            ControlCommand right_command;

            status = packet_decode_control_command(
                left_packet,
                packet_size,
                options->instance_id,
                &left_command);
            if (status == SIM_OK) {
                status = packet_decode_control_command(
                    right_packet,
                    packet_size,
                    options->instance_id,
                    &right_command);
            }
            if (status == SIM_OK) {
                frames_match = compare_command(&left_command, &right_command, options, stats);
            }
        } else {
            SensorFrame left_sensor;
            SensorFrame right_sensor;

            status = packet_decode_sensor_frame(
                left_packet,
                packet_size,
                options->instance_id,
                &left_sensor);
            if (status == SIM_OK) {
                status = packet_decode_sensor_frame(
                    right_packet,
                    packet_size,
                    options->instance_id,
                    &right_sensor);
            }
            if (status == SIM_OK) {
                frames_match = compare_sensor(&left_sensor, &right_sensor, options, stats);
            }
        }
        if (options->abs_tol == 0.0 && options->rel_tol == 0.0 &&
            memcmp(left_packet, right_packet, packet_size) != 0) {
            frames_match = 0;
        }
        if (status != SIM_OK || frames_match == 0) {
            stats->diverged = 1;
            if (stats->first_divergent_frame == UINT32_MAX) {
                stats->first_divergent_frame = stats->frame_count_left;
            }
            if (status != SIM_OK) {
                break;
            }
        }
    }
    (void)fclose(left);
    (void)fclose(right);
    if (stats->first_divergent_frame == UINT32_MAX) {
        stats->first_divergent_frame = 0u;
    }
    return status == SIM_ERR_BAD_PACKET ? SIM_OK : status;
}

/** @brief 将一行无引号数值 CSV 解析为浮点数组。 */
static SimStatus parse_numeric_csv_row(
    char *line,
    double *values,
    size_t capacity,
    size_t *count_out)
{
    char *cursor = line;
    size_t count = 0u;

    if (line == 0 || values == 0 || count_out == 0) {
        return SIM_ERR_INVALID_ARG;
    }
    while (*cursor != '\0' && *cursor != '\n' && *cursor != '\r') {
        char *end = 0;
        double value;

        if (count >= capacity) {
            return SIM_ERR_OUT_OF_RANGE;
        }
        errno = 0;
        value = strtod(cursor, &end);
        if (end == cursor || errno == ERANGE || !isfinite(value)) {
            return SIM_ERR_BAD_PACKET;
        }
        while (*end == ' ' || *end == '\t') {
            ++end;
        }
        values[count++] = value;
        if (*end == ',') {
            cursor = end + 1;
            if (*cursor == '\0' || *cursor == '\n' || *cursor == '\r') {
                return SIM_ERR_BAD_PACKET;
            }
            continue;
        }
        if (*end == '\0' || *end == '\n' || *end == '\r') {
            cursor = end;
            break;
        }
        return SIM_ERR_BAD_PACKET;
    }
    if (count == 0u) {
        return SIM_ERR_BAD_PACKET;
    }
    *count_out = count;
    return SIM_OK;
}

/** @brief 比较环境真值 trajectory.csv。 */
static SimStatus compare_trajectory_logs(const CompareOptions *options, CompareStats *stats)
{
    FILE *left = fopen(options->left_path, "rb");
    FILE *right;
    char left_line[COMPARE_CSV_LINE_SIZE];
    char right_line[COMPARE_CSV_LINE_SIZE];
    SimStatus status = SIM_OK;

    (void)memset(stats, 0, sizeof(*stats));
    stats->first_divergent_frame = UINT32_MAX;
    if (left == 0) {
        return SIM_ERR_IO;
    }
    right = fopen(options->right_path, "rb");
    if (right == 0) {
        (void)fclose(left);
        return SIM_ERR_IO;
    }
    if (fgets(left_line, sizeof(left_line), left) == 0 ||
        fgets(right_line, sizeof(right_line), right) == 0) {
        status = SIM_ERR_BAD_PACKET;
    } else if (strcmp(left_line, right_line) != 0) {
        stats->diverged = 1;
        stats->first_divergent_frame = 0u;
    }
    while (status == SIM_OK) {
        char *left_result = fgets(left_line, sizeof(left_line), left);
        char *right_result = fgets(right_line, sizeof(right_line), right);
        double left_values[COMPARE_CSV_MAX_COLUMNS];
        double right_values[COMPARE_CSV_MAX_COLUMNS];
        size_t left_count = 0u;
        size_t right_count = 0u;
        size_t index;
        int frames_match = 1;

        if (left_result == 0 && right_result == 0) {
            if (ferror(left) != 0 || ferror(right) != 0) {
                status = SIM_ERR_IO;
            }
            break;
        }
        if (left_result != 0) {
            ++stats->frame_count_left;
        }
        if (right_result != 0) {
            ++stats->frame_count_right;
        }
        if (left_result == 0 || right_result == 0) {
            stats->diverged = 1;
            stats->first_divergent_frame = stats->frame_count_left > stats->frame_count_right ?
                stats->frame_count_left : stats->frame_count_right;
            break;
        }
        if ((strchr(left_line, '\n') == 0 && feof(left) == 0) ||
            (strchr(right_line, '\n') == 0 && feof(right) == 0)) {
            status = SIM_ERR_BAD_PACKET;
            break;
        }
        if (options->abs_tol == 0.0 && options->rel_tol == 0.0 &&
            strcmp(left_line, right_line) != 0) {
            frames_match = 0;
        }
        status = parse_numeric_csv_row(
            left_line,
            left_values,
            COMPARE_CSV_MAX_COLUMNS,
            &left_count);
        if (status == SIM_OK) {
            status = parse_numeric_csv_row(
                right_line,
                right_values,
                COMPARE_CSV_MAX_COLUMNS,
                &right_count);
        }
        if (status != SIM_OK || left_count != right_count) {
            stats->diverged = 1;
            if (stats->first_divergent_frame == UINT32_MAX) {
                stats->first_divergent_frame = stats->frame_count_left;
            }
            if (status == SIM_OK) {
                status = SIM_ERR_BAD_PACKET;
            }
            break;
        }
        for (index = 0u; index < left_count; ++index) {
            frames_match = compare_double(
                left_values[index],
                right_values[index],
                options,
                stats) && frames_match;
        }
        if (frames_match == 0) {
            stats->diverged = 1;
            if (stats->first_divergent_frame == UINT32_MAX) {
                stats->first_divergent_frame = stats->frame_count_left;
            }
        }
    }
    (void)fclose(left);
    (void)fclose(right);
    if (stats->first_divergent_frame == UINT32_MAX) {
        stats->first_divergent_frame = 0u;
    }
    return status == SIM_ERR_BAD_PACKET ? SIM_OK : status;
}

static SimStatus compare_logs(const CompareOptions *options, CompareStats *stats)
{
    return options->kind == COMPARE_TRAJECTORY ?
        compare_trajectory_logs(options, stats) :
        compare_binary_logs(options, stats);
}

static const char *compare_kind_name(CompareKind kind)
{
    switch (kind) {
    case COMPARE_COMMAND:
        return "command";
    case COMPARE_SENSOR:
        return "sensor";
    case COMPARE_TRAJECTORY:
        return "trajectory";
    default:
        return "unknown";
    }
}

static int write_result(const CompareOptions *options, const CompareStats *stats, SimStatus status)
{
    FILE *out = stdout;
    const int pass = status == SIM_OK && stats->diverged == 0;

    if (options->output_path != 0) {
        out = fopen(options->output_path, "wb");
        if (out == 0) {
            return -1;
        }
    }
    (void)fprintf(out, "{\n");
    (void)fprintf(out, "  \"verdict\": \"%s\",\n", pass ? "PASS" : "FAIL");
    (void)fprintf(out, "  \"status\": \"%s\",\n", sim_status_to_string(status));
    (void)fprintf(out, "  \"type\": \"%s\",\n", compare_kind_name(options->kind));
    (void)fprintf(
        out,
        "  \"comparison_mode\": \"%s\",\n",
        options->abs_tol == 0.0 && options->rel_tol == 0.0 ? "EXACT" : "TOLERANCE");
    (void)fprintf(out, "  \"abs_tol\": %.17g,\n", options->abs_tol);
    (void)fprintf(out, "  \"rel_tol\": %.17g,\n", options->rel_tol);
    (void)fprintf(
        out,
        "  \"tolerance_config\": %s",
        options->tolerance_config_path != 0 ? "\"" : "null");
    if (options->tolerance_config_path != 0) {
        (void)fprintf(out, "%s\"", options->tolerance_config_path);
    }
    (void)fprintf(out, ",\n");
    (void)fprintf(out, "  \"frame_count_left\": %u,\n", stats->frame_count_left);
    (void)fprintf(out, "  \"frame_count_right\": %u,\n", stats->frame_count_right);
    (void)fprintf(out, "  \"first_divergent_frame\": %u,\n", stats->first_divergent_frame);
    (void)fprintf(out, "  \"max_abs_error\": %.17g,\n", stats->max_abs_error);
    (void)fprintf(out, "  \"max_rel_error\": %.17g,\n", stats->max_rel_error);
    (void)fprintf(out, "  \"status_bit_mismatch_count\": %u,\n", stats->status_bit_mismatch_count);
    (void)fprintf(out, "  \"mode_mismatch_count\": %u\n", stats->mode_mismatch_count);
    (void)fprintf(out, "}\n");
    if (options->output_path != 0 && fclose(out) != 0) {
        return -1;
    }
    return pass ? 0 : 1;
}

int main(int argc, char **argv)
{
    CompareOptions options;
    CompareStats stats;
    SimStatus status = parse_args(argc, argv, &options);

    if (status != SIM_OK) {
        print_usage(argv[0]);
        return 2;
    }
    status = compare_logs(&options, &stats);
    return write_result(&options, &stats, status) == 0 ? 0 : 1;
}
