/** @file main.c
 *  @brief 汇总 summary.json 和 campaign_summary.json 的批量统计工具。
 */
#include "common/config.h"
#include "common/status.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MAX_INPUTS 256u
#define MAX_FAILURE_REASONS 64u
#define FAILURE_REASON_SIZE 64u
#define FAILURE_SOURCE_SIZE 512u

typedef struct BatchOptions {
    const char *inputs[MAX_INPUTS];
    size_t input_count;
    const char *output_path;
} BatchOptions;

typedef struct FailureReasonCount {
    char reason[FAILURE_REASON_SIZE];
    unsigned int count;
} FailureReasonCount;

typedef struct FailedInstance {
    char source[FAILURE_SOURCE_SIZE];
    unsigned int instance_id;
    char reason[FAILURE_REASON_SIZE];
} FailedInstance;

typedef struct BatchStats {
    unsigned int run_count;
    unsigned int completed_count;
    unsigned int failed_count;
    unsigned int hit_count;
    unsigned int summary_available_count;
    unsigned int total_fault_start_count;
    unsigned int total_fault_end_count;
    unsigned int total_fault_sensor_affected_step_count;
    unsigned int total_fault_actuator_affected_step_count;
    unsigned int total_diagnostic_sample_count;
    unsigned int aero_model_flags_or;
    unsigned int model_degradation_flags_or;
    unsigned int total_aero_extrapolated_sample_count;
    unsigned int miss_sample_count;
    unsigned int unattributed_failure_count;
    double total_campaign_wall_time_s;
    double total_instance_wall_time_s;
    double max_instance_wall_time_s;
    double miss_min;
    double miss_max;
    double miss_sum;
    double miss_square_sum;
    double max_quat_norm_error;
    double max_dcm_orthogonality_error;
    double min_mass_kg;
    double min_inertia_diag_kgm2;
    FailureReasonCount failure_reasons[MAX_FAILURE_REASONS];
    size_t failure_reason_count;
    FailedInstance *failed_instances;
    size_t failed_instance_count;
    size_t failed_instance_capacity;
} BatchStats;

static void print_usage(const char *argv0)
{
    (void)fprintf(
        stderr,
        "usage: %s --input summary.json [--input campaign_summary.json ...] [--output stats.json]\n",
        argv0);
}

static SimStatus parse_args(int argc, char **argv, BatchOptions *out)
{
    int i;

    if (out == 0) {
        return SIM_ERR_INVALID_ARG;
    }
    (void)memset(out, 0, sizeof(*out));
    for (i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "--help") == 0) {
            print_usage(argv[0]);
            exit(0);
        }
        if (strcmp(argv[i], "--input") == 0 && (i + 1) < argc) {
            if (out->input_count >= MAX_INPUTS) {
                return SIM_ERR_OUT_OF_RANGE;
            }
            out->inputs[out->input_count++] = argv[++i];
            continue;
        }
        if (strcmp(argv[i], "--output") == 0 && (i + 1) < argc) {
            out->output_path = argv[++i];
            continue;
        }
        return SIM_ERR_CONFIG;
    }
    return out->input_count > 0u ? SIM_OK : SIM_ERR_CONFIG;
}

static void init_stats(BatchStats *stats)
{
    (void)memset(stats, 0, sizeof(*stats));
    stats->miss_min = HUGE_VAL;
    stats->miss_max = -HUGE_VAL;
    stats->min_mass_kg = HUGE_VAL;
    stats->min_inertia_diag_kgm2 = HUGE_VAL;
}

static void add_miss_distance(BatchStats *stats, double miss_distance)
{
    if (miss_distance < stats->miss_min) {
        stats->miss_min = miss_distance;
    }
    if (miss_distance > stats->miss_max) {
        stats->miss_max = miss_distance;
    }
    stats->miss_sum += miss_distance;
    stats->miss_square_sum += miss_distance * miss_distance;
    ++stats->miss_sample_count;
}

/** @brief 记录失败实例并累计失败原因分布。 */
static SimStatus add_failed_instance(
    BatchStats *stats,
    const char *source,
    unsigned int instance_id,
    const char *reason)
{
    const char *effective_reason = reason != 0 && reason[0] != '\0' ? reason : "process_failed";
    size_t index;

    for (index = 0u; index < stats->failure_reason_count; ++index) {
        if (strcmp(stats->failure_reasons[index].reason, effective_reason) == 0) {
            ++stats->failure_reasons[index].count;
            break;
        }
    }
    if (index == stats->failure_reason_count) {
        if (stats->failure_reason_count >= MAX_FAILURE_REASONS) {
            return SIM_ERR_OUT_OF_RANGE;
        }
        (void)snprintf(
            stats->failure_reasons[index].reason,
            sizeof(stats->failure_reasons[index].reason),
            "%s",
            effective_reason);
        stats->failure_reasons[index].count = 1u;
        ++stats->failure_reason_count;
    }
    if (stats->failed_instance_count == stats->failed_instance_capacity) {
        const size_t new_capacity = stats->failed_instance_capacity == 0u ?
            16u : stats->failed_instance_capacity * 2u;
        FailedInstance *resized;

        if (new_capacity < stats->failed_instance_capacity ||
            new_capacity > ((size_t)-1) / sizeof(*resized)) {
            return SIM_ERR_OUT_OF_RANGE;
        }
        resized = (FailedInstance *)realloc(
            stats->failed_instances,
            new_capacity * sizeof(*resized));
        if (resized == 0) {
            return SIM_ERR_INTERNAL;
        }
        stats->failed_instances = resized;
        stats->failed_instance_capacity = new_capacity;
    }
    (void)snprintf(
        stats->failed_instances[stats->failed_instance_count].source,
        sizeof(stats->failed_instances[stats->failed_instance_count].source),
        "%s",
        source);
    stats->failed_instances[stats->failed_instance_count].instance_id = instance_id;
    (void)snprintf(
        stats->failed_instances[stats->failed_instance_count].reason,
        sizeof(stats->failed_instances[stats->failed_instance_count].reason),
        "%s",
        effective_reason);
    ++stats->failed_instance_count;
    return SIM_OK;
}

static void add_summary(const ConfigTree *tree, BatchStats *stats)
{
    int hit_flag = 0;
    double miss_distance = 0.0;
    double metric = 0.0;
    unsigned int value = 0u;

    ++stats->run_count;
    ++stats->completed_count;
    ++stats->summary_available_count;
    (void)config_get_bool(tree, "hit_flag", &hit_flag);
    if (hit_flag != 0) {
        ++stats->hit_count;
    }
    if (config_get_double(tree, "miss_distance", &miss_distance) == SIM_OK) {
        add_miss_distance(stats, miss_distance);
    }
    if (config_get_uint32(tree, "fault_start_count", &value) == SIM_OK) {
        stats->total_fault_start_count += value;
    }
    if (config_get_uint32(tree, "fault_end_count", &value) == SIM_OK) {
        stats->total_fault_end_count += value;
    }
    if (config_get_uint32(tree, "fault_sensor_affected_step_count", &value) == SIM_OK) {
        stats->total_fault_sensor_affected_step_count += value;
    }
    if (config_get_uint32(tree, "fault_actuator_affected_step_count", &value) == SIM_OK) {
        stats->total_fault_actuator_affected_step_count += value;
    }
    if (config_get_uint32(tree, "diagnostic_sample_count", &value) == SIM_OK) {
        stats->total_diagnostic_sample_count += value;
    }
    if (config_get_double(tree, "max_quat_norm_error", &metric) == SIM_OK &&
        metric > stats->max_quat_norm_error) {
        stats->max_quat_norm_error = metric;
    }
    if (config_get_double(tree, "max_dcm_orthogonality_error", &metric) == SIM_OK &&
        metric > stats->max_dcm_orthogonality_error) {
        stats->max_dcm_orthogonality_error = metric;
    }
    if (config_get_double(tree, "min_mass_kg", &metric) == SIM_OK &&
        metric > 0.0 &&
        metric < stats->min_mass_kg) {
        stats->min_mass_kg = metric;
    }
    if (config_get_double(tree, "min_inertia_diag_kgm2", &metric) == SIM_OK &&
        metric > 0.0 &&
        metric < stats->min_inertia_diag_kgm2) {
        stats->min_inertia_diag_kgm2 = metric;
    }
    if (config_get_uint32(tree, "aero_model_flags_or", &value) == SIM_OK) {
        stats->aero_model_flags_or |= value;
    }
    if (config_get_uint32(tree, "model_degradation_flags_or", &value) == SIM_OK) {
        stats->model_degradation_flags_or |= value;
    }
    if (config_get_uint32(tree, "aero_extrapolated_sample_count", &value) == SIM_OK) {
        stats->total_aero_extrapolated_sample_count += value;
    }
}

static SimStatus add_campaign(const ConfigTree *tree, const char *source, BatchStats *stats)
{
    unsigned int value = 0u;
    unsigned int campaign_failed_count = 0u;
    unsigned int detailed_failed_count = 0u;
    double metric = 0.0;
    size_t instance_count = 0u;
    size_t index;
    int has_instance_details = 0;

    if (config_get_uint32(tree, "instance_count", &value) == SIM_OK) {
        stats->run_count += value;
    }
    if (config_get_uint32(tree, "completed_count", &value) == SIM_OK) {
        stats->completed_count += value;
    }
    if (config_get_uint32(tree, "failed_count", &value) == SIM_OK) {
        stats->failed_count += value;
        campaign_failed_count = value;
    }
    if (config_get_uint32(tree, "summary_available_count", &value) == SIM_OK) {
        stats->summary_available_count += value;
    }
    if (config_get_uint32(tree, "hit_count", &value) == SIM_OK) {
        stats->hit_count += value;
    }
    if (config_get_array_count(tree, "instances", &instance_count) == SIM_OK) {
        has_instance_details = 1;
        for (index = 0u; index < instance_count; ++index) {
            char path[96];
            int summary_available = 0;
            unsigned int env_status = 0u;
            unsigned int fc_status = 0u;
            unsigned int instance_id = (unsigned int)index;
            double miss_distance = 0.0;

            (void)snprintf(path, sizeof(path), "instances[%u].summary_available", (unsigned int)index);
            (void)config_get_bool(tree, path, &summary_available);
            if (summary_available != 0) {
                (void)snprintf(path, sizeof(path), "instances[%u].miss_distance", (unsigned int)index);
                if (config_get_double(tree, path, &miss_distance) == SIM_OK) {
                    add_miss_distance(stats, miss_distance);
                }
            }
            (void)snprintf(path, sizeof(path), "instances[%u].env_status", (unsigned int)index);
            if (config_get_uint32(tree, path, &env_status) != SIM_OK) {
                continue;
            }
            (void)snprintf(path, sizeof(path), "instances[%u].fc_status", (unsigned int)index);
            if (config_get_uint32(tree, path, &fc_status) != SIM_OK ||
                (env_status == 0u && fc_status == 0u)) {
                continue;
            }
            {
                char reason[FAILURE_REASON_SIZE] = "process_failed";
                SimStatus status;

                (void)snprintf(path, sizeof(path), "instances[%u].instance_id", (unsigned int)index);
                (void)config_get_uint32(tree, path, &instance_id);
                (void)snprintf(path, sizeof(path), "instances[%u].exit_reason", (unsigned int)index);
                (void)config_get_string(tree, path, reason, sizeof(reason));
                status = add_failed_instance(stats, source, instance_id, reason);
                if (status != SIM_OK) {
                    return status;
                }
                ++detailed_failed_count;
            }
        }
    }
    if (campaign_failed_count > detailed_failed_count) {
        stats->unattributed_failure_count += campaign_failed_count - detailed_failed_count;
    }
    if (has_instance_details == 0 && stats->summary_available_count > 0u) {
        double miss_distance = 0.0;

        if (config_get_double(tree, "min_miss_distance", &miss_distance) == SIM_OK) {
            add_miss_distance(stats, miss_distance);
        }
    }
    if (config_get_uint32(tree, "total_fault_start_count", &value) == SIM_OK) {
        stats->total_fault_start_count += value;
    }
    if (config_get_uint32(tree, "total_fault_end_count", &value) == SIM_OK) {
        stats->total_fault_end_count += value;
    }
    if (config_get_uint32(tree, "total_fault_sensor_affected_step_count", &value) == SIM_OK) {
        stats->total_fault_sensor_affected_step_count += value;
    }
    if (config_get_uint32(tree, "total_fault_actuator_affected_step_count", &value) == SIM_OK) {
        stats->total_fault_actuator_affected_step_count += value;
    }
    if (config_get_uint32(tree, "total_diagnostic_sample_count", &value) == SIM_OK) {
        stats->total_diagnostic_sample_count += value;
    }
    if (config_get_double(tree, "max_quat_norm_error", &metric) == SIM_OK &&
        metric > stats->max_quat_norm_error) {
        stats->max_quat_norm_error = metric;
    }
    if (config_get_double(tree, "max_dcm_orthogonality_error", &metric) == SIM_OK &&
        metric > stats->max_dcm_orthogonality_error) {
        stats->max_dcm_orthogonality_error = metric;
    }
    if (config_get_double(tree, "min_mass_kg", &metric) == SIM_OK &&
        metric > 0.0 &&
        metric < stats->min_mass_kg) {
        stats->min_mass_kg = metric;
    }
    if (config_get_double(tree, "min_inertia_diag_kgm2", &metric) == SIM_OK &&
        metric > 0.0 &&
        metric < stats->min_inertia_diag_kgm2) {
        stats->min_inertia_diag_kgm2 = metric;
    }
    if (config_get_uint32(tree, "aero_model_flags_or", &value) == SIM_OK) {
        stats->aero_model_flags_or |= value;
    }
    if (config_get_uint32(tree, "model_degradation_flags_or", &value) == SIM_OK) {
        stats->model_degradation_flags_or |= value;
    }
    if (config_get_uint32(tree, "total_aero_extrapolated_sample_count", &value) == SIM_OK) {
        stats->total_aero_extrapolated_sample_count += value;
    }
    if (config_get_double(tree, "campaign_wall_time_s", &metric) == SIM_OK && metric > 0.0) {
        stats->total_campaign_wall_time_s += metric;
    }
    if (config_get_double(tree, "total_instance_wall_time_s", &metric) == SIM_OK && metric > 0.0) {
        stats->total_instance_wall_time_s += metric;
    }
    if (config_get_double(tree, "max_instance_wall_time_s", &metric) == SIM_OK &&
        metric > stats->max_instance_wall_time_s) {
        stats->max_instance_wall_time_s = metric;
    }
    return SIM_OK;
}

static SimStatus add_input_file(const char *path, BatchStats *stats)
{
    ConfigTree tree;
    SimStatus status;
    unsigned int campaign_instance_count = 0u;

    (void)memset(&tree, 0, sizeof(tree));
    status = config_load_file(path, &tree);
    if (status != SIM_OK) {
        return status;
    }
    if (config_get_uint32(&tree, "instance_count", &campaign_instance_count) == SIM_OK) {
        status = add_campaign(&tree, path, stats);
    } else {
        add_summary(&tree, stats);
        status = SIM_OK;
    }
    config_free(&tree);
    return status;
}

/** @brief 写出带必要转义的 JSON 字符串。 */
static void write_json_string(FILE *out, const char *value)
{
    const unsigned char *cursor = (const unsigned char *)value;

    (void)fputc('"', out);
    while (*cursor != '\0') {
        switch (*cursor) {
        case '"':
            (void)fputs("\\\"", out);
            break;
        case '\\':
            (void)fputs("\\\\", out);
            break;
        case '\n':
            (void)fputs("\\n", out);
            break;
        case '\r':
            (void)fputs("\\r", out);
            break;
        case '\t':
            (void)fputs("\\t", out);
            break;
        default:
            if (*cursor < 0x20u) {
                (void)fprintf(out, "\\u%04x", (unsigned int)*cursor);
            } else {
                (void)fputc((int)*cursor, out);
            }
            break;
        }
        ++cursor;
    }
    (void)fputc('"', out);
}

static int write_stats(const BatchOptions *options, const BatchStats *stats)
{
    FILE *out = stdout;
    const double hit_rate = stats->run_count > 0u ?
        (double)stats->hit_count / (double)stats->run_count :
        0.0;
    const double miss_mean = stats->miss_sample_count > 0u ?
        stats->miss_sum / (double)stats->miss_sample_count :
        0.0;
    double variance = 0.0;
    double miss_std = 0.0;
    size_t index;

    if (stats->miss_sample_count > 0u) {
        variance = (stats->miss_square_sum / (double)stats->miss_sample_count) -
            (miss_mean * miss_mean);
        if (variance < 0.0 && variance > -1.0e-12) {
            variance = 0.0;
        }
        miss_std = variance > 0.0 ? sqrt(variance) : 0.0;
    }
    if (options->output_path != 0) {
        out = fopen(options->output_path, "wb");
        if (out == 0) {
            return -1;
        }
    }
    (void)fprintf(out, "{\n");
    (void)fprintf(out, "  \"run_count\": %u,\n", stats->run_count);
    (void)fprintf(out, "  \"completed_count\": %u,\n", stats->completed_count);
    (void)fprintf(out, "  \"failed_count\": %u,\n", stats->failed_count);
    (void)fprintf(out, "  \"summary_available_count\": %u,\n", stats->summary_available_count);
    (void)fprintf(out, "  \"miss_distance_sample_count\": %u,\n", stats->miss_sample_count);
    (void)fprintf(out, "  \"hit_count\": %u,\n", stats->hit_count);
    (void)fprintf(out, "  \"hit_rate\": %.9f,\n", hit_rate);
    (void)fprintf(
        out,
        "  \"miss_distance_min\": %.9f,\n",
        stats->miss_sample_count > 0u ? stats->miss_min : 0.0);
    (void)fprintf(
        out,
        "  \"miss_distance_max\": %.9f,\n",
        stats->miss_sample_count > 0u ? stats->miss_max : 0.0);
    (void)fprintf(out, "  \"miss_distance_mean\": %.9f,\n", miss_mean);
    (void)fprintf(out, "  \"miss_distance_std\": %.9f,\n", miss_std);
    (void)fprintf(out, "  \"total_fault_start_count\": %u,\n", stats->total_fault_start_count);
    (void)fprintf(out, "  \"total_fault_end_count\": %u,\n", stats->total_fault_end_count);
    (void)fprintf(
        out,
        "  \"total_fault_sensor_affected_step_count\": %u,\n",
        stats->total_fault_sensor_affected_step_count);
    (void)fprintf(
        out,
        "  \"total_fault_actuator_affected_step_count\": %u,\n",
        stats->total_fault_actuator_affected_step_count);
    (void)fprintf(out, "  \"total_diagnostic_sample_count\": %u,\n", stats->total_diagnostic_sample_count);
    (void)fprintf(out, "  \"max_quat_norm_error\": %.12e,\n", stats->max_quat_norm_error);
    (void)fprintf(
        out,
        "  \"max_dcm_orthogonality_error\": %.12e,\n",
        stats->max_dcm_orthogonality_error);
    (void)fprintf(
        out,
        "  \"min_mass_kg\": %.9f,\n",
        stats->total_diagnostic_sample_count > 0u ? stats->min_mass_kg : 0.0);
    (void)fprintf(
        out,
        "  \"min_inertia_diag_kgm2\": %.9f,\n",
        stats->total_diagnostic_sample_count > 0u ? stats->min_inertia_diag_kgm2 : 0.0);
    (void)fprintf(out, "  \"aero_model_flags_or\": %u,\n", stats->aero_model_flags_or);
    (void)fprintf(
        out,
        "  \"model_degradation_flags_or\": %u,\n",
        stats->model_degradation_flags_or);
    (void)fprintf(
        out,
        "  \"total_aero_extrapolated_sample_count\": %u,\n",
        stats->total_aero_extrapolated_sample_count);
    (void)fprintf(out, "  \"total_campaign_wall_time_s\": %.6f,\n", stats->total_campaign_wall_time_s);
    (void)fprintf(out, "  \"total_instance_wall_time_s\": %.6f,\n", stats->total_instance_wall_time_s);
    (void)fprintf(out, "  \"max_instance_wall_time_s\": %.6f,\n", stats->max_instance_wall_time_s);
    (void)fprintf(out, "  \"failed_instance_count\": %zu,\n", stats->failed_instance_count);
    (void)fprintf(
        out,
        "  \"unattributed_failure_count\": %u,\n",
        stats->unattributed_failure_count);
    (void)fprintf(out, "  \"failure_reason_distribution\": [\n");
    for (index = 0u; index < stats->failure_reason_count; ++index) {
        (void)fprintf(out, "    { \"reason\": ");
        write_json_string(out, stats->failure_reasons[index].reason);
        (void)fprintf(
            out,
            ", \"count\": %u }%s\n",
            stats->failure_reasons[index].count,
            index + 1u == stats->failure_reason_count ? "" : ",");
    }
    (void)fprintf(out, "  ],\n");
    (void)fprintf(out, "  \"failed_instances\": [\n");
    for (index = 0u; index < stats->failed_instance_count; ++index) {
        (void)fprintf(out, "    { \"source\": ");
        write_json_string(out, stats->failed_instances[index].source);
        (void)fprintf(
            out,
            ", \"instance_id\": %u, \"reason\": ",
            stats->failed_instances[index].instance_id);
        write_json_string(out, stats->failed_instances[index].reason);
        (void)fprintf(
            out,
            " }%s\n",
            index + 1u == stats->failed_instance_count ? "" : ",");
    }
    (void)fprintf(out, "  ]\n");
    (void)fprintf(out, "}\n");
    if (options->output_path != 0 && fclose(out) != 0) {
        return -1;
    }
    return 0;
}

int main(int argc, char **argv)
{
    BatchOptions options;
    BatchStats stats;
    size_t index;
    SimStatus status = parse_args(argc, argv, &options);

    if (status != SIM_OK) {
        print_usage(argv[0]);
        return 2;
    }
    init_stats(&stats);
    for (index = 0u; index < options.input_count; ++index) {
        status = add_input_file(options.inputs[index], &stats);
        if (status != SIM_OK) {
            (void)fprintf(
                stderr,
                "batch_stats: failed to read %s: %s\n",
                options.inputs[index],
                sim_status_to_string(status));
            free(stats.failed_instances);
            return 1;
        }
    }
    {
        const int result = write_stats(&options, &stats) == 0 ? 0 : 1;

        free(stats.failed_instances);
        return result;
    }
}
