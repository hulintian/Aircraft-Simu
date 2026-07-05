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

typedef struct BatchOptions {
    const char *inputs[MAX_INPUTS];
    size_t input_count;
    const char *output_path;
} BatchOptions;

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

static void add_campaign(const ConfigTree *tree, BatchStats *stats)
{
    unsigned int value = 0u;
    double miss_distance = 0.0;
    double metric = 0.0;

    if (config_get_uint32(tree, "instance_count", &value) == SIM_OK) {
        stats->run_count += value;
    }
    if (config_get_uint32(tree, "completed_count", &value) == SIM_OK) {
        stats->completed_count += value;
    }
    if (config_get_uint32(tree, "failed_count", &value) == SIM_OK) {
        stats->failed_count += value;
    }
    if (config_get_uint32(tree, "summary_available_count", &value) == SIM_OK) {
        stats->summary_available_count += value;
    }
    if (config_get_uint32(tree, "hit_count", &value) == SIM_OK) {
        stats->hit_count += value;
    }
    if (config_get_double(tree, "min_miss_distance", &miss_distance) == SIM_OK &&
        stats->summary_available_count > 0u) {
        add_miss_distance(stats, miss_distance);
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
        add_campaign(&tree, stats);
    } else {
        add_summary(&tree, stats);
    }
    config_free(&tree);
    return SIM_OK;
}

static int write_stats(const BatchOptions *options, const BatchStats *stats)
{
    FILE *out = stdout;
    const double hit_rate = stats->run_count > 0u ?
        (double)stats->hit_count / (double)stats->run_count :
        0.0;
    const double miss_mean = stats->summary_available_count > 0u ?
        stats->miss_sum / (double)stats->summary_available_count :
        0.0;
    double variance = 0.0;
    double miss_std = 0.0;

    if (stats->summary_available_count > 0u) {
        variance = (stats->miss_square_sum / (double)stats->summary_available_count) -
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
    (void)fprintf(out, "  \"hit_count\": %u,\n", stats->hit_count);
    (void)fprintf(out, "  \"hit_rate\": %.9f,\n", hit_rate);
    (void)fprintf(
        out,
        "  \"miss_distance_min\": %.9f,\n",
        stats->summary_available_count > 0u ? stats->miss_min : 0.0);
    (void)fprintf(
        out,
        "  \"miss_distance_max\": %.9f,\n",
        stats->summary_available_count > 0u ? stats->miss_max : 0.0);
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
    (void)fprintf(out, "  \"max_instance_wall_time_s\": %.6f\n", stats->max_instance_wall_time_s);
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
            return 1;
        }
    }
    return write_stats(&options, &stats) == 0 ? 0 : 1;
}
