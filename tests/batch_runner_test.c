/** @file batch_runner_test.c
 *  @brief batch_runner 工具集成测试。
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

/** @brief 记录布尔断言结果并返回失败计数增量。 */
static int expect(int condition, const char *name)
{
    if (!condition) {
        (void)fprintf(stderr, "failed: %s\n", name);
        return 1;
    }
    return 0;
}

/** @brief 计算文件行数。 */
static int count_lines(const char *path)
{
    FILE *file = fopen(path, "r");
    int lines = 0;
    int ch;

    if (file == 0) {
        return -1;
    }
    while ((ch = fgetc(file)) != EOF) {
        if (ch == '\n') {
            ++lines;
        }
    }
    (void)fclose(file);
    return lines;
}

/** @brief 检查小型文本文件中是否包含给定片段。 */
static int file_contains(const char *path, const char *needle)
{
    char buffer[4096];
    FILE *file = fopen(path, "rb");
    size_t bytes_read;

    if (file == 0 || needle == 0) {
        return 0;
    }
    bytes_read = fread(buffer, 1u, sizeof(buffer) - 1u, file);
    (void)fclose(file);
    buffer[bytes_read] = '\0';
    return strstr(buffer, needle) != 0;
}

int main(int argc, char **argv)
{
    char manifest_path[128];
    char generated_manifest_path[128];
    char generated_run_manifest_path[192];
    char manual_run_manifest_path[192];
    char generated_dir[128];
    char template_path[128];
    char script_path[128];
    char marker_path[128];
    char runtime_a[128];
    char runtime_b[128];
    char generated_runtime_a[160];
    char generated_runtime_b[160];
    char generated_sample_a[160];
    char generated_sample_b[160];
    char command[1024];
    FILE *file;
    int failures = 0;

    if (argc != 2) {
        (void)fprintf(stderr, "usage: %s <batch_runner>\n", argv[0]);
        return 2;
    }
    (void)snprintf(manifest_path, sizeof(manifest_path), "/tmp/missile_batch_%ld.txt", (long)getpid());
    (void)snprintf(generated_manifest_path, sizeof(generated_manifest_path), "/tmp/missile_generated_batch_%ld.txt", (long)getpid());
    (void)snprintf(
        generated_run_manifest_path,
        sizeof(generated_run_manifest_path),
        "%s.run_manifest.json",
        generated_manifest_path);
    (void)snprintf(
        manual_run_manifest_path,
        sizeof(manual_run_manifest_path),
        "%s.run_manifest.json",
        manifest_path);
    (void)snprintf(generated_dir, sizeof(generated_dir), "/tmp/missile_mc_%ld", (long)getpid());
    (void)snprintf(template_path, sizeof(template_path), "/tmp/missile_runtime_template_%ld.json", (long)getpid());
    (void)snprintf(script_path, sizeof(script_path), "/tmp/missile_fake_manager_%ld.sh", (long)getpid());
    (void)snprintf(marker_path, sizeof(marker_path), "/tmp/missile_batch_marker_%ld.txt", (long)getpid());
    (void)snprintf(runtime_a, sizeof(runtime_a), "/tmp/missile_runtime_a_%ld.json", (long)getpid());
    (void)snprintf(runtime_b, sizeof(runtime_b), "/tmp/missile_runtime_b_%ld.json", (long)getpid());
    (void)snprintf(generated_runtime_a, sizeof(generated_runtime_a), "%s/runtime_0000.json", generated_dir);
    (void)snprintf(generated_runtime_b, sizeof(generated_runtime_b), "%s/runtime_0001.json", generated_dir);
    (void)snprintf(generated_sample_a, sizeof(generated_sample_a), "%s/sample_0000", generated_dir);
    (void)snprintf(generated_sample_b, sizeof(generated_sample_b), "%s/sample_0001", generated_dir);

    file = fopen(runtime_a, "w");
    if (file == 0) {
        return 1;
    }
    (void)fprintf(file, "{}\n");
    (void)fclose(file);
    file = fopen(runtime_b, "w");
    if (file == 0) {
        return 1;
    }
    (void)fprintf(file, "{}\n");
    (void)fclose(file);

    file = fopen(script_path, "w");
    if (file == 0) {
        return 1;
    }
    (void)fprintf(file, "#!/bin/sh\n");
    (void)fprintf(file, "if [ \"$1\" != \"--runtime\" ]; then exit 2; fi\n");
    (void)fprintf(file, "echo \"$2\" >> %s\n", marker_path);
    (void)fprintf(file, "exit 0\n");
    (void)fclose(file);
    if (chmod(script_path, 0700) != 0) {
        return 1;
    }

    file = fopen(manifest_path, "w");
    if (file == 0) {
        return 1;
    }
    (void)fprintf(file, "# comment\n");
    (void)fprintf(file, "%s\n", runtime_a);
    (void)fprintf(file, "\n");
    (void)fprintf(file, "%s\n", runtime_b);
    (void)fclose(file);

    (void)snprintf(
        command,
        sizeof(command),
        "%s --manifest %s --instance-manager %s",
        argv[1],
        manifest_path,
        script_path);
    failures += expect(system(command) == 0, "batch_runner_command");
    failures += expect(count_lines(marker_path) == 2, "batch_runner_line_count");
    failures += expect(file_contains(manual_run_manifest_path, "\"run_mode\": \"MONTE_CARLO\""), "batch_runner_manual_run_mode");
    failures += expect(file_contains(manual_run_manifest_path, "\"attempted_count\": 2"), "batch_runner_manual_attempted");

    file = fopen(template_path, "w");
    if (file == 0) {
        return 1;
    }
    (void)fprintf(file, "{\n");
    (void)fprintf(file, "  \"schema_version\": 1,\n");
    (void)fprintf(file, "  \"campaign\": {\"base_random_seed\": ${random_seed}},\n");
    (void)fprintf(file, "  \"sample_index\": ${sample_index},\n");
    (void)fprintf(file, "  \"wind_x\": ${uniform:2:-1.0:1.0},\n");
    (void)fprintf(file, "  \"wind_y\": ${lhs_uniform:10:-2.0:2.0},\n");
    (void)fprintf(file, "  \"target_lat_error_m\": ${halton_uniform:2:-10.0:10.0},\n");
    (void)fprintf(file, "  \"target_altitude_error_m\": ${normal:3:0.0:5.0},\n");
    (void)fprintf(file, "  \"thrust_scale\": ${lognormal:5:0.0:0.1},\n");
    (void)fprintf(file, "  \"drag_delta\": ${truncated_normal:6:0.0:1.0:-2.0:2.0},\n");
    (void)fprintf(file, "  \"integrator_choice\": ${choice:7:\"RK2\"|\"RK4\"},\n");
    (void)fprintf(file, "  \"target_speed_error_mps\": ${correlated_normal:4:3:0.0:2.0:0.75},\n");
    (void)fprintf(file, "  \"logging\": {\"output_dir\": \"${sample_output_dir}\"}\n");
    (void)fprintf(file, "}\n");
    (void)fclose(file);
    (void)snprintf(
        command,
        sizeof(command),
        "%s --generate-manifest %s --runtime-template %s --runtime-output-dir %s --sample-count 2 --base-seed 9000",
        argv[1],
        generated_manifest_path,
        template_path,
        generated_dir);
    failures += expect(system(command) == 0, "batch_runner_generate_command");
    failures += expect(count_lines(generated_manifest_path) == 2, "batch_runner_generated_manifest_lines");
    failures += expect(file_contains(generated_run_manifest_path, "\"run_mode\": \"MONTE_CARLO\""), "batch_runner_generated_run_mode");
    failures += expect(file_contains(generated_run_manifest_path, "\"random_seed\": 9000"), "batch_runner_generated_base_seed");
    failures += expect(file_contains(generated_run_manifest_path, "\"run_count\": 2"), "batch_runner_generated_run_count");
    failures += expect(file_contains(generated_run_manifest_path, "\"generated_only\": true"), "batch_runner_generated_only");
    failures += expect(file_contains(generated_manifest_path, "runtime_0000.json"), "batch_runner_manifest_runtime_a");
    failures += expect(file_contains(generated_manifest_path, "campaign_summary.json"), "batch_runner_manifest_stats_path");
    failures += expect(file_contains(generated_runtime_a, "\"base_random_seed\": 9000"), "batch_runner_seed_a");
    failures += expect(file_contains(generated_runtime_b, "\"base_random_seed\": 9001"), "batch_runner_seed_b");
    failures += expect(file_contains(generated_runtime_b, "\"sample_index\": 1"), "batch_runner_sample_index");
    failures += expect(file_contains(generated_runtime_a, "sample_0000"), "batch_runner_sample_output_dir");
    failures += expect(file_contains(generated_runtime_a, "\"wind_x\":"), "batch_runner_uniform_field");
    failures += expect(!file_contains(generated_runtime_a, "${uniform:"), "batch_runner_uniform_substituted");
    failures += expect(file_contains(generated_runtime_a, "\"wind_y\":"), "batch_runner_lhs_uniform_field");
    failures += expect(
        !file_contains(generated_runtime_a, "${lhs_uniform:"),
        "batch_runner_lhs_uniform_substituted");
    failures += expect(
        file_contains(generated_runtime_a, "\"target_lat_error_m\":"),
        "batch_runner_halton_uniform_field");
    failures += expect(
        !file_contains(generated_runtime_a, "${halton_uniform:"),
        "batch_runner_halton_uniform_substituted");
    failures += expect(
        file_contains(generated_runtime_a, "\"target_altitude_error_m\":"),
        "batch_runner_normal_field");
    failures += expect(!file_contains(generated_runtime_a, "${normal:"), "batch_runner_normal_substituted");
    failures += expect(
        file_contains(generated_runtime_a, "\"thrust_scale\":"),
        "batch_runner_lognormal_field");
    failures += expect(!file_contains(generated_runtime_a, "${lognormal:"), "batch_runner_lognormal_substituted");
    failures += expect(
        file_contains(generated_runtime_a, "\"drag_delta\":"),
        "batch_runner_truncated_normal_field");
    failures += expect(
        !file_contains(generated_runtime_a, "${truncated_normal:"),
        "batch_runner_truncated_normal_substituted");
    failures += expect(
        file_contains(generated_runtime_a, "\"integrator_choice\":"),
        "batch_runner_choice_field");
    failures += expect(!file_contains(generated_runtime_a, "${choice:"), "batch_runner_choice_substituted");
    failures += expect(
        file_contains(generated_runtime_a, "\"target_speed_error_mps\":"),
        "batch_runner_correlated_normal_field");
    failures += expect(
        !file_contains(generated_runtime_a, "${correlated_normal:"),
        "batch_runner_correlated_normal_substituted");

    file = fopen(template_path, "w");
    if (file == 0) {
        return 1;
    }
    (void)fprintf(file, "{\"bad\": ${lognormal:7:0.0:-1.0}}\n");
    (void)fclose(file);
    (void)snprintf(
        command,
        sizeof(command),
        "%s --generate-manifest %s --runtime-template %s --runtime-output-dir %s --sample-count 1 --base-seed 9000",
        argv[1],
        generated_manifest_path,
        template_path,
        generated_dir);
    failures += expect(system(command) != 0, "batch_runner_invalid_lognormal_rejected");

    file = fopen(template_path, "w");
    if (file == 0) {
        return 1;
    }
    (void)fprintf(file, "{\"bad\": ${truncated_normal:8:0.0:1.0:2.0:-2.0}}\n");
    (void)fclose(file);
    (void)snprintf(
        command,
        sizeof(command),
        "%s --generate-manifest %s --runtime-template %s --runtime-output-dir %s --sample-count 1 --base-seed 9000",
        argv[1],
        generated_manifest_path,
        template_path,
        generated_dir);
    failures += expect(system(command) != 0, "batch_runner_invalid_truncated_normal_rejected");

    file = fopen(template_path, "w");
    if (file == 0) {
        return 1;
    }
    (void)fprintf(file, "{\"bad\": ${choice:9:alpha||beta}}\n");
    (void)fclose(file);
    (void)snprintf(
        command,
        sizeof(command),
        "%s --generate-manifest %s --runtime-template %s --runtime-output-dir %s --sample-count 1 --base-seed 9000",
        argv[1],
        generated_manifest_path,
        template_path,
        generated_dir);
    failures += expect(system(command) != 0, "batch_runner_invalid_choice_rejected");

    file = fopen(template_path, "w");
    if (file == 0) {
        return 1;
    }
    (void)fprintf(file, "{\"bad\": ${halton_uniform:1:0.0:1.0}}\n");
    (void)fclose(file);
    (void)snprintf(
        command,
        sizeof(command),
        "%s --generate-manifest %s --runtime-template %s --runtime-output-dir %s --sample-count 1 --base-seed 9000",
        argv[1],
        generated_manifest_path,
        template_path,
        generated_dir);
    failures += expect(system(command) != 0, "batch_runner_invalid_halton_rejected");

    (void)unlink(manifest_path);
    (void)unlink(generated_manifest_path);
    (void)unlink(generated_run_manifest_path);
    (void)unlink(manual_run_manifest_path);
    (void)unlink(template_path);
    (void)unlink(script_path);
    (void)unlink(marker_path);
    (void)unlink(runtime_a);
    (void)unlink(runtime_b);
    (void)unlink(generated_runtime_a);
    (void)unlink(generated_runtime_b);
    (void)rmdir(generated_sample_a);
    (void)rmdir(generated_sample_b);
    (void)rmdir(generated_dir);
    return failures == 0 ? 0 : 1;
}
