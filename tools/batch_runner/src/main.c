/** @file main.c
 *  @brief 批次清单运行入口。
 */
#include "common/build_info.h"
#include "common/provenance.h"
#include "common/status.h"

#include <ctype.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <stdint.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <unistd.h>

#define BATCH_RUNNER_MAX_RUNS 128u
#define BATCH_RUNNER_MAX_PATH 512u
#define BATCH_RUNNER_MAX_LINE 1200u
#define BATCH_RUNNER_MAX_TEMPLATE_BYTES 65536u
#define BATCH_RUNNER_PI 3.141592653589793238462643383279502884

typedef struct BatchRun {
    char runtime_path[BATCH_RUNNER_MAX_PATH];
    char stats_input_path[BATCH_RUNNER_MAX_PATH];
    int has_stats_input;
} BatchRun;

typedef struct BatchRunnerArgs {
    const char *manifest_path;
    const char *generate_manifest_path;
    const char *runtime_template_path;
    const char *runtime_output_dir;
    const char *instance_manager_path;
    const char *batch_stats_path;
    const char *output_path;
    const char *run_manifest_path;
    char default_run_manifest_path[BATCH_RUNNER_MAX_PATH];
    size_t sample_count;
    uint64_t base_seed;
    int base_seed_set;
    int stop_on_failure;
} BatchRunnerArgs;

/** @brief 打印命令行帮助。 */
static void print_usage(const char *program)
{
    (void)fprintf(
        stderr,
        "usage: %s --manifest runs.txt --instance-manager PATH "
        "[--batch-stats PATH --output stats.json] [--stop-on-failure] [--run-manifest PATH]\n"
        "       %s --generate-manifest runs.txt --runtime-template template.json "
        "--runtime-output-dir DIR --sample-count N [--base-seed SEED] "
        "[--instance-manager PATH] [--run-manifest PATH]\n",
        program,
        program);
}

/** @brief 消费带值参数。 */
static int consume_value(int argc, char **argv, int *index, const char **value)
{
    if (index == 0 || value == 0 || *index + 1 >= argc) {
        return 0;
    }
    ++(*index);
    *value = argv[*index];
    return 1;
}

/** @brief 消费无符号整数参数。 */
static int consume_uint64(int argc, char **argv, int *index, uint64_t *value)
{
    char *end = 0;
    unsigned long long parsed;

    if (index == 0 || value == 0 || *index + 1 >= argc) {
        return 0;
    }
    ++(*index);
    errno = 0;
    parsed = strtoull(argv[*index], &end, 10);
    if (errno != 0 || end == argv[*index] || *end != '\0') {
        return 0;
    }
    *value = (uint64_t)parsed;
    return 1;
}

/** @brief 解析命令行参数。 */
static int parse_args(int argc, char **argv, BatchRunnerArgs *args)
{
    int index;
    uint64_t value;

    if (args == 0) {
        return 0;
    }
    (void)memset(args, 0, sizeof(*args));
    for (index = 1; index < argc; ++index) {
        if (strcmp(argv[index], "--manifest") == 0) {
            if (!consume_value(argc, argv, &index, &args->manifest_path)) {
                return 0;
            }
        } else if (strcmp(argv[index], "--generate-manifest") == 0) {
            if (!consume_value(argc, argv, &index, &args->generate_manifest_path)) {
                return 0;
            }
        } else if (strcmp(argv[index], "--runtime-template") == 0) {
            if (!consume_value(argc, argv, &index, &args->runtime_template_path)) {
                return 0;
            }
        } else if (strcmp(argv[index], "--runtime-output-dir") == 0) {
            if (!consume_value(argc, argv, &index, &args->runtime_output_dir)) {
                return 0;
            }
        } else if (strcmp(argv[index], "--sample-count") == 0) {
            if (!consume_uint64(argc, argv, &index, &value) ||
                value == 0u ||
                value > BATCH_RUNNER_MAX_RUNS) {
                return 0;
            }
            args->sample_count = (size_t)value;
        } else if (strcmp(argv[index], "--base-seed") == 0) {
            if (!consume_uint64(argc, argv, &index, &args->base_seed)) {
                return 0;
            }
            args->base_seed_set = 1;
        } else if (strcmp(argv[index], "--instance-manager") == 0) {
            if (!consume_value(argc, argv, &index, &args->instance_manager_path)) {
                return 0;
            }
        } else if (strcmp(argv[index], "--batch-stats") == 0) {
            if (!consume_value(argc, argv, &index, &args->batch_stats_path)) {
                return 0;
            }
        } else if (strcmp(argv[index], "--output") == 0) {
            if (!consume_value(argc, argv, &index, &args->output_path)) {
                return 0;
            }
        } else if (strcmp(argv[index], "--run-manifest") == 0) {
            if (!consume_value(argc, argv, &index, &args->run_manifest_path)) {
                return 0;
            }
        } else if (strcmp(argv[index], "--stop-on-failure") == 0) {
            args->stop_on_failure = 1;
        } else {
            return 0;
        }
    }
    if (args->generate_manifest_path != 0) {
        if (args->manifest_path != 0 ||
            args->runtime_template_path == 0 ||
            args->runtime_output_dir == 0 ||
            args->sample_count == 0u) {
            return 0;
        }
        if (args->base_seed_set == 0) {
            args->base_seed = UINT64_C(1);
        }
    } else {
        if (args->manifest_path == 0 || args->instance_manager_path == 0) {
            return 0;
        }
    }
    if (args->run_manifest_path == 0) {
        const char *manifest_base = args->generate_manifest_path != 0 ?
            args->generate_manifest_path : args->manifest_path;
        const int written = snprintf(
            args->default_run_manifest_path,
            sizeof(args->default_run_manifest_path),
            "%s.run_manifest.json",
            manifest_base);

        if (written < 0 || (size_t)written >= sizeof(args->default_run_manifest_path)) {
            return 0;
        }
        args->run_manifest_path = args->default_run_manifest_path;
    }
    return args->output_path == 0 || args->batch_stats_path != 0;
}

/** @brief 去掉行首尾空白。 */
static char *trim_line(char *line)
{
    char *start = line;
    char *end;

    while (*start != '\0' && isspace((unsigned char)*start) != 0) {
        ++start;
    }
    end = start + strlen(start);
    while (end > start && isspace((unsigned char)end[-1]) != 0) {
        --end;
    }
    *end = '\0';
    return start;
}

/** @brief 读取小型 runtime 模板。 */
static SimStatus read_template_file(const char *path, char **out)
{
    FILE *file;
    long size;
    char *buffer;
    size_t bytes_read;

    if (path == 0 || out == 0) {
        return SIM_ERR_INVALID_ARG;
    }
    *out = 0;
    file = fopen(path, "rb");
    if (file == 0) {
        return SIM_ERR_IO;
    }
    if (fseek(file, 0, SEEK_END) != 0 ||
        (size = ftell(file)) < 0 ||
        (unsigned long)size > BATCH_RUNNER_MAX_TEMPLATE_BYTES ||
        fseek(file, 0, SEEK_SET) != 0) {
        (void)fclose(file);
        return SIM_ERR_OUT_OF_RANGE;
    }
    buffer = (char *)malloc((size_t)size + 1u);
    if (buffer == 0) {
        (void)fclose(file);
        return SIM_ERR_INTERNAL;
    }
    bytes_read = fread(buffer, 1u, (size_t)size, file);
    (void)fclose(file);
    if (bytes_read != (size_t)size) {
        free(buffer);
        return SIM_ERR_IO;
    }
    buffer[(size_t)size] = '\0';
    *out = buffer;
    return SIM_OK;
}

/** @brief 生成确定性 0 到 1 均匀数，用于 Monte Carlo 模板参数扰动。 */
static double deterministic_u01(uint64_t random_seed, uint64_t stream)
{
    uint64_t state = random_seed ^ (stream * UINT64_C(0x9e3779b97f4a7c15));

    state ^= state >> 30u;
    state *= UINT64_C(0xbf58476d1ce4e5b9);
    state ^= state >> 27u;
    state *= UINT64_C(0x94d049bb133111eb);
    state ^= state >> 31u;
    return (double)(state >> 11u) * (1.0 / 9007199254740992.0);
}

/** @brief 生成确定性标准正态变量。 */
static double deterministic_standard_normal(uint64_t random_seed, uint64_t stream)
{
    double u1 = deterministic_u01(random_seed, stream);
    const double u2 = deterministic_u01(random_seed, stream ^ UINT64_C(0x9e3779b97f4a7c15));

    if (u1 < 1.0e-12) {
        u1 = 1.0e-12;
    }
    return sqrt(-2.0 * log(u1)) * cos(2.0 * BATCH_RUNNER_PI * u2);
}

/** @brief 计算最大公约数。 */
static size_t gcd_size(size_t left, size_t right)
{
    while (right != 0u) {
        const size_t temp = left % right;

        left = right;
        right = temp;
    }
    return left;
}

/** @brief 计算指定整数基的 Halton radical inverse。 */
static double halton_radical_inverse(size_t index, unsigned int base)
{
    double inverse_base = 1.0 / (double)base;
    double fraction = inverse_base;
    double result = 0.0;

    while (index > 0u) {
        result += (double)(index % (size_t)base) * fraction;
        index /= (size_t)base;
        fraction *= inverse_base;
    }
    return result;
}

/** @brief 解析 `${uniform:stream:min:max}` Monte Carlo 占位符。 */
static int parse_uniform_placeholder(
    const char *cursor,
    uint64_t random_seed,
    size_t *consumed,
    double *value)
{
    const char prefix[] = "${uniform:";
    const char *end;
    char token[128];
    char *parse;
    char *next;
    uint64_t stream;
    double minimum;
    double maximum;
    size_t token_size;

    if (cursor == 0 || consumed == 0 || value == 0 ||
        strncmp(cursor, prefix, sizeof(prefix) - 1u) != 0) {
        return 0;
    }
    end = strchr(cursor, '}');
    if (end == 0) {
        return -1;
    }
    token_size = (size_t)(end - (cursor + sizeof(prefix) - 1u));
    if (token_size == 0u || token_size >= sizeof(token)) {
        return -1;
    }
    (void)memcpy(token, cursor + sizeof(prefix) - 1u, token_size);
    token[token_size] = '\0';
    parse = token;
    errno = 0;
    stream = strtoull(parse, &next, 10);
    if (errno != 0 || next == parse || *next != ':') {
        return -1;
    }
    parse = next + 1;
    errno = 0;
    minimum = strtod(parse, &next);
    if (errno != 0 || next == parse || *next != ':') {
        return -1;
    }
    parse = next + 1;
    errno = 0;
    maximum = strtod(parse, &next);
    if (errno != 0 || next == parse || *next != '\0' ||
        !isfinite(minimum) ||
        !isfinite(maximum) ||
        maximum < minimum) {
        return -1;
    }
    *value = minimum + ((maximum - minimum) * deterministic_u01(random_seed, stream));
    *consumed = (size_t)(end - cursor) + 1u;
    return 1;
}

/** @brief 解析 `${halton_uniform:base:min:max}` 低差异均匀采样占位符。 */
static int parse_halton_uniform_placeholder(
    const char *cursor,
    size_t sample_index,
    size_t *consumed,
    double *value)
{
    const char prefix[] = "${halton_uniform:";
    const char *end;
    char token[128];
    char *parse;
    char *next;
    unsigned long base;
    double minimum;
    double maximum;
    size_t token_size;

    if (cursor == 0 || consumed == 0 || value == 0 ||
        strncmp(cursor, prefix, sizeof(prefix) - 1u) != 0) {
        return 0;
    }
    end = strchr(cursor, '}');
    if (end == 0) {
        return -1;
    }
    token_size = (size_t)(end - (cursor + sizeof(prefix) - 1u));
    if (token_size == 0u || token_size >= sizeof(token)) {
        return -1;
    }
    (void)memcpy(token, cursor + sizeof(prefix) - 1u, token_size);
    token[token_size] = '\0';
    parse = token;
    errno = 0;
    base = strtoul(parse, &next, 10);
    if (errno != 0 || next == parse || *next != ':' || base < 2ul || base > 97ul) {
        return -1;
    }
    parse = next + 1;
    errno = 0;
    minimum = strtod(parse, &next);
    if (errno != 0 || next == parse || *next != ':') {
        return -1;
    }
    parse = next + 1;
    errno = 0;
    maximum = strtod(parse, &next);
    if (errno != 0 || next == parse || *next != '\0' ||
        !isfinite(minimum) ||
        !isfinite(maximum) ||
        maximum < minimum) {
        return -1;
    }
    *value = minimum + ((maximum - minimum) *
        halton_radical_inverse(sample_index + 1u, (unsigned int)base));
    *consumed = (size_t)(end - cursor) + 1u;
    return 1;
}

/** @brief 解析 `${lhs_uniform:stream:min:max}` 分层均匀采样占位符。 */
static int parse_lhs_uniform_placeholder(
    const char *cursor,
    uint64_t random_seed,
    size_t sample_index,
    size_t sample_count,
    size_t *consumed,
    double *value)
{
    const char prefix[] = "${lhs_uniform:";
    const char *end;
    char token[128];
    char *parse;
    char *next;
    uint64_t stream;
    double minimum;
    double maximum;
    size_t token_size;
    size_t stride;
    size_t offset;
    size_t stratum;
    double jitter;

    if (cursor == 0 || consumed == 0 || value == 0 ||
        strncmp(cursor, prefix, sizeof(prefix) - 1u) != 0) {
        return 0;
    }
    if (sample_count == 0u || sample_index >= sample_count) {
        return -1;
    }
    end = strchr(cursor, '}');
    if (end == 0) {
        return -1;
    }
    token_size = (size_t)(end - (cursor + sizeof(prefix) - 1u));
    if (token_size == 0u || token_size >= sizeof(token)) {
        return -1;
    }
    (void)memcpy(token, cursor + sizeof(prefix) - 1u, token_size);
    token[token_size] = '\0';
    parse = token;
    errno = 0;
    stream = strtoull(parse, &next, 10);
    if (errno != 0 || next == parse || *next != ':') {
        return -1;
    }
    parse = next + 1;
    errno = 0;
    minimum = strtod(parse, &next);
    if (errno != 0 || next == parse || *next != ':') {
        return -1;
    }
    parse = next + 1;
    errno = 0;
    maximum = strtod(parse, &next);
    if (errno != 0 || next == parse || *next != '\0' ||
        !isfinite(minimum) ||
        !isfinite(maximum) ||
        maximum < minimum) {
        return -1;
    }
    stride = sample_count == 1u ?
        1u :
        1u + (size_t)(deterministic_u01(random_seed, stream ^ UINT64_C(0x51ed5eed)) *
            (double)(sample_count - 1u));
    if (stride == 0u) {
        stride = 1u;
    }
    while (gcd_size(stride, sample_count) != 1u) {
        ++stride;
        if (stride >= sample_count) {
            stride = 1u;
        }
    }
    offset = (size_t)(deterministic_u01(random_seed, stream ^ UINT64_C(0x0ff5e7)) *
        (double)sample_count);
    if (offset >= sample_count) {
        offset = sample_count - 1u;
    }
    stratum = ((sample_index * stride) + offset) % sample_count;
    jitter = deterministic_u01(random_seed, stream ^ ((uint64_t)sample_index * UINT64_C(0x100000001b3)));
    *value = minimum + ((maximum - minimum) *
        (((double)stratum + jitter) / (double)sample_count));
    *consumed = (size_t)(end - cursor) + 1u;
    return 1;
}

/** @brief 解析 `${normal:stream:mean:stddev}` Monte Carlo 占位符。 */
static int parse_normal_placeholder(
    const char *cursor,
    uint64_t random_seed,
    size_t *consumed,
    double *value)
{
    const char prefix[] = "${normal:";
    const char *end;
    char token[128];
    char *parse;
    char *next;
    uint64_t stream;
    double mean;
    double stddev;
    size_t token_size;

    if (cursor == 0 || consumed == 0 || value == 0 ||
        strncmp(cursor, prefix, sizeof(prefix) - 1u) != 0) {
        return 0;
    }
    end = strchr(cursor, '}');
    if (end == 0) {
        return -1;
    }
    token_size = (size_t)(end - (cursor + sizeof(prefix) - 1u));
    if (token_size == 0u || token_size >= sizeof(token)) {
        return -1;
    }
    (void)memcpy(token, cursor + sizeof(prefix) - 1u, token_size);
    token[token_size] = '\0';
    parse = token;
    errno = 0;
    stream = strtoull(parse, &next, 10);
    if (errno != 0 || next == parse || *next != ':') {
        return -1;
    }
    parse = next + 1;
    errno = 0;
    mean = strtod(parse, &next);
    if (errno != 0 || next == parse || *next != ':') {
        return -1;
    }
    parse = next + 1;
    errno = 0;
    stddev = strtod(parse, &next);
    if (errno != 0 || next == parse || *next != '\0' ||
        !isfinite(mean) ||
        !isfinite(stddev) ||
        stddev < 0.0) {
        return -1;
    }
    *value = mean + (stddev * deterministic_standard_normal(random_seed, stream));
    *consumed = (size_t)(end - cursor) + 1u;
    return 1;
}

/** @brief 解析 `${lognormal:stream:mu:sigma}` Monte Carlo 占位符。 */
static int parse_lognormal_placeholder(
    const char *cursor,
    uint64_t random_seed,
    size_t *consumed,
    double *value)
{
    const char prefix[] = "${lognormal:";
    const char *end;
    char token[128];
    char *parse;
    char *next;
    uint64_t stream;
    double mu;
    double sigma;
    double exponent;
    size_t token_size;

    if (cursor == 0 || consumed == 0 || value == 0 ||
        strncmp(cursor, prefix, sizeof(prefix) - 1u) != 0) {
        return 0;
    }
    end = strchr(cursor, '}');
    if (end == 0) {
        return -1;
    }
    token_size = (size_t)(end - (cursor + sizeof(prefix) - 1u));
    if (token_size == 0u || token_size >= sizeof(token)) {
        return -1;
    }
    (void)memcpy(token, cursor + sizeof(prefix) - 1u, token_size);
    token[token_size] = '\0';
    parse = token;
    errno = 0;
    stream = strtoull(parse, &next, 10);
    if (errno != 0 || next == parse || *next != ':') {
        return -1;
    }
    parse = next + 1;
    errno = 0;
    mu = strtod(parse, &next);
    if (errno != 0 || next == parse || *next != ':') {
        return -1;
    }
    parse = next + 1;
    errno = 0;
    sigma = strtod(parse, &next);
    if (errno != 0 || next == parse || *next != '\0' ||
        !isfinite(mu) ||
        !isfinite(sigma) ||
        sigma < 0.0) {
        return -1;
    }
    exponent = mu + (sigma * deterministic_standard_normal(random_seed, stream));
    if (exponent > 700.0 || exponent < -745.0) {
        return -1;
    }
    *value = exp(exponent);
    *consumed = (size_t)(end - cursor) + 1u;
    return 1;
}

/** @brief 解析 `${truncated_normal:stream:mean:stddev:min:max}` 占位符。 */
static int parse_truncated_normal_placeholder(
    const char *cursor,
    uint64_t random_seed,
    size_t *consumed,
    double *value)
{
    const char prefix[] = "${truncated_normal:";
    const char *end;
    char token[160];
    char *parse;
    char *next;
    uint64_t stream;
    double mean;
    double stddev;
    double minimum;
    double maximum;
    size_t token_size;
    unsigned int attempt;

    if (cursor == 0 || consumed == 0 || value == 0 ||
        strncmp(cursor, prefix, sizeof(prefix) - 1u) != 0) {
        return 0;
    }
    end = strchr(cursor, '}');
    if (end == 0) {
        return -1;
    }
    token_size = (size_t)(end - (cursor + sizeof(prefix) - 1u));
    if (token_size == 0u || token_size >= sizeof(token)) {
        return -1;
    }
    (void)memcpy(token, cursor + sizeof(prefix) - 1u, token_size);
    token[token_size] = '\0';
    parse = token;
    errno = 0;
    stream = strtoull(parse, &next, 10);
    if (errno != 0 || next == parse || *next != ':') {
        return -1;
    }
    parse = next + 1;
    errno = 0;
    mean = strtod(parse, &next);
    if (errno != 0 || next == parse || *next != ':') {
        return -1;
    }
    parse = next + 1;
    errno = 0;
    stddev = strtod(parse, &next);
    if (errno != 0 || next == parse || *next != ':') {
        return -1;
    }
    parse = next + 1;
    errno = 0;
    minimum = strtod(parse, &next);
    if (errno != 0 || next == parse || *next != ':') {
        return -1;
    }
    parse = next + 1;
    errno = 0;
    maximum = strtod(parse, &next);
    if (errno != 0 || next == parse || *next != '\0' ||
        !isfinite(mean) ||
        !isfinite(stddev) ||
        !isfinite(minimum) ||
        !isfinite(maximum) ||
        stddev < 0.0 ||
        maximum < minimum) {
        return -1;
    }
    for (attempt = 0u; attempt < 16u; ++attempt) {
        const double sample =
            mean + (stddev * deterministic_standard_normal(random_seed, stream + attempt));

        if (sample >= minimum && sample <= maximum) {
            *value = sample;
            *consumed = (size_t)(end - cursor) + 1u;
            return 1;
        }
    }
    *value = mean;
    if (*value < minimum) {
        *value = minimum;
    } else if (*value > maximum) {
        *value = maximum;
    }
    *consumed = (size_t)(end - cursor) + 1u;
    return 1;
}

/** @brief 解析 `${choice:stream:option|option}` 离散选择占位符。 */
static int parse_choice_placeholder(
    const char *cursor,
    uint64_t random_seed,
    size_t *consumed,
    char *value,
    size_t value_size)
{
    const char prefix[] = "${choice:";
    const char *end;
    char token[256];
    char *parse;
    char *next;
    char *options;
    char *scan;
    char *selected;
    uint64_t stream;
    size_t token_size;
    size_t option_count = 1u;
    size_t selected_index;
    size_t option_size;

    if (cursor == 0 || consumed == 0 || value == 0 || value_size == 0u ||
        strncmp(cursor, prefix, sizeof(prefix) - 1u) != 0) {
        return 0;
    }
    end = strchr(cursor, '}');
    if (end == 0) {
        return -1;
    }
    token_size = (size_t)(end - (cursor + sizeof(prefix) - 1u));
    if (token_size == 0u || token_size >= sizeof(token)) {
        return -1;
    }
    (void)memcpy(token, cursor + sizeof(prefix) - 1u, token_size);
    token[token_size] = '\0';
    parse = token;
    errno = 0;
    stream = strtoull(parse, &next, 10);
    if (errno != 0 || next == parse || *next != ':') {
        return -1;
    }
    options = next + 1;
    if (*options == '\0' || *options == '|') {
        return -1;
    }
    for (scan = options; *scan != '\0'; ++scan) {
        if ((unsigned char)*scan < 0x20u) {
            return -1;
        }
        if (*scan == '|') {
            if (scan[1] == '\0' || scan[1] == '|') {
                return -1;
            }
            ++option_count;
        }
    }
    selected_index = (size_t)(deterministic_u01(random_seed, stream) * (double)option_count);
    if (selected_index >= option_count) {
        selected_index = option_count - 1u;
    }
    selected = options;
    while (selected_index > 0u) {
        selected = strchr(selected, '|');
        if (selected == 0) {
            return -1;
        }
        ++selected;
        --selected_index;
    }
    scan = strchr(selected, '|');
    option_size = scan == 0 ? strlen(selected) : (size_t)(scan - selected);
    if (option_size == 0u || option_size >= value_size) {
        return -1;
    }
    (void)memcpy(value, selected, option_size);
    value[option_size] = '\0';
    *consumed = (size_t)(end - cursor) + 1u;
    return 1;
}

/** @brief 解析 `${correlated_normal:stream:base_stream:mean:stddev:rho}` 占位符。 */
static int parse_correlated_normal_placeholder(
    const char *cursor,
    uint64_t random_seed,
    size_t *consumed,
    double *value)
{
    const char prefix[] = "${correlated_normal:";
    const char *end;
    char token[160];
    char *parse;
    char *next;
    uint64_t stream;
    uint64_t base_stream;
    double mean;
    double stddev;
    double rho;
    double independent_scale;
    size_t token_size;

    if (cursor == 0 || consumed == 0 || value == 0 ||
        strncmp(cursor, prefix, sizeof(prefix) - 1u) != 0) {
        return 0;
    }
    end = strchr(cursor, '}');
    if (end == 0) {
        return -1;
    }
    token_size = (size_t)(end - (cursor + sizeof(prefix) - 1u));
    if (token_size == 0u || token_size >= sizeof(token)) {
        return -1;
    }
    (void)memcpy(token, cursor + sizeof(prefix) - 1u, token_size);
    token[token_size] = '\0';
    parse = token;
    errno = 0;
    stream = strtoull(parse, &next, 10);
    if (errno != 0 || next == parse || *next != ':') {
        return -1;
    }
    parse = next + 1;
    errno = 0;
    base_stream = strtoull(parse, &next, 10);
    if (errno != 0 || next == parse || *next != ':') {
        return -1;
    }
    parse = next + 1;
    errno = 0;
    mean = strtod(parse, &next);
    if (errno != 0 || next == parse || *next != ':') {
        return -1;
    }
    parse = next + 1;
    errno = 0;
    stddev = strtod(parse, &next);
    if (errno != 0 || next == parse || *next != ':') {
        return -1;
    }
    parse = next + 1;
    errno = 0;
    rho = strtod(parse, &next);
    if (errno != 0 || next == parse || *next != '\0' ||
        !isfinite(mean) ||
        !isfinite(stddev) ||
        !isfinite(rho) ||
        stddev < 0.0 ||
        rho < -1.0 ||
        rho > 1.0) {
        return -1;
    }
    independent_scale = sqrt(1.0 - (rho * rho));
    *value = mean + (stddev *
        ((rho * deterministic_standard_normal(random_seed, base_stream)) +
            (independent_scale * deterministic_standard_normal(random_seed, stream))));
    *consumed = (size_t)(end - cursor) + 1u;
    return 1;
}

/** @brief 写入带 Monte Carlo 占位符替换的 runtime。 */
static SimStatus write_template_instance(
    FILE *file,
    const char *template_text,
    size_t sample_index,
    size_t sample_count,
    uint64_t random_seed,
    const char *sample_output_dir)
{
    const char *cursor = template_text;

    if (file == 0 || template_text == 0 || sample_output_dir == 0) {
        return SIM_ERR_INVALID_ARG;
    }
    while (*cursor != '\0') {
        size_t consumed = 0u;
        double uniform_value = 0.0;
        double halton_uniform_value = 0.0;
        double lhs_uniform_value = 0.0;
        double normal_value = 0.0;
        double lognormal_value = 0.0;
        double truncated_normal_value = 0.0;
        double correlated_normal_value = 0.0;
        char choice_value[128];
        const int uniform_status =
            parse_uniform_placeholder(cursor, random_seed, &consumed, &uniform_value);
        const int lhs_uniform_status =
            parse_lhs_uniform_placeholder(
                cursor,
                random_seed,
                sample_index,
                sample_count,
                &consumed,
                &lhs_uniform_value);
        const int halton_uniform_status =
            parse_halton_uniform_placeholder(
                cursor,
                sample_index,
                &consumed,
                &halton_uniform_value);
        const int normal_status =
            parse_normal_placeholder(cursor, random_seed, &consumed, &normal_value);
        const int lognormal_status =
            parse_lognormal_placeholder(cursor, random_seed, &consumed, &lognormal_value);
        const int truncated_normal_status =
            parse_truncated_normal_placeholder(
                cursor,
                random_seed,
                &consumed,
                &truncated_normal_value);
        const int choice_status =
            parse_choice_placeholder(
                cursor,
                random_seed,
                &consumed,
                choice_value,
                sizeof(choice_value));
        const int correlated_normal_status =
            parse_correlated_normal_placeholder(
                cursor,
                random_seed,
                &consumed,
                &correlated_normal_value);

        if (uniform_status < 0 ||
            halton_uniform_status < 0 ||
            lhs_uniform_status < 0 ||
            normal_status < 0 ||
            lognormal_status < 0 ||
            truncated_normal_status < 0 ||
            choice_status < 0 ||
            correlated_normal_status < 0) {
            return SIM_ERR_CONFIG;
        }
        if (uniform_status > 0) {
            (void)fprintf(file, "%.17g", uniform_value);
            cursor += consumed;
            continue;
        }
        if (lhs_uniform_status > 0) {
            (void)fprintf(file, "%.17g", lhs_uniform_value);
            cursor += consumed;
            continue;
        }
        if (halton_uniform_status > 0) {
            (void)fprintf(file, "%.17g", halton_uniform_value);
            cursor += consumed;
            continue;
        }
        if (normal_status > 0) {
            (void)fprintf(file, "%.17g", normal_value);
            cursor += consumed;
            continue;
        }
        if (lognormal_status > 0) {
            (void)fprintf(file, "%.17g", lognormal_value);
            cursor += consumed;
            continue;
        }
        if (truncated_normal_status > 0) {
            (void)fprintf(file, "%.17g", truncated_normal_value);
            cursor += consumed;
            continue;
        }
        if (choice_status > 0) {
            (void)fprintf(file, "%s", choice_value);
            cursor += consumed;
            continue;
        }
        if (correlated_normal_status > 0) {
            (void)fprintf(file, "%.17g", correlated_normal_value);
            cursor += consumed;
            continue;
        }
        if (strncmp(cursor, "${sample_index}", 15u) == 0) {
            (void)fprintf(file, "%zu", sample_index);
            cursor += 15u;
        } else if (strncmp(cursor, "${random_seed}", 14u) == 0) {
            (void)fprintf(file, "%llu", (unsigned long long)random_seed);
            cursor += 14u;
        } else if (strncmp(cursor, "${sample_output_dir}", 20u) == 0) {
            (void)fprintf(file, "%s", sample_output_dir);
            cursor += 20u;
        } else {
            if (fputc((unsigned char)*cursor, file) == EOF) {
                return SIM_ERR_IO;
            }
            ++cursor;
        }
    }
    return ferror(file) == 0 ? SIM_OK : SIM_ERR_IO;
}

/** @brief 从 runtime 模板生成确定性 Monte Carlo 运行清单。 */
static SimStatus generate_monte_carlo_manifest(const BatchRunnerArgs *args)
{
    char *template_text = 0;
    FILE *manifest;
    size_t index;
    SimStatus status;

    if (args == 0) {
        return SIM_ERR_INVALID_ARG;
    }
    if (mkdir(args->runtime_output_dir, 0777) != 0 && errno != EEXIST) {
        return SIM_ERR_IO;
    }
    status = read_template_file(args->runtime_template_path, &template_text);
    if (status != SIM_OK) {
        return status;
    }
    manifest = fopen(args->generate_manifest_path, "wb");
    if (manifest == 0) {
        free(template_text);
        return SIM_ERR_IO;
    }
    for (index = 0u; index < args->sample_count; ++index) {
        char runtime_path[BATCH_RUNNER_MAX_PATH];
        char sample_output_dir[BATCH_RUNNER_MAX_PATH];
        char stats_path[BATCH_RUNNER_MAX_PATH];
        FILE *runtime;
        int written;
        const uint64_t random_seed = args->base_seed + (uint64_t)index;

        written = snprintf(
            runtime_path,
            sizeof(runtime_path),
            "%s/runtime_%04zu.json",
            args->runtime_output_dir,
            index);
        if (written < 0 || (size_t)written >= sizeof(runtime_path)) {
            (void)fclose(manifest);
            free(template_text);
            return SIM_ERR_OUT_OF_RANGE;
        }
        written = snprintf(
            sample_output_dir,
            sizeof(sample_output_dir),
            "%s/sample_%04zu",
            args->runtime_output_dir,
            index);
        if (written < 0 || (size_t)written >= sizeof(sample_output_dir)) {
            (void)fclose(manifest);
            free(template_text);
            return SIM_ERR_OUT_OF_RANGE;
        }
        if (mkdir(sample_output_dir, 0777) != 0 && errno != EEXIST) {
            (void)fclose(manifest);
            free(template_text);
            return SIM_ERR_IO;
        }
        runtime = fopen(runtime_path, "wb");
        if (runtime == 0) {
            (void)fclose(manifest);
            free(template_text);
            return SIM_ERR_IO;
        }
        status = write_template_instance(
            runtime,
            template_text,
            index,
            args->sample_count,
            random_seed,
            sample_output_dir);
        if (fclose(runtime) != 0 && status == SIM_OK) {
            status = SIM_ERR_IO;
        }
        if (status != SIM_OK) {
            (void)fclose(manifest);
            free(template_text);
            return status;
        }
        written = snprintf(stats_path, sizeof(stats_path), "%s/campaign_summary.json", sample_output_dir);
        if (written < 0 || (size_t)written >= sizeof(stats_path) ||
            fprintf(manifest, "%s %s\n", runtime_path, stats_path) < 0) {
            (void)fclose(manifest);
            free(template_text);
            return SIM_ERR_IO;
        }
    }
    free(template_text);
    if (fclose(manifest) != 0) {
        return SIM_ERR_IO;
    }
    return SIM_OK;
}

/** @brief 从批次清单读取 runtime 和可选统计输入路径。 */
static SimStatus load_manifest(
    const char *path,
    BatchRun *runs,
    size_t run_capacity,
    size_t *run_count)
{
    FILE *file;
    char line[BATCH_RUNNER_MAX_LINE];
    size_t count = 0u;

    if (path == 0 || runs == 0 || run_count == 0 || run_capacity == 0u) {
        return SIM_ERR_INVALID_ARG;
    }
    file = fopen(path, "r");
    if (file == 0) {
        return SIM_ERR_IO;
    }
    while (fgets(line, sizeof(line), file) != 0) {
        char *trimmed = trim_line(line);
        char *runtime;
        char *stats_input;
        char *extra;

        if (trimmed[0] == '\0' || trimmed[0] == '#') {
            continue;
        }
        if (count >= run_capacity) {
            (void)fclose(file);
            return SIM_ERR_OUT_OF_RANGE;
        }
        runtime = strtok(trimmed, " \t\r\n");
        stats_input = strtok(0, " \t\r\n");
        extra = strtok(0, " \t\r\n");
        if (runtime == 0 || extra != 0 ||
            strlen(runtime) >= BATCH_RUNNER_MAX_PATH ||
            (stats_input != 0 && strlen(stats_input) >= BATCH_RUNNER_MAX_PATH)) {
            (void)fclose(file);
            return SIM_ERR_CONFIG;
        }
        (void)memset(&runs[count], 0, sizeof(runs[count]));
        (void)strncpy(runs[count].runtime_path, runtime, BATCH_RUNNER_MAX_PATH - 1u);
        if (stats_input != 0) {
            (void)strncpy(
                runs[count].stats_input_path,
                stats_input,
                BATCH_RUNNER_MAX_PATH - 1u);
            runs[count].has_stats_input = 1;
        }
        ++count;
    }
    if (ferror(file) != 0) {
        (void)fclose(file);
        return SIM_ERR_IO;
    }
    if (fclose(file) != 0) {
        return SIM_ERR_IO;
    }
    *run_count = count;
    return count > 0u ? SIM_OK : SIM_ERR_CONFIG;
}

/** @brief 执行 instance_manager。 */
static int run_instance_manager(const char *program, const char *runtime_path)
{
    pid_t pid = fork();
    int status = 0;

    if (pid < 0) {
        return 1;
    }
    if (pid == 0) {
        execl(program, program, "--runtime", runtime_path, (char *)0);
        _exit(127);
    }
    if (waitpid(pid, &status, 0) < 0) {
        return 1;
    }
    return WIFEXITED(status) && WEXITSTATUS(status) == 0 ? 0 : 1;
}

/** @brief 执行 batch_stats 聚合可用的统计输入。 */
static int run_batch_stats(
    const char *program,
    const char *output_path,
    const BatchRun *runs,
    size_t run_count)
{
    char **argv;
    size_t arg_count = 1u;
    size_t index;
    size_t out_index;
    pid_t pid;
    int status = 0;

    for (index = 0u; index < run_count; ++index) {
        if (runs[index].has_stats_input != 0) {
            arg_count += 2u;
        }
    }
    if (output_path != 0) {
        arg_count += 2u;
    }
    argv = (char **)calloc(arg_count + 1u, sizeof(*argv));
    if (argv == 0) {
        return 1;
    }
    out_index = 0u;
    argv[out_index++] = (char *)program;
    for (index = 0u; index < run_count; ++index) {
        if (runs[index].has_stats_input != 0) {
            argv[out_index++] = (char *)"--input";
            argv[out_index++] = (char *)runs[index].stats_input_path;
        }
    }
    if (output_path != 0) {
        argv[out_index++] = (char *)"--output";
        argv[out_index++] = (char *)output_path;
    }
    argv[out_index] = 0;
    pid = fork();
    if (pid < 0) {
        free(argv);
        return 1;
    }
    if (pid == 0) {
        execv(program, argv);
        _exit(127);
    }
    free(argv);
    if (waitpid(pid, &status, 0) < 0) {
        return 1;
    }
    return WIFEXITED(status) && WEXITSTATUS(status) == 0 ? 0 : 1;
}

/** @brief 写出 Monte Carlo 工作流级运行清单。 */
static SimStatus write_batch_run_manifest(
    const BatchRunnerArgs *args,
    size_t run_count,
    size_t attempted_count,
    size_t failure_count,
    SimStatus workflow_status)
{
    const char *list_path;
    uint32_t list_crc = 0u;
    uint32_t template_crc = 0u;
    uint64_t list_size = 0u;
    uint64_t template_size = 0u;
    char wall_clock[32];
    FILE *file;
    SimStatus status;
    int close_status;

    if (args == 0 || args->run_manifest_path == 0) {
        return SIM_ERR_INVALID_ARG;
    }
    list_path = args->generate_manifest_path != 0 ?
        args->generate_manifest_path : args->manifest_path;
    status = provenance_file_crc32(list_path, &list_crc, &list_size);
    if (status != SIM_OK) {
        return status;
    }
    if (args->runtime_template_path != 0) {
        status = provenance_file_crc32(
            args->runtime_template_path,
            &template_crc,
            &template_size);
        if (status != SIM_OK) {
            return status;
        }
    }
    provenance_format_wall_clock_utc(wall_clock, sizeof(wall_clock));
    file = fopen(args->run_manifest_path, "wb");
    if (file == 0) {
        return SIM_ERR_IO;
    }
    (void)fprintf(file, "{\n");
    (void)fprintf(file, "  \"schema_version\": 1,\n");
    (void)fprintf(file, "  \"run_mode\": \"MONTE_CARLO\",\n");
    (void)fprintf(file, "  \"status\": ");
    (void)provenance_write_json_string(file, sim_status_to_string(workflow_status));
    (void)fprintf(file, ",\n");
    (void)fprintf(
        file,
        "  \"program_version\": \"batch_runner %d.%d.%d\",\n",
        MISSILE_SIM_VERSION_MAJOR,
        MISSILE_SIM_VERSION_MINOR,
        MISSILE_SIM_VERSION_PATCH);
    (void)fprintf(file, "  \"git_commit\": ");
    (void)provenance_write_json_string(file, MISSILE_SIM_GIT_COMMIT);
    (void)fprintf(
        file,
        ",\n  \"git_worktree_dirty\": %s,\n",
        MISSILE_SIM_GIT_DIRTY != 0 ? "true" : "false");
    (void)fprintf(file, "  \"build_time\": ");
    (void)provenance_write_json_string(file, MISSILE_SIM_BUILD_TIME);
    (void)fprintf(file, ",\n  \"compiler\": ");
    (void)provenance_write_json_string(file, MISSILE_SIM_COMPILER);
    (void)fprintf(file, ",\n  \"start_time_wall_clock\": ");
    (void)provenance_write_json_string(file, wall_clock);
    (void)fprintf(
        file,
        ",\n  \"protocol_version\": \"%d.%d\",\n",
        MISSILE_SIM_PROTOCOL_VERSION_MAJOR,
        MISSILE_SIM_PROTOCOL_VERSION_MINOR);
    (void)fprintf(file, "  \"configuration_provenance\": ");
    (void)provenance_write_json_string(file, "each generated runtime and child run_manifest");
    (void)fprintf(file, ",\n  \"random_seed\": ");
    if (args->runtime_template_path != 0) {
        (void)fprintf(file, "%llu,\n", (unsigned long long)args->base_seed);
        (void)fprintf(file, "  \"random_seed_policy\": \"base_seed_plus_sample_index\",\n");
    } else {
        (void)fprintf(file, "null,\n");
        (void)fprintf(file, "  \"random_seed_policy\": \"declared_by_input_runtime_files\",\n");
    }
    (void)fprintf(file, "  \"inputs\": {\n    \"run_list\": { \"path\": ");
    (void)provenance_write_json_string(file, list_path);
    (void)fprintf(
        file,
        ", \"crc32\": \"0x%08x\", \"size_bytes\": %llu },\n",
        list_crc,
        (unsigned long long)list_size);
    (void)fprintf(file, "    \"runtime_template\": ");
    if (args->runtime_template_path != 0) {
        (void)fprintf(file, "{ \"path\": ");
        (void)provenance_write_json_string(file, args->runtime_template_path);
        (void)fprintf(
            file,
            ", \"crc32\": \"0x%08x\", \"size_bytes\": %llu }\n",
            template_crc,
            (unsigned long long)template_size);
    } else {
        (void)fprintf(file, "null\n");
    }
    (void)fprintf(file, "  },\n");
    (void)fprintf(file, "  \"outputs\": {\n    \"runtime_output_dir\": ");
    if (args->runtime_output_dir != 0) {
        (void)provenance_write_json_string(file, args->runtime_output_dir);
    } else {
        (void)fprintf(file, "null");
    }
    (void)fprintf(file, ",\n    \"batch_statistics\": ");
    if (args->output_path != 0) {
        (void)provenance_write_json_string(file, args->output_path);
    } else {
        (void)fprintf(file, "null");
    }
    (void)fprintf(file, "\n  },\n");
    (void)fprintf(file, "  \"requested_sample_count\": %zu,\n", args->sample_count);
    (void)fprintf(file, "  \"run_count\": %zu,\n", run_count);
    (void)fprintf(file, "  \"attempted_count\": %zu,\n", attempted_count);
    (void)fprintf(
        file,
        "  \"completed_count\": %zu,\n",
        attempted_count >= failure_count ? attempted_count - failure_count : 0u);
    (void)fprintf(file, "  \"failed_count\": %zu,\n", failure_count);
    (void)fprintf(
        file,
        "  \"generated_only\": %s\n",
        args->instance_manager_path == 0 ? "true" : "false");
    (void)fprintf(file, "}\n");
    status = ferror(file) == 0 ? SIM_OK : SIM_ERR_IO;
    close_status = fclose(file);
    return status == SIM_OK && close_status == 0 ? SIM_OK : SIM_ERR_IO;
}

int main(int argc, char **argv)
{
    BatchRunnerArgs args;
    BatchRun runs[BATCH_RUNNER_MAX_RUNS];
    size_t run_count = 0u;
    size_t index;
    size_t failure_count = 0u;
    size_t stats_input_count = 0u;
    size_t attempted_count = 0u;
    SimStatus status;
    SimStatus manifest_status;

    if (!parse_args(argc, argv, &args)) {
        print_usage(argv[0]);
        return 2;
    }
    if (args.generate_manifest_path != 0) {
        status = generate_monte_carlo_manifest(&args);
        if (status != SIM_OK) {
            (void)fprintf(stderr, "failed to generate manifest: %s\n", sim_status_to_string(status));
            return 1;
        }
        if (args.instance_manager_path == 0) {
            status = load_manifest(
                args.generate_manifest_path,
                runs,
                BATCH_RUNNER_MAX_RUNS,
                &run_count);
            if (status == SIM_OK) {
                status = write_batch_run_manifest(&args, run_count, 0u, 0u, SIM_OK);
            }
            if (status != SIM_OK) {
                (void)fprintf(stderr, "failed to write run manifest: %s\n", sim_status_to_string(status));
                return 1;
            }
            return 0;
        }
        args.manifest_path = args.generate_manifest_path;
    }
    status = load_manifest(args.manifest_path, runs, BATCH_RUNNER_MAX_RUNS, &run_count);
    if (status != SIM_OK) {
        (void)fprintf(stderr, "failed to load manifest: %s\n", sim_status_to_string(status));
        return 1;
    }
    for (index = 0u; index < run_count; ++index) {
        ++attempted_count;
        if (run_instance_manager(args.instance_manager_path, runs[index].runtime_path) != 0) {
            ++failure_count;
            (void)fprintf(stderr, "instance_manager failed: %s\n", runs[index].runtime_path);
            if (args.stop_on_failure != 0) {
                break;
            }
        }
        if (runs[index].has_stats_input != 0) {
            ++stats_input_count;
        }
    }
    if (args.batch_stats_path != 0 && stats_input_count > 0u && failure_count == 0u) {
        if (run_batch_stats(args.batch_stats_path, args.output_path, runs, run_count) != 0) {
            (void)fprintf(stderr, "batch_stats failed\n");
            manifest_status = write_batch_run_manifest(
                &args,
                run_count,
                attempted_count,
                failure_count,
                SIM_ERR_INTERNAL);
            if (manifest_status != SIM_OK) {
                (void)fprintf(stderr, "failed to write run manifest: %s\n", sim_status_to_string(manifest_status));
            }
            return 1;
        }
    }
    manifest_status = write_batch_run_manifest(
        &args,
        run_count,
        attempted_count,
        failure_count,
        failure_count == 0u ? SIM_OK : SIM_ERR_INTERNAL);
    if (manifest_status != SIM_OK) {
        (void)fprintf(stderr, "failed to write run manifest: %s\n", sim_status_to_string(manifest_status));
        return 1;
    }
    return failure_count == 0u ? 0 : 1;
}
