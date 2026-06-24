/** @file config.c
 *  @brief 轻量 JSON 配置读取实现。
 *
 *  该实现不依赖外部 JSON 库，面向工程里稳定的配置结构提供足够的字段
 *  提取能力，主要服务于运行时启动路径。
 */
#include "common/config.h"

#include <ctype.h>
#include <errno.h>
#include <limits.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CONFIG_PATH_MAX 192
#define CONFIG_JSON_MAX_DEPTH 64u

typedef struct JsonCursor {
    /** @brief 当前待解析字符。 */
    const char *current;
    /** @brief 输入缓冲区末尾后一字节。 */
    const char *end;
} JsonCursor;

/** @brief 跳过空白字符。 */
static const char *skip_ws(const char *p, const char *end)
{
    while (p < end && isspace((unsigned char)*p)) {
        ++p;
    }
    return p;
}

/** @brief 判断字符是否属于 JSON Unicode 转义允许的十六进制集合。 */
static int is_json_hex(char ch)
{
    return (ch >= '0' && ch <= '9') ||
        (ch >= 'a' && ch <= 'f') ||
        (ch >= 'A' && ch <= 'F');
}

/** @brief 递归解析一个 JSON 值，depth 用于限制恶意深层嵌套。 */
static SimStatus parse_json_value(JsonCursor *cursor, unsigned int depth);

/** @brief 解析字符串并校验控制字符和反斜杠转义。 */
static SimStatus parse_json_string(JsonCursor *cursor)
{
    const char *p;

    if (cursor == 0 || cursor->current >= cursor->end || *cursor->current != '"') {
        return SIM_ERR_CONFIG;
    }
    p = cursor->current + 1;
    while (p < cursor->end) {
        unsigned char ch = (unsigned char)*p++;

        if (ch == '"') {
            cursor->current = p;
            return SIM_OK;
        }
        if (ch < 0x20u) {
            return SIM_ERR_CONFIG;
        }
        if (ch == '\\') {
            char escape;
            unsigned int index;

            if (p >= cursor->end) {
                return SIM_ERR_CONFIG;
            }
            escape = *p++;
            if (escape == 'u') {
                for (index = 0u; index < 4u; ++index) {
                    if (p >= cursor->end || !is_json_hex(*p)) {
                        return SIM_ERR_CONFIG;
                    }
                    ++p;
                }
            } else if (escape != '"' && escape != '\\' && escape != '/' &&
                escape != 'b' && escape != 'f' && escape != 'n' &&
                escape != 'r' && escape != 't') {
                return SIM_ERR_CONFIG;
            }
        }
    }
    return SIM_ERR_CONFIG;
}

/** @brief 严格解析 JSON 数字，不接受 NaN、Inf 和非法前导零。 */
static SimStatus parse_json_number(JsonCursor *cursor)
{
    const char *p = cursor->current;

    if (p < cursor->end && *p == '-') {
        ++p;
    }
    if (p >= cursor->end) {
        return SIM_ERR_CONFIG;
    }
    if (*p == '0') {
        ++p;
        if (p < cursor->end && isdigit((unsigned char)*p)) {
            return SIM_ERR_CONFIG;
        }
    } else {
        if (!isdigit((unsigned char)*p) || *p == '0') {
            return SIM_ERR_CONFIG;
        }
        while (p < cursor->end && isdigit((unsigned char)*p)) {
            ++p;
        }
    }
    if (p < cursor->end && *p == '.') {
        ++p;
        if (p >= cursor->end || !isdigit((unsigned char)*p)) {
            return SIM_ERR_CONFIG;
        }
        while (p < cursor->end && isdigit((unsigned char)*p)) {
            ++p;
        }
    }
    if (p < cursor->end && (*p == 'e' || *p == 'E')) {
        ++p;
        if (p < cursor->end && (*p == '+' || *p == '-')) {
            ++p;
        }
        if (p >= cursor->end || !isdigit((unsigned char)*p)) {
            return SIM_ERR_CONFIG;
        }
        while (p < cursor->end && isdigit((unsigned char)*p)) {
            ++p;
        }
    }
    cursor->current = p;
    return SIM_OK;
}

/** @brief 递归解析 JSON 数组及其逗号分隔规则。 */
static SimStatus parse_json_array(JsonCursor *cursor, unsigned int depth)
{
    SimStatus status;

    ++cursor->current;
    cursor->current = skip_ws(cursor->current, cursor->end);
    if (cursor->current < cursor->end && *cursor->current == ']') {
        ++cursor->current;
        return SIM_OK;
    }

    for (;;) {
        status = parse_json_value(cursor, depth + 1u);
        if (status != SIM_OK) {
            return status;
        }
        cursor->current = skip_ws(cursor->current, cursor->end);
        if (cursor->current >= cursor->end) {
            return SIM_ERR_CONFIG;
        }
        if (*cursor->current == ']') {
            ++cursor->current;
            return SIM_OK;
        }
        if (*cursor->current != ',') {
            return SIM_ERR_CONFIG;
        }
        ++cursor->current;
        cursor->current = skip_ws(cursor->current, cursor->end);
    }
}

/** @brief 递归解析 JSON 对象及键值分隔规则。 */
static SimStatus parse_json_object(JsonCursor *cursor, unsigned int depth)
{
    SimStatus status;

    ++cursor->current;
    cursor->current = skip_ws(cursor->current, cursor->end);
    if (cursor->current < cursor->end && *cursor->current == '}') {
        ++cursor->current;
        return SIM_OK;
    }

    for (;;) {
        status = parse_json_string(cursor);
        if (status != SIM_OK) {
            return status;
        }
        cursor->current = skip_ws(cursor->current, cursor->end);
        if (cursor->current >= cursor->end || *cursor->current != ':') {
            return SIM_ERR_CONFIG;
        }
        ++cursor->current;
        cursor->current = skip_ws(cursor->current, cursor->end);
        status = parse_json_value(cursor, depth + 1u);
        if (status != SIM_OK) {
            return status;
        }
        cursor->current = skip_ws(cursor->current, cursor->end);
        if (cursor->current >= cursor->end) {
            return SIM_ERR_CONFIG;
        }
        if (*cursor->current == '}') {
            ++cursor->current;
            return SIM_OK;
        }
        if (*cursor->current != ',') {
            return SIM_ERR_CONFIG;
        }
        ++cursor->current;
        cursor->current = skip_ws(cursor->current, cursor->end);
    }
}

/** @brief 根据首字符分派对象、数组、字符串、数字或字面量解析。 */
static SimStatus parse_json_value(JsonCursor *cursor, unsigned int depth)
{
    if (cursor == 0 || depth > CONFIG_JSON_MAX_DEPTH) {
        return SIM_ERR_CONFIG;
    }
    cursor->current = skip_ws(cursor->current, cursor->end);
    if (cursor->current >= cursor->end) {
        return SIM_ERR_CONFIG;
    }

    switch (*cursor->current) {
    case '{':
        return parse_json_object(cursor, depth);
    case '[':
        return parse_json_array(cursor, depth);
    case '"':
        return parse_json_string(cursor);
    case 't':
        if ((size_t)(cursor->end - cursor->current) >= 4u &&
            strncmp(cursor->current, "true", 4u) == 0) {
            cursor->current += 4;
            return SIM_OK;
        }
        return SIM_ERR_CONFIG;
    case 'f':
        if ((size_t)(cursor->end - cursor->current) >= 5u &&
            strncmp(cursor->current, "false", 5u) == 0) {
            cursor->current += 5;
            return SIM_OK;
        }
        return SIM_ERR_CONFIG;
    case 'n':
        if ((size_t)(cursor->end - cursor->current) >= 4u &&
            strncmp(cursor->current, "null", 4u) == 0) {
            cursor->current += 4;
            return SIM_OK;
        }
        return SIM_ERR_CONFIG;
    default:
        return parse_json_number(cursor);
    }
}

/** @brief 跳过一个 JSON 字符串字面量。 */
static const char *skip_json_string(const char *p, const char *end)
{
    if (p >= end || *p != '"') {
        return p;
    }
    ++p;
    while (p < end) {
        if (*p == '\\') {
            p += 2;
            continue;
        }
        if (*p == '"') {
            return p + 1;
        }
        ++p;
    }
    return end;
}

/** @brief 查找与当前括号匹配的结束括号。 */
static const char *find_matching_bracket(const char *open, const char *end)
{
    int depth = 0;
    char open_ch;
    char close_ch;
    const char *p = open;

    if (open >= end) {
        return 0;
    }

    open_ch = *open;
    close_ch = open_ch == '{' ? '}' : ']';

    while (p < end) {
        if (*p == '"') {
            p = skip_json_string(p, end);
            continue;
        }
        if (*p == open_ch) {
            ++depth;
        } else if (*p == close_ch) {
            --depth;
            if (depth == 0) {
                return p;
            }
        }
        ++p;
    }

    return 0;
}

/** @brief 比较 JSON 键名。 */
static int key_equals(const char *key_start, const char *key_end, const char *key)
{
    size_t key_len = strlen(key);
    size_t span_len = (size_t)(key_end - key_start);
    return key_len == span_len && strncmp(key_start, key, key_len) == 0;
}

/** @brief 解析点路径中的单段名称和可选数组下标。 */
static SimStatus parse_path_part(
    const char *part,
    char *key,
    size_t key_size,
    int *has_index,
    size_t *index)
{
    const char *bracket;
    size_t key_len;

    if (part == 0 || key == 0 || key_size == 0u ||
        has_index == 0 || index == 0 || part[0] == '\0') {
        return SIM_ERR_INVALID_ARG;
    }
    bracket = strchr(part, '[');
    if (bracket == 0) {
        key_len = strlen(part);
        if (key_len == 0u || key_len >= key_size) {
            return SIM_ERR_OUT_OF_RANGE;
        }
        (void)memcpy(key, part, key_len + 1u);
        *has_index = 0;
        *index = 0u;
        return SIM_OK;
    }
    key_len = (size_t)(bracket - part);
    if (key_len == 0u || key_len >= key_size) {
        return SIM_ERR_OUT_OF_RANGE;
    }
    if (bracket[1] == '-' || bracket[1] == '\0') {
        return SIM_ERR_CONFIG;
    }
    {
        char *endptr;
        unsigned long parsed;

        errno = 0;
        parsed = strtoul(bracket + 1, &endptr, 10);
        if (endptr == bracket + 1 || errno == ERANGE ||
            *endptr != ']' || endptr[1] != '\0') {
            return SIM_ERR_CONFIG;
        }
        (void)memcpy(key, part, key_len);
        key[key_len] = '\0';
        *has_index = 1;
        *index = (size_t)parsed;
        return SIM_OK;
    }
}

/** @brief 定位一个 JSON 值的结束位置，返回结束后一字节。 */
static const char *find_json_value_end(const char *value, const char *end)
{
    const char *p;

    if (value == 0 || value >= end) {
        return 0;
    }
    p = skip_ws(value, end);
    if (p >= end) {
        return 0;
    }
    if (*p == '{' || *p == '[') {
        const char *matched = find_matching_bracket(p, end);
        return matched == 0 ? 0 : matched + 1;
    }
    if (*p == '"') {
        return skip_json_string(p, end);
    }
    while (p < end && *p != ',' && *p != '}' && *p != ']') {
        ++p;
    }
    return p;
}

/** @brief 从 JSON 数组中定位第 index 个元素。 */
static SimStatus find_array_element(
    const char *array_start,
    const char *end,
    size_t index,
    const char **value)
{
    const char *array_end;
    const char *p;
    size_t current = 0u;

    if (array_start == 0 || value == 0 || array_start >= end || *array_start != '[') {
        return SIM_ERR_CONFIG;
    }
    array_end = find_matching_bracket(array_start, end);
    if (array_end == 0) {
        return SIM_ERR_CONFIG;
    }
    p = skip_ws(array_start + 1, array_end);
    if (p >= array_end) {
        return SIM_ERR_OUT_OF_RANGE;
    }
    for (;;) {
        const char *next;

        p = skip_ws(p, array_end);
        if (p >= array_end) {
            return SIM_ERR_OUT_OF_RANGE;
        }
        if (current == index) {
            *value = p;
            return SIM_OK;
        }
        next = find_json_value_end(p, array_end);
        if (next == 0 || next <= p) {
            return SIM_ERR_CONFIG;
        }
        p = skip_ws(next, array_end);
        if (p >= array_end) {
            return SIM_ERR_OUT_OF_RANGE;
        }
        if (*p != ',') {
            return SIM_ERR_CONFIG;
        }
        ++p;
        ++current;
    }
}

/** @brief 计算 JSON 数组元素数量。 */
static SimStatus count_array_elements(const char *array_start, const char *end, size_t *out)
{
    const char *array_end;
    const char *p;
    size_t count = 0u;

    if (array_start == 0 || out == 0 || array_start >= end || *array_start != '[') {
        return SIM_ERR_CONFIG;
    }
    array_end = find_matching_bracket(array_start, end);
    if (array_end == 0) {
        return SIM_ERR_CONFIG;
    }
    p = skip_ws(array_start + 1, array_end);
    if (p >= array_end) {
        *out = 0u;
        return SIM_OK;
    }
    for (;;) {
        const char *next = find_json_value_end(p, array_end);

        if (next == 0 || next <= p) {
            return SIM_ERR_CONFIG;
        }
        ++count;
        p = skip_ws(next, array_end);
        if (p >= array_end) {
            *out = count;
            return SIM_OK;
        }
        if (*p != ',') {
            return SIM_ERR_CONFIG;
        }
        p = skip_ws(p + 1, array_end);
        if (p >= array_end) {
            return SIM_ERR_CONFIG;
        }
    }
}

/** @brief 在给定对象范围内定位键值对。 */
static const char *find_key_value(const char *start, const char *end, const char *key)
{
    const char *p = start;
    int depth = 0;

    while (p < end) {
        if (*p == '"') {
            const char *key_start = p + 1;
            const char *key_end = key_start;
            const char *after_string;

            while (key_end < end && *key_end != '"') {
                if (*key_end == '\\') {
                    return 0;
                }
                ++key_end;
            }
            after_string = key_end < end ? key_end + 1 : end;
            if (depth == 0 && key_equals(key_start, key_end, key)) {
                const char *colon = skip_ws(after_string, end);
                if (colon < end && *colon == ':') {
                    return skip_ws(colon + 1, end);
                }
            }
            p = skip_json_string(p, end);
            continue;
        }
        if (*p == '{' || *p == '[') {
            ++depth;
        } else if (*p == '}' || *p == ']') {
            --depth;
        }
        ++p;
    }

    return 0;
}

/** @brief 按点路径查找 JSON 值起始位置。 */
static SimStatus find_path_value(const ConfigTree *config, const char *path, const char **value)
{
    char path_copy[CONFIG_PATH_MAX];
    char key[CONFIG_PATH_MAX];
    char *part;
    const char *start;
    const char *end;

    if (config == 0 || config->data == 0 || path == 0 || value == 0) {
        return SIM_ERR_INVALID_ARG;
    }
    if (strlen(path) >= sizeof(path_copy)) {
        return SIM_ERR_OUT_OF_RANGE;
    }

    start = skip_ws(config->data, config->data + config->size);
    if (start >= config->data + config->size || *start != '{') {
        return SIM_ERR_CONFIG;
    }
    end = find_matching_bracket(start, config->data + config->size);
    if (end == 0) {
        return SIM_ERR_CONFIG;
    }
    ++start;

    (void)strcpy(path_copy, path);
    part = strtok(path_copy, ".");
    while (part != 0) {
        char *next = strtok(0, ".");
        const char *v;
        int has_index = 0;
        size_t index = 0u;
        SimStatus status = parse_path_part(part, key, sizeof(key), &has_index, &index);

        if (status != SIM_OK) {
            return status;
        }
        v = find_key_value(start, end, key);
        if (v == 0) {
            return SIM_ERR_CONFIG;
        }
        if (has_index != 0) {
            status = find_array_element(v, end, index, &v);
            if (status != SIM_OK) {
                return status;
            }
        }
        if (next == 0) {
            *value = v;
            return SIM_OK;
        }
        if (*v != '{') {
            return SIM_ERR_CONFIG;
        }
        start = v + 1;
        end = find_matching_bracket(v, end);
        if (end == 0) {
            return SIM_ERR_CONFIG;
        }
        part = next;
    }

    return SIM_ERR_CONFIG;
}

/** @brief 探测 JSON 文件。 */
SimStatus config_probe_json_file(const char *path, ConfigFile *out)
{
    FILE *file;
    long size;
    int first;

    if (path == 0 || out == 0) {
        return SIM_ERR_INVALID_ARG;
    }

    file = fopen(path, "rb");
    if (file == 0) {
        return SIM_ERR_IO;
    }

    do {
        first = fgetc(file);
    } while (first != EOF && isspace(first));

    if (first != '{' && first != '[') {
        fclose(file);
        return SIM_ERR_CONFIG;
    }

    if (fseek(file, 0, SEEK_END) != 0) {
        fclose(file);
        return SIM_ERR_IO;
    }

    size = ftell(file);
    if (size < 0) {
        fclose(file);
        return SIM_ERR_IO;
    }

    fclose(file);
    out->path = path;
    out->size = (size_t)size;
    return SIM_OK;
}

/** @brief 读取整个文件到内存。 */
SimStatus config_load_file(const char *path, ConfigTree *out)
{
    FILE *file;
    long size;
    size_t read_size;

    if (path == 0 || out == 0) {
        return SIM_ERR_INVALID_ARG;
    }

    out->data = 0;
    out->size = 0u;

    file = fopen(path, "rb");
    if (file == 0) {
        return SIM_ERR_IO;
    }
    if (fseek(file, 0, SEEK_END) != 0) {
        fclose(file);
        return SIM_ERR_IO;
    }
    size = ftell(file);
    if (size <= 0) {
        fclose(file);
        return SIM_ERR_CONFIG;
    }
    if (fseek(file, 0, SEEK_SET) != 0) {
        fclose(file);
        return SIM_ERR_IO;
    }

    out->data = (char *)calloc((size_t)size + 1u, 1u);
    if (out->data == 0) {
        fclose(file);
        return SIM_ERR_INTERNAL;
    }

    read_size = fread(out->data, 1u, (size_t)size, file);
    fclose(file);
    if (read_size != (size_t)size) {
        config_free(out);
        return SIM_ERR_IO;
    }

    out->size = (size_t)size;
    {
        SimStatus status = config_validate_json(out);
        if (status != SIM_OK) {
            config_free(out);
            return status;
        }
    }
    return SIM_OK;
}

/** @brief 释放配置内存。 */
void config_free(ConfigTree *config)
{
    if (config != 0) {
        free(config->data);
        config->data = 0;
        config->size = 0u;
    }
}

/** @brief 校验缓冲区只包含一个完整 JSON 值和尾随空白。 */
SimStatus config_validate_json(const ConfigTree *config)
{
    JsonCursor cursor;
    SimStatus status;

    if (config == 0 || config->data == 0 || config->size == 0u) {
        return SIM_ERR_INVALID_ARG;
    }
    cursor.current = config->data;
    cursor.end = config->data + config->size;
    status = parse_json_value(&cursor, 0u);
    if (status != SIM_OK) {
        return status;
    }
    cursor.current = skip_ws(cursor.current, cursor.end);
    return cursor.current == cursor.end ? SIM_OK : SIM_ERR_CONFIG;
}

/** @brief 校验根对象 schema_version 与程序期望一致。 */
SimStatus config_validate_schema(const ConfigTree *config, unsigned int expected_version)
{
    unsigned int actual_version;
    SimStatus status = config_get_uint32(config, "schema_version", &actual_version);

    if (status != SIM_OK) {
        return status;
    }
    return actual_version == expected_version ? SIM_OK : SIM_ERR_CONFIG;
}

/** @brief 校验指定点路径存在且对应 JSON 对象。 */
SimStatus config_require_section(const ConfigTree *config, const char *path)
{
    const char *value;
    SimStatus status = find_path_value(config, path, &value);

    if (status != SIM_OK) {
        return status;
    }
    return *value == '{' ? SIM_OK : SIM_ERR_CONFIG;
}

/** @brief 读取 JSON 数组长度。 */
SimStatus config_get_array_count(const ConfigTree *config, const char *path, size_t *out)
{
    const char *value;
    SimStatus status = find_path_value(config, path, &value);

    if (status != SIM_OK) {
        return status;
    }
    if (out == 0) {
        return SIM_ERR_INVALID_ARG;
    }
    return count_array_elements(value, config->data + config->size, out);
}

/** @brief 读取浮点字段。 */
SimStatus config_get_double(const ConfigTree *config, const char *path, double *out)
{
    const char *value;
    char *endptr;
    double parsed;
    SimStatus status = find_path_value(config, path, &value);

    if (status != SIM_OK) {
        return status;
    }
    if (out == 0) {
        return SIM_ERR_INVALID_ARG;
    }
    errno = 0;
    parsed = strtod(value, &endptr);
    if (endptr == value || errno == ERANGE || !isfinite(parsed) ||
        (*endptr != ',' && *endptr != '}' && *endptr != ']' &&
         !isspace((unsigned char)*endptr))) {
        return SIM_ERR_CONFIG;
    }
    *out = parsed;
    return SIM_OK;
}

/** @brief 读取并执行 int 范围检查的有符号整数字段。 */
SimStatus config_get_int(const ConfigTree *config, const char *path, int *out)
{
    const char *value;
    char *endptr;
    long parsed;
    SimStatus status = find_path_value(config, path, &value);

    if (status != SIM_OK) {
        return status;
    }
    if (out == 0) {
        return SIM_ERR_INVALID_ARG;
    }
    errno = 0;
    parsed = strtol(value, &endptr, 10);
    if (endptr == value || errno == ERANGE || parsed < INT_MIN || parsed > INT_MAX ||
        (*endptr != ',' && *endptr != '}' && *endptr != ']' &&
         !isspace((unsigned char)*endptr))) {
        return SIM_ERR_CONFIG;
    }
    *out = (int)parsed;
    return SIM_OK;
}

/** @brief 读取无符号整数字段。 */
SimStatus config_get_uint32(const ConfigTree *config, const char *path, unsigned int *out)
{
    const char *value;
    char *endptr;
    unsigned long parsed;
    SimStatus status = find_path_value(config, path, &value);

    if (status != SIM_OK) {
        return status;
    }
    if (out == 0) {
        return SIM_ERR_INVALID_ARG;
    }
    if (*value == '-') {
        return SIM_ERR_CONFIG;
    }
    errno = 0;
    parsed = strtoul(value, &endptr, 10);
    if (endptr == value || errno == ERANGE || parsed > UINT_MAX ||
        (*endptr != ',' && *endptr != '}' && *endptr != ']' &&
         !isspace((unsigned char)*endptr))) {
        return SIM_ERR_CONFIG;
    }
    *out = (unsigned int)parsed;
    return SIM_OK;
}

/** @brief 读取布尔字段。 */
SimStatus config_get_bool(const ConfigTree *config, const char *path, int *out)
{
    const char *value;
    SimStatus status = find_path_value(config, path, &value);

    if (status != SIM_OK) {
        return status;
    }
    if (out == 0) {
        return SIM_ERR_INVALID_ARG;
    }
    if (strncmp(value, "true", 4u) == 0) {
        *out = 1;
        return SIM_OK;
    }
    if (strncmp(value, "false", 5u) == 0) {
        *out = 0;
        return SIM_OK;
    }
    return SIM_ERR_CONFIG;
}

/** @brief 读取字符串字段。 */
SimStatus config_get_string(const ConfigTree *config, const char *path, char *out, size_t out_size)
{
    const char *value;
    const char *end;
    size_t len;
    SimStatus status = find_path_value(config, path, &value);

    if (status != SIM_OK) {
        return status;
    }
    if (out == 0 || out_size == 0u) {
        return SIM_ERR_INVALID_ARG;
    }
    if (*value != '"') {
        return SIM_ERR_CONFIG;
    }
    end = skip_json_string(value, config->data + config->size);
    if (end <= value + 1) {
        return SIM_ERR_CONFIG;
    }
    len = (size_t)((end - 1) - (value + 1));
    if (len >= out_size) {
        return SIM_ERR_OUT_OF_RANGE;
    }
    (void)memcpy(out, value + 1, len);
    out[len] = '\0';
    return SIM_OK;
}

/** @brief 读取定长浮点数组。 */
SimStatus config_get_double_array(const ConfigTree *config, const char *path, double *out, size_t count)
{
    const char *value;
    const char *p;
    const char *end;
    size_t i;
    SimStatus status = find_path_value(config, path, &value);

    if (status != SIM_OK) {
        return status;
    }
    if (out == 0) {
        return SIM_ERR_INVALID_ARG;
    }
    if (*value != '[') {
        return SIM_ERR_CONFIG;
    }
    end = find_matching_bracket(value, config->data + config->size);
    if (end == 0) {
        return SIM_ERR_CONFIG;
    }
    p = value + 1;
    for (i = 0u; i < count; ++i) {
        char *next;
        p = skip_ws(p, end);
        errno = 0;
        out[i] = strtod(p, &next);
        if (next == p || errno == ERANGE || !isfinite(out[i])) {
            return SIM_ERR_CONFIG;
        }
        p = skip_ws(next, end);
        if (i + 1u < count) {
            if (p >= end || *p != ',') {
                return SIM_ERR_CONFIG;
            }
            ++p;
        }
    }
    p = skip_ws(p, end);
    return p == end ? SIM_OK : SIM_ERR_CONFIG;
}
