/** @file config.h
 *  @brief 简化 JSON 配置读取接口。
 *
 *  仿真工程使用 JSON 作为运行时配置格式。这里提供轻量级加载、字段读取
 *  和数组读取接口，供环境、飞控和实例管理器统一使用。
 */
#ifndef COMMON_CONFIG_H
#define COMMON_CONFIG_H

#include "common/status.h"

#include <stddef.h>

/** @brief 配置文件探测结果。 */
typedef struct ConfigFile {
    const char *path;
    size_t size;
} ConfigFile;

/** @brief 已加载到内存中的 JSON 配置。 */
typedef struct ConfigTree {
    char *data;
    size_t size;
} ConfigTree;

/** @brief 未识别对象键访问回调；object_path 为空字符串表示根对象。 */
typedef void (*ConfigUnknownKeyVisitor)(
    const char *object_path,
    const char *key,
    void *user_data);

/** @brief 探测 JSON 文件是否存在并读取其大小。 */
SimStatus config_probe_json_file(const char *path, ConfigFile *out);
/** @brief 将文件完整加载到内存。 */
SimStatus config_load_file(const char *path, ConfigTree *out);
/** @brief 释放配置树占用的内存。 */
void config_free(ConfigTree *config);
/** @brief 校验配置树是否为完整且语法正确的 JSON 文档。 */
SimStatus config_validate_json(const ConfigTree *config);
/** @brief 校验根对象中的 schema_version。 */
SimStatus config_validate_schema(const ConfigTree *config, unsigned int expected_version);
/** @brief 要求指定点路径存在且值为 JSON 对象。 */
SimStatus config_require_section(const ConfigTree *config, const char *path);
/** @brief 判断指定点路径是否存在；路径或参数非法时返回错误。 */
SimStatus config_path_exists(const ConfigTree *config, const char *path, int *exists_out);
/** @brief 遍历对象的直接键，对不在白名单中的键调用 visitor。 */
SimStatus config_visit_unknown_keys(
    const ConfigTree *config,
    const char *object_path,
    const char *const *allowed_keys,
    size_t allowed_key_count,
    ConfigUnknownKeyVisitor visitor,
    void *user_data,
    size_t *unknown_count_out);
/** @brief 读取 JSON 数组元素数量。点路径支持 `items[0].field` 形式。 */
SimStatus config_get_array_count(const ConfigTree *config, const char *path, size_t *out);
/** @brief 读取 JSON 浮点字段。 */
SimStatus config_get_double(const ConfigTree *config, const char *path, double *out);
/** @brief 读取 JSON 有符号整数字段。 */
SimStatus config_get_int(const ConfigTree *config, const char *path, int *out);
/** @brief 读取 JSON 无符号整数字段。 */
SimStatus config_get_uint32(const ConfigTree *config, const char *path, unsigned int *out);
/** @brief 读取 JSON 布尔字段。 */
SimStatus config_get_bool(const ConfigTree *config, const char *path, int *out);
/** @brief 读取 JSON 字符串字段。 */
SimStatus config_get_string(const ConfigTree *config, const char *path, char *out, size_t out_size);
/** @brief 读取 JSON 浮点数组字段。 */
SimStatus config_get_double_array(const ConfigTree *config, const char *path, double *out, size_t count);

#endif
