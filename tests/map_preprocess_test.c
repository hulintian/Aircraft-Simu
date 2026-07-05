/** @file map_preprocess_test.c
 *  @brief map_preprocess 工具集成测试。
 */
#include "env/map_tile.h"
#include "env/terrain_model.h"

#include <math.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
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

/** @brief 使用绝对误差比较两个双精度值。 */
static int expect_near(double actual, double expected, double tolerance, const char *name)
{
    return expect(fabs(actual - expected) <= tolerance, name);
}

/** @brief 检查二进制索引文件魔数。 */
static int tile_index_has_expected_magic(const char *path)
{
    FILE *file = fopen(path, "rb");
    unsigned char bytes[4];
    uint32_t magic;

    if (file == 0) {
        return 0;
    }
    if (fread(bytes, 1u, sizeof(bytes), file) != sizeof(bytes)) {
        (void)fclose(file);
        return 0;
    }
    (void)fclose(file);
    magic = (uint32_t)bytes[0] |
        ((uint32_t)bytes[1] << 8u) |
        ((uint32_t)bytes[2] << 16u) |
        ((uint32_t)bytes[3] << 24u);
    return magic == TERRAIN_TILE_INDEX_MAGIC;
}

/** @brief 按 SRTM HGT big-endian int16 格式写入一个小型测试 DEM。 */
static int write_hgt_sample(FILE *file, int16_t value)
{
    const uint16_t raw = (uint16_t)value;
    unsigned char bytes[2];

    if (file == 0) {
        return 0;
    }
    bytes[0] = (unsigned char)((raw >> 8u) & UINT16_C(0xff));
    bytes[1] = (unsigned char)(raw & UINT16_C(0xff));
    return fwrite(bytes, 1u, sizeof(bytes), file) == sizeof(bytes);
}

int main(int argc, char **argv)
{
    char grid_path[128];
    char tile_path[128];
    char tile_index_path[128];
    char esri_grid_path[128];
    char esri_tile_path[128];
    char hgt_path[128];
    char hgt_tile_path[128];
    char command[1024];
    FILE *grid;
    TerrainTile tile;
    double height_m = 0.0;
    int failures = 0;

    if (argc != 2) {
        (void)fprintf(stderr, "usage: %s <map_preprocess>\n", argv[0]);
        return 2;
    }
    (void)snprintf(grid_path, sizeof(grid_path), "/tmp/missile_grid_%ld.txt", (long)getpid());
    (void)snprintf(tile_path, sizeof(tile_path), "/tmp/missile_tile_%ld.bin", (long)getpid());
    (void)snprintf(
        tile_index_path,
        sizeof(tile_index_path),
        "/tmp/missile_tile_index_%ld.bin",
        (long)getpid());
    (void)snprintf(
        esri_grid_path,
        sizeof(esri_grid_path),
        "/tmp/missile_esri_grid_%ld.asc",
        (long)getpid());
    (void)snprintf(
        esri_tile_path,
        sizeof(esri_tile_path),
        "/tmp/missile_esri_tile_%ld.bin",
        (long)getpid());
    (void)snprintf(
        hgt_path,
        sizeof(hgt_path),
        "/tmp/N10E020_%ld.hgt",
        (long)getpid());
    (void)snprintf(
        hgt_tile_path,
        sizeof(hgt_tile_path),
        "/tmp/missile_hgt_tile_%ld.bin",
        (long)getpid());
    grid = fopen(grid_path, "w");
    if (grid == 0) {
        return 1;
    }
    (void)fprintf(grid, "0 100\n200 300\n");
    if (fclose(grid) != 0) {
        return 1;
    }

    (void)snprintf(
        command,
        sizeof(command),
        "%s --input %s --output %s --width 2 --height 2 "
        "--lat-min-deg 0 --lat-max-deg 1 --lon-min-deg 0 --lon-max-deg 1 "
        "--index-output %s",
        argv[1],
        grid_path,
        tile_path,
        tile_index_path);
    failures += expect(system(command) == 0, "map_preprocess_command");
    failures += expect(tile_index_has_expected_magic(tile_index_path), "map_preprocess_index_magic");
    failures += expect(map_tile_load_file(tile_path, &tile) == SIM_OK, "map_preprocess_load");
    failures += expect(
        map_tile_get_height(
            &tile,
            0.5 * 0.017453292519943295769236907684886,
            0.5 * 0.017453292519943295769236907684886,
            &height_m) == SIM_OK,
        "map_preprocess_query");
    failures += expect_near(height_m, 150.0, 1.0e-12, "map_preprocess_height");
    map_tile_unload(&tile);

    grid = fopen(esri_grid_path, "w");
    if (grid == 0) {
        return 1;
    }
    (void)fprintf(grid, "ncols 2\n");
    (void)fprintf(grid, "nrows 2\n");
    (void)fprintf(grid, "xllcorner 20\n");
    (void)fprintf(grid, "yllcorner 10\n");
    (void)fprintf(grid, "cellsize 1\n");
    (void)fprintf(grid, "NODATA_value -9999\n");
    (void)fprintf(grid, "100 200\n");
    (void)fprintf(grid, "-9999 50\n");
    if (fclose(grid) != 0) {
        return 1;
    }
    (void)snprintf(
        command,
        sizeof(command),
        "%s --input %s --input-format esri-ascii --output %s --nodata-fill 25",
        argv[1],
        esri_grid_path,
        esri_tile_path);
    failures += expect(system(command) == 0, "map_preprocess_esri_command");
    failures += expect(map_tile_load_file(esri_tile_path, &tile) == SIM_OK, "map_preprocess_esri_load");
    failures += expect(
        map_tile_get_height(
            &tile,
            10.5 * 0.017453292519943295769236907684886,
            20.5 * 0.017453292519943295769236907684886,
            &height_m) == SIM_OK,
        "map_preprocess_esri_query");
    failures += expect_near(height_m, 93.75, 1.0e-12, "map_preprocess_esri_height");
    map_tile_unload(&tile);

    grid = fopen(hgt_path, "wb");
    if (grid == 0) {
        return 1;
    }
    failures += expect(write_hgt_sample(grid, 100), "map_preprocess_hgt_write_00");
    failures += expect(write_hgt_sample(grid, 200), "map_preprocess_hgt_write_01");
    failures += expect(write_hgt_sample(grid, 300), "map_preprocess_hgt_write_10");
    failures += expect(write_hgt_sample(grid, 400), "map_preprocess_hgt_write_11");
    if (fclose(grid) != 0) {
        return 1;
    }
    (void)snprintf(
        command,
        sizeof(command),
        "%s --input %s --input-format srtm-hgt --output %s",
        argv[1],
        hgt_path,
        hgt_tile_path);
    failures += expect(system(command) == 0, "map_preprocess_hgt_command");
    failures += expect(map_tile_load_file(hgt_tile_path, &tile) == SIM_OK, "map_preprocess_hgt_load");
    failures += expect_near(
        tile.header.lat_min,
        10.0 * 0.017453292519943295769236907684886,
        1.0e-12,
        "map_preprocess_hgt_lat_min");
    failures += expect_near(
        tile.header.lon_min,
        20.0 * 0.017453292519943295769236907684886,
        1.0e-12,
        "map_preprocess_hgt_lon_min");
    failures += expect(
        map_tile_get_height(
            &tile,
            10.0 * 0.017453292519943295769236907684886,
            20.0 * 0.017453292519943295769236907684886,
            &height_m) == SIM_OK,
        "map_preprocess_hgt_southwest_query");
    failures += expect_near(height_m, 300.0, 1.0e-12, "map_preprocess_hgt_southwest_height");
    failures += expect(
        map_tile_get_height(
            &tile,
            10.5 * 0.017453292519943295769236907684886,
            20.5 * 0.017453292519943295769236907684886,
            &height_m) == SIM_OK,
        "map_preprocess_hgt_center_query");
    failures += expect_near(height_m, 250.0, 1.0e-12, "map_preprocess_hgt_center_height");
    map_tile_unload(&tile);
    (void)unlink(grid_path);
    (void)unlink(tile_path);
    (void)unlink(tile_index_path);
    (void)unlink(esri_grid_path);
    (void)unlink(esri_tile_path);
    (void)unlink(hgt_path);
    (void)unlink(hgt_tile_path);
    return failures == 0 ? 0 : 1;
}
