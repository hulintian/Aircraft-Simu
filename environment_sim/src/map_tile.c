/** @file map_tile.c
 *  @brief 内部二进制地形瓦片加载与插值。
 */
#include "env/map_tile.h"

#include "common/crc32.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef struct TileReader {
    /** @brief 瓦片头原始字节起始地址。 */
    const unsigned char *data;
    /** @brief 可读取字节数。 */
    size_t size;
    /** @brief 下一字段的字节偏移。 */
    size_t offset;
} TileReader;

/** @brief 将 16 位整数按小端字节序写入文件。 */
static SimStatus write_u16(FILE *file, uint16_t value)
{
    unsigned char bytes[2];

    bytes[0] = (unsigned char)(value & UINT16_C(0x00ff));
    bytes[1] = (unsigned char)((value >> 8u) & UINT16_C(0x00ff));
    return fwrite(bytes, 1u, sizeof(bytes), file) == sizeof(bytes) ? SIM_OK : SIM_ERR_IO;
}

/** @brief 将 32 位整数按小端字节序写入文件。 */
static SimStatus write_u32(FILE *file, uint32_t value)
{
    unsigned char bytes[4];
    size_t index;

    for (index = 0u; index < sizeof(bytes); ++index) {
        bytes[index] = (unsigned char)((value >> (8u * index)) & UINT32_C(0xff));
    }
    return fwrite(bytes, 1u, sizeof(bytes), file) == sizeof(bytes) ? SIM_OK : SIM_ERR_IO;
}

/** @brief 将双精度原始位模式按小端字节序写入文件。 */
static SimStatus write_double(FILE *file, double value)
{
    uint64_t bits;
    unsigned char bytes[8];
    size_t index;

    (void)memcpy(&bits, &value, sizeof(bits));
    for (index = 0u; index < sizeof(bytes); ++index) {
        bytes[index] = (unsigned char)((bits >> (8u * index)) & UINT64_C(0xff));
    }
    return fwrite(bytes, 1u, sizeof(bytes), file) == sizeof(bytes) ? SIM_OK : SIM_ERR_IO;
}

/** @brief 从瓦片头的小端字节流读取 16 位整数。 */
static SimStatus read_u16(TileReader *reader, uint16_t *out)
{
    if (reader->offset + 2u > reader->size) {
        return SIM_ERR_CONFIG;
    }
    *out = (uint16_t)reader->data[reader->offset] |
        (uint16_t)((uint16_t)reader->data[reader->offset + 1u] << 8u);
    reader->offset += 2u;
    return SIM_OK;
}

/** @brief 从瓦片头的小端字节流读取 32 位整数。 */
static SimStatus read_u32(TileReader *reader, uint32_t *out)
{
    size_t index;
    uint32_t value = 0u;

    if (reader->offset + 4u > reader->size) {
        return SIM_ERR_CONFIG;
    }
    for (index = 0u; index < 4u; ++index) {
        value |= (uint32_t)reader->data[reader->offset + index] << (8u * index);
    }
    reader->offset += 4u;
    *out = value;
    return SIM_OK;
}

/** @brief 从瓦片头读取 IEEE-754 双精度原始位模式。 */
static SimStatus read_double(TileReader *reader, double *out)
{
    uint64_t bits = 0u;
    size_t index;

    if (reader->offset + 8u > reader->size) {
        return SIM_ERR_CONFIG;
    }
    for (index = 0u; index < 8u; ++index) {
        bits |= (uint64_t)reader->data[reader->offset + index] << (8u * index);
    }
    reader->offset += 8u;
    (void)memcpy(out, &bits, sizeof(bits));
    return SIM_OK;
}

/** @brief 校验瓦片版本、边界、网格尺寸和样本数量。 */
static SimStatus validate_header(const TerrainTileHeader *header, size_t sample_count)
{
    size_t expected_count;

    if (header == 0) {
        return SIM_ERR_INVALID_ARG;
    }
    if (header->magic != TERRAIN_TILE_MAGIC ||
        header->version != TERRAIN_TILE_VERSION ||
        header->grid_width < 2u ||
        header->grid_height < 2u ||
        !isfinite(header->lat_min) ||
        !isfinite(header->lat_max) ||
        !isfinite(header->lon_min) ||
        !isfinite(header->lon_max) ||
        !isfinite(header->height_scale) ||
        !isfinite(header->height_offset) ||
        header->lat_min >= header->lat_max ||
        header->lon_min >= header->lon_max ||
        header->height_scale == 0.0) {
        return SIM_ERR_CONFIG;
    }
    expected_count = (size_t)header->grid_width * (size_t)header->grid_height;
    return expected_count == sample_count ? SIM_OK : SIM_ERR_CONFIG;
}

/** @brief 将调用方管理的样本网格绑定到瓦片对象。 */
SimStatus map_tile_bind(
    TerrainTile *tile,
    const TerrainTileHeader *header,
    int16_t *samples,
    size_t sample_count)
{
    SimStatus status;

    if (tile == 0 || samples == 0) {
        return SIM_ERR_INVALID_ARG;
    }
    status = validate_header(header, sample_count);
    if (status != SIM_OK) {
        return status;
    }
    tile->header = *header;
    tile->samples = samples;
    tile->sample_count = sample_count;
    tile->owns_samples = 0;
    return SIM_OK;
}

/** @brief 加载并校验内部瓦片文件。
 *
 *  文件头和样本网格均采用小端格式。函数在初始化阶段分配样本数组，
 *  调用方必须使用 map_tile_unload() 释放。
 */
SimStatus map_tile_load_file(const char *path, TerrainTile *out)
{
    FILE *file;
    long file_size;
    unsigned char *bytes;
    size_t bytes_read;
    size_t sample_count;
    size_t data_size;
    size_t index;
    TileReader reader;
    TerrainTileHeader header;
    SimStatus status;

    if (path == 0 || out == 0) {
        return SIM_ERR_INVALID_ARG;
    }
    (void)memset(out, 0, sizeof(*out));
    file = fopen(path, "rb");
    if (file == 0) {
        return SIM_ERR_IO;
    }
    if (fseek(file, 0, SEEK_END) != 0 || (file_size = ftell(file)) < 0 ||
        fseek(file, 0, SEEK_SET) != 0) {
        (void)fclose(file);
        return SIM_ERR_IO;
    }
    if ((size_t)file_size < TERRAIN_TILE_HEADER_WIRE_SIZE) {
        (void)fclose(file);
        return SIM_ERR_CONFIG;
    }
    bytes = (unsigned char *)malloc((size_t)file_size);
    if (bytes == 0) {
        (void)fclose(file);
        return SIM_ERR_INTERNAL;
    }
    bytes_read = fread(bytes, 1u, (size_t)file_size, file);
    (void)fclose(file);
    if (bytes_read != (size_t)file_size) {
        free(bytes);
        return SIM_ERR_IO;
    }

    reader.data = bytes;
    reader.size = TERRAIN_TILE_HEADER_WIRE_SIZE;
    reader.offset = 0u;
    status = read_u32(&reader, &header.magic);
    if (status == SIM_OK) {
        status = read_u16(&reader, &header.version);
    }
    if (status == SIM_OK) {
        status = read_u16(&reader, &header.grid_width);
    }
    if (status == SIM_OK) {
        status = read_u16(&reader, &header.grid_height);
    }
    if (status == SIM_OK) {
        status = read_double(&reader, &header.lat_min);
    }
    if (status == SIM_OK) {
        status = read_double(&reader, &header.lat_max);
    }
    if (status == SIM_OK) {
        status = read_double(&reader, &header.lon_min);
    }
    if (status == SIM_OK) {
        status = read_double(&reader, &header.lon_max);
    }
    if (status == SIM_OK) {
        status = read_double(&reader, &header.height_scale);
    }
    if (status == SIM_OK) {
        status = read_double(&reader, &header.height_offset);
    }
    if (status == SIM_OK) {
        status = read_u32(&reader, &header.data_crc32);
    }
    if (status != SIM_OK || reader.offset != TERRAIN_TILE_HEADER_WIRE_SIZE) {
        free(bytes);
        return SIM_ERR_CONFIG;
    }

    sample_count = (size_t)header.grid_width * (size_t)header.grid_height;
    if (sample_count > SIZE_MAX / sizeof(int16_t)) {
        free(bytes);
        return SIM_ERR_OUT_OF_RANGE;
    }
    data_size = sample_count * sizeof(int16_t);
    status = validate_header(&header, sample_count);
    if (status != SIM_OK ||
        TERRAIN_TILE_HEADER_WIRE_SIZE + data_size != (size_t)file_size ||
        crc32_compute(bytes + TERRAIN_TILE_HEADER_WIRE_SIZE, data_size) != header.data_crc32) {
        free(bytes);
        return SIM_ERR_CONFIG;
    }

    out->samples = (int16_t *)malloc(data_size);
    if (out->samples == 0) {
        free(bytes);
        return SIM_ERR_INTERNAL;
    }
    for (index = 0u; index < sample_count; ++index) {
        const size_t offset = TERRAIN_TILE_HEADER_WIRE_SIZE + (2u * index);
        const uint16_t raw = (uint16_t)bytes[offset] |
            (uint16_t)((uint16_t)bytes[offset + 1u] << 8u);
        out->samples[index] = (int16_t)raw;
    }
    free(bytes);
    out->header = header;
    out->sample_count = sample_count;
    out->owns_samples = 1;
    return SIM_OK;
}

/** @brief 将内存瓦片序列化为带 CRC 的内部文件格式。 */
SimStatus map_tile_write_file(const char *path, const TerrainTile *tile)
{
    FILE *file;
    unsigned char *sample_bytes;
    size_t data_size;
    size_t index;
    uint32_t crc;
    SimStatus status;

    if (path == 0 || tile == 0 || tile->samples == 0) {
        return SIM_ERR_INVALID_ARG;
    }
    status = validate_header(&tile->header, tile->sample_count);
    if (status != SIM_OK) {
        return status;
    }
    if (tile->sample_count > SIZE_MAX / sizeof(int16_t)) {
        return SIM_ERR_OUT_OF_RANGE;
    }
    data_size = tile->sample_count * sizeof(int16_t);
    sample_bytes = (unsigned char *)malloc(data_size);
    if (sample_bytes == 0) {
        return SIM_ERR_INTERNAL;
    }
    for (index = 0u; index < tile->sample_count; ++index) {
        const uint16_t raw = (uint16_t)tile->samples[index];
        sample_bytes[2u * index] = (unsigned char)(raw & UINT16_C(0x00ff));
        sample_bytes[(2u * index) + 1u] =
            (unsigned char)((raw >> 8u) & UINT16_C(0x00ff));
    }
    crc = crc32_compute(sample_bytes, data_size);

    file = fopen(path, "wb");
    if (file == 0) {
        free(sample_bytes);
        return SIM_ERR_IO;
    }
    status = write_u32(file, tile->header.magic);
    if (status == SIM_OK) {
        status = write_u16(file, tile->header.version);
    }
    if (status == SIM_OK) {
        status = write_u16(file, tile->header.grid_width);
    }
    if (status == SIM_OK) {
        status = write_u16(file, tile->header.grid_height);
    }
    if (status == SIM_OK) {
        status = write_double(file, tile->header.lat_min);
    }
    if (status == SIM_OK) {
        status = write_double(file, tile->header.lat_max);
    }
    if (status == SIM_OK) {
        status = write_double(file, tile->header.lon_min);
    }
    if (status == SIM_OK) {
        status = write_double(file, tile->header.lon_max);
    }
    if (status == SIM_OK) {
        status = write_double(file, tile->header.height_scale);
    }
    if (status == SIM_OK) {
        status = write_double(file, tile->header.height_offset);
    }
    if (status == SIM_OK) {
        status = write_u32(file, crc);
    }
    if (status == SIM_OK &&
        fwrite(sample_bytes, 1u, data_size, file) != data_size) {
        status = SIM_ERR_IO;
    }
    free(sample_bytes);
    if (fclose(file) != 0 && status == SIM_OK) {
        status = SIM_ERR_IO;
    }
    return status;
}

/** @brief 释放由 map_tile_load_file() 分配的样本内存。 */
void map_tile_unload(TerrainTile *tile)
{
    if (tile != 0) {
        if (tile->owns_samples != 0) {
            free(tile->samples);
        }
        (void)memset(tile, 0, sizeof(*tile));
    }
}

/** @brief 以闭区间规则判断查询点是否落在瓦片覆盖范围。 */
int map_tile_contains(const TerrainTile *tile, double lat_rad, double lon_rad)
{
    return tile != 0 && isfinite(lat_rad) && isfinite(lon_rad) &&
        lat_rad >= tile->header.lat_min && lat_rad <= tile->header.lat_max &&
        lon_rad >= tile->header.lon_min && lon_rad <= tile->header.lon_max;
}

/** @brief 在四个相邻网格点之间执行双线性高程插值。 */
SimStatus map_tile_get_height(
    const TerrainTile *tile,
    double lat_rad,
    double lon_rad,
    double *height_m)
{
    double x;
    double y;
    size_t x0;
    size_t y0;
    size_t x1;
    size_t y1;
    double u;
    double v;
    double h00;
    double h10;
    double h01;
    double h11;
    size_t width;

    if (tile == 0 || tile->samples == 0 || height_m == 0) {
        return SIM_ERR_INVALID_ARG;
    }
    if (!map_tile_contains(tile, lat_rad, lon_rad)) {
        return SIM_ERR_OUT_OF_RANGE;
    }

    width = (size_t)tile->header.grid_width;
    x = ((lon_rad - tile->header.lon_min) /
            (tile->header.lon_max - tile->header.lon_min)) *
        (double)(tile->header.grid_width - 1u);
    y = ((lat_rad - tile->header.lat_min) /
            (tile->header.lat_max - tile->header.lat_min)) *
        (double)(tile->header.grid_height - 1u);
    x0 = (size_t)floor(x);
    y0 = (size_t)floor(y);
    x1 = x0 + 1u < tile->header.grid_width ? x0 + 1u : x0;
    y1 = y0 + 1u < tile->header.grid_height ? y0 + 1u : y0;
    u = x - (double)x0;
    v = y - (double)y0;
    h00 = ((double)tile->samples[(y0 * width) + x0] * tile->header.height_scale) +
        tile->header.height_offset;
    h10 = ((double)tile->samples[(y0 * width) + x1] * tile->header.height_scale) +
        tile->header.height_offset;
    h01 = ((double)tile->samples[(y1 * width) + x0] * tile->header.height_scale) +
        tile->header.height_offset;
    h11 = ((double)tile->samples[(y1 * width) + x1] * tile->header.height_scale) +
        tile->header.height_offset;
    *height_m =
        ((1.0 - u) * (1.0 - v) * h00) +
        (u * (1.0 - v) * h10) +
        ((1.0 - u) * v * h01) +
        (u * v * h11);
    return isfinite(*height_m) ? SIM_OK : SIM_ERR_NUMERIC;
}
