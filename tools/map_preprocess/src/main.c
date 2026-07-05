/** @file main.c
 *  @brief ASCII 高程网格到内部地形瓦片格式的预处理工具。
 */
#include "env/map_tile.h"
#include "env/terrain_model.h"

#include <errno.h>
#include <ctype.h>
#include <limits.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define DEG_TO_RAD 0.017453292519943295769236907684886

typedef enum InputFormat {
    INPUT_FORMAT_RAW_GRID = 0,
    INPUT_FORMAT_ESRI_ASCII = 1,
    INPUT_FORMAT_SRTM_HGT = 2
} InputFormat;

typedef struct MapPreprocessArgs {
    const char *input_path;
    const char *output_path;
    const char *index_output_path;
    InputFormat input_format;
    unsigned long width;
    unsigned long height;
    double lat_min_rad;
    double lat_max_rad;
    double lon_min_rad;
    double lon_max_rad;
    double height_scale;
    double height_offset;
    int nodata_enabled;
    double nodata_value;
    int nodata_fill_enabled;
    double nodata_fill_height_m;
} MapPreprocessArgs;

/** @brief 打印命令行帮助。 */
static void print_usage(const char *program)
{
    (void)fprintf(
        stderr,
        "usage: %s --input grid.txt --output tile.bin --width W --height H "
        "--lat-min-deg A --lat-max-deg B --lon-min-deg C --lon-max-deg D "
        "[--height-scale S] [--height-offset O]\n"
        "       %s --input dem.asc --input-format esri-ascii --output tile.bin "
        "[--height-scale S] [--height-offset O] [--nodata-fill H]\n"
        "       %s --input N30E120.hgt --input-format srtm-hgt --output tile.bin "
        "[--height-scale S] [--height-offset O] [--nodata-fill H]\n"
        "       optional: --index-output tile_index.bin writes a binary spatial index\n",
        program,
        program,
        program);
}

/** @brief 小写化 ASCII 标识符。 */
static void lowercase_key(char *text)
{
    while (text != 0 && *text != '\0') {
        *text = (char)tolower((unsigned char)*text);
        ++text;
    }
}

/** @brief 解析无符号长整型参数。 */
static int parse_ulong(const char *text, unsigned long *out)
{
    char *end = 0;
    unsigned long value;

    if (text == 0 || out == 0 || text[0] == '\0') {
        return 0;
    }
    errno = 0;
    value = strtoul(text, &end, 10);
    if (errno != 0 || end == text || *end != '\0') {
        return 0;
    }
    *out = value;
    return 1;
}

/** @brief 解析双精度参数。 */
static int parse_double(const char *text, double *out)
{
    char *end = 0;
    double value;

    if (text == 0 || out == 0 || text[0] == '\0') {
        return 0;
    }
    errno = 0;
    value = strtod(text, &end);
    if (errno != 0 || end == text || *end != '\0' || !isfinite(value)) {
        return 0;
    }
    *out = value;
    return 1;
}

/** @brief 消费一个带值参数。 */
static int consume_value(int argc, char **argv, int *index, const char **value)
{
    if (index == 0 || value == 0 || *index + 1 >= argc) {
        return 0;
    }
    ++(*index);
    *value = argv[*index];
    return 1;
}

/** @brief 解析命令行参数。 */
static int parse_args(int argc, char **argv, MapPreprocessArgs *args)
{
    int index;

    if (args == 0) {
        return 0;
    }
    (void)memset(args, 0, sizeof(*args));
    args->height_scale = 1.0;
    args->height_offset = 0.0;
    args->input_format = INPUT_FORMAT_RAW_GRID;
    for (index = 1; index < argc; ++index) {
        const char *value = 0;
        if (strcmp(argv[index], "--input") == 0) {
            if (!consume_value(argc, argv, &index, &args->input_path)) {
                return 0;
            }
        } else if (strcmp(argv[index], "--output") == 0) {
            if (!consume_value(argc, argv, &index, &args->output_path)) {
                return 0;
            }
        } else if (strcmp(argv[index], "--index-output") == 0) {
            if (!consume_value(argc, argv, &index, &args->index_output_path)) {
                return 0;
            }
        } else if (strcmp(argv[index], "--input-format") == 0) {
            if (!consume_value(argc, argv, &index, &value)) {
                return 0;
            }
            if (strcmp(value, "raw") == 0 || strcmp(value, "raw-grid") == 0) {
                args->input_format = INPUT_FORMAT_RAW_GRID;
            } else if (strcmp(value, "esri-ascii") == 0 || strcmp(value, "esri") == 0) {
                args->input_format = INPUT_FORMAT_ESRI_ASCII;
            } else if (strcmp(value, "srtm-hgt") == 0 || strcmp(value, "hgt") == 0) {
                args->input_format = INPUT_FORMAT_SRTM_HGT;
            } else {
                return 0;
            }
        } else if (strcmp(argv[index], "--width") == 0) {
            if (!consume_value(argc, argv, &index, &value) ||
                !parse_ulong(value, &args->width)) {
                return 0;
            }
        } else if (strcmp(argv[index], "--height") == 0) {
            if (!consume_value(argc, argv, &index, &value) ||
                !parse_ulong(value, &args->height)) {
                return 0;
            }
        } else if (strcmp(argv[index], "--lat-min-deg") == 0) {
            if (!consume_value(argc, argv, &index, &value) ||
                !parse_double(value, &args->lat_min_rad)) {
                return 0;
            }
            args->lat_min_rad *= DEG_TO_RAD;
        } else if (strcmp(argv[index], "--lat-max-deg") == 0) {
            if (!consume_value(argc, argv, &index, &value) ||
                !parse_double(value, &args->lat_max_rad)) {
                return 0;
            }
            args->lat_max_rad *= DEG_TO_RAD;
        } else if (strcmp(argv[index], "--lon-min-deg") == 0) {
            if (!consume_value(argc, argv, &index, &value) ||
                !parse_double(value, &args->lon_min_rad)) {
                return 0;
            }
            args->lon_min_rad *= DEG_TO_RAD;
        } else if (strcmp(argv[index], "--lon-max-deg") == 0) {
            if (!consume_value(argc, argv, &index, &value) ||
                !parse_double(value, &args->lon_max_rad)) {
                return 0;
            }
            args->lon_max_rad *= DEG_TO_RAD;
        } else if (strcmp(argv[index], "--height-scale") == 0) {
            if (!consume_value(argc, argv, &index, &value) ||
                !parse_double(value, &args->height_scale)) {
                return 0;
            }
        } else if (strcmp(argv[index], "--height-offset") == 0) {
            if (!consume_value(argc, argv, &index, &value) ||
                !parse_double(value, &args->height_offset)) {
                return 0;
            }
        } else if (strcmp(argv[index], "--nodata-fill") == 0) {
            if (!consume_value(argc, argv, &index, &value) ||
                !parse_double(value, &args->nodata_fill_height_m)) {
                return 0;
            }
            args->nodata_fill_enabled = 1;
        } else {
            return 0;
        }
    }
    if (args->input_path == 0 || args->output_path == 0 ||
        args->height_scale == 0.0) {
        return 0;
    }
    if (args->input_format == INPUT_FORMAT_ESRI_ASCII ||
        args->input_format == INPUT_FORMAT_SRTM_HGT) {
        return 1;
    }
    return
        args->width >= 2ul &&
        args->height >= 2ul &&
        args->width <= (unsigned long)UINT16_MAX &&
        args->height <= (unsigned long)UINT16_MAX &&
        args->lat_min_rad < args->lat_max_rad &&
        args->lon_min_rad < args->lon_max_rad;
}

/** @brief 返回路径中的文件名起始位置。 */
static const char *path_basename(const char *path)
{
    const char *base = path;
    const char *cursor;

    if (path == 0) {
        return 0;
    }
    for (cursor = path; *cursor != '\0'; ++cursor) {
        if (*cursor == '/' || *cursor == '\\') {
            base = cursor + 1;
        }
    }
    return base;
}

/** @brief 从 SRTM HGT 文件名和文件大小推导瓦片元数据。 */
static SimStatus read_srtm_hgt_metadata(const char *path, MapPreprocessArgs *args)
{
    FILE *file;
    long file_size;
    unsigned long sample_count;
    unsigned long side;
    const char *name;
    char lat_hemi;
    char lon_hemi;
    int lat_deg;
    int lon_deg;
    double lat_min_deg;
    double lon_min_deg;

    if (path == 0 || args == 0) {
        return SIM_ERR_INVALID_ARG;
    }
    file = fopen(path, "rb");
    if (file == 0) {
        return SIM_ERR_IO;
    }
    if (fseek(file, 0, SEEK_END) != 0 || (file_size = ftell(file)) < 0) {
        (void)fclose(file);
        return SIM_ERR_IO;
    }
    (void)fclose(file);
    if (file_size < 8 || (file_size % 2) != 0) {
        return SIM_ERR_CONFIG;
    }
    sample_count = (unsigned long)((unsigned long)file_size / 2ul);
    side = (unsigned long)(sqrt((double)sample_count) + 0.5);
    if (side < 2ul ||
        side > (unsigned long)UINT16_MAX ||
        side * side != sample_count) {
        return SIM_ERR_CONFIG;
    }
    name = path_basename(path);
    if (name == 0 ||
        sscanf(name, "%c%d%c%d", &lat_hemi, &lat_deg, &lon_hemi, &lon_deg) != 4) {
        return SIM_ERR_CONFIG;
    }
    lat_hemi = (char)toupper((unsigned char)lat_hemi);
    lon_hemi = (char)toupper((unsigned char)lon_hemi);
    if ((lat_hemi != 'N' && lat_hemi != 'S') ||
        (lon_hemi != 'E' && lon_hemi != 'W') ||
        lat_deg < 0 ||
        lat_deg > 89 ||
        lon_deg < 0 ||
        lon_deg > 179) {
        return SIM_ERR_CONFIG;
    }
    lat_min_deg = lat_hemi == 'N' ? (double)lat_deg : -(double)(lat_deg + 1);
    lon_min_deg = lon_hemi == 'E' ? (double)lon_deg : -(double)(lon_deg + 1);
    args->width = side;
    args->height = side;
    args->lat_min_rad = lat_min_deg * DEG_TO_RAD;
    args->lat_max_rad = (lat_min_deg + 1.0) * DEG_TO_RAD;
    args->lon_min_rad = lon_min_deg * DEG_TO_RAD;
    args->lon_max_rad = (lon_min_deg + 1.0) * DEG_TO_RAD;
    args->nodata_enabled = 1;
    args->nodata_value = -32768.0;
    return SIM_OK;
}

/** @brief 读取 ESRI ASCII Grid 头并推导内部瓦片元数据。 */
static SimStatus read_esri_ascii_header(FILE *file, MapPreprocessArgs *args)
{
    int ncols_found = 0;
    int nrows_found = 0;
    int xll_found = 0;
    int yll_found = 0;
    int cellsize_found = 0;
    unsigned long ncols = 0ul;
    unsigned long nrows = 0ul;
    double xll_deg = 0.0;
    double yll_deg = 0.0;
    double cellsize_deg = 0.0;
    unsigned int field;

    if (file == 0 || args == 0) {
        return SIM_ERR_INVALID_ARG;
    }
    for (field = 0u; field < 5u; ++field) {
        char key[64];

        if (fscanf(file, "%63s", key) != 1) {
            return SIM_ERR_CONFIG;
        }
        lowercase_key(key);
        if (strcmp(key, "ncols") == 0) {
            if (fscanf(file, "%lu", &ncols) != 1) {
                return SIM_ERR_CONFIG;
            }
            ncols_found = 1;
        } else if (strcmp(key, "nrows") == 0) {
            if (fscanf(file, "%lu", &nrows) != 1) {
                return SIM_ERR_CONFIG;
            }
            nrows_found = 1;
        } else if (strcmp(key, "xllcorner") == 0) {
            if (fscanf(file, "%lf", &xll_deg) != 1 || !isfinite(xll_deg)) {
                return SIM_ERR_CONFIG;
            }
            xll_found = 1;
        } else if (strcmp(key, "yllcorner") == 0) {
            if (fscanf(file, "%lf", &yll_deg) != 1 || !isfinite(yll_deg)) {
                return SIM_ERR_CONFIG;
            }
            yll_found = 1;
        } else if (strcmp(key, "cellsize") == 0) {
            if (fscanf(file, "%lf", &cellsize_deg) != 1 || !isfinite(cellsize_deg)) {
                return SIM_ERR_CONFIG;
            }
            cellsize_found = 1;
        } else if (strcmp(key, "nodata_value") == 0) {
            return SIM_ERR_CONFIG;
        } else {
            return SIM_ERR_CONFIG;
        }
    }
    {
        long position = ftell(file);
        char key[64];

        if (position < 0) {
            return SIM_ERR_IO;
        }
        if (fscanf(file, "%63s", key) == 1) {
            lowercase_key(key);
            if (strcmp(key, "nodata_value") == 0) {
                if (fscanf(file, "%lf", &args->nodata_value) != 1 || !isfinite(args->nodata_value)) {
                    return SIM_ERR_CONFIG;
                }
                args->nodata_enabled = 1;
            } else if (fseek(file, position, SEEK_SET) != 0) {
                return SIM_ERR_IO;
            }
        } else if (fseek(file, position, SEEK_SET) != 0) {
            return SIM_ERR_IO;
        }
    }
    if (ncols_found == 0 || nrows_found == 0 || xll_found == 0 ||
        yll_found == 0 || cellsize_found == 0 ||
        ncols < 2ul || nrows < 2ul ||
        ncols > (unsigned long)UINT16_MAX ||
        nrows > (unsigned long)UINT16_MAX ||
        cellsize_deg <= 0.0) {
        return SIM_ERR_CONFIG;
    }
    args->width = ncols;
    args->height = nrows;
    args->lon_min_rad = xll_deg * DEG_TO_RAD;
    args->lon_max_rad = (xll_deg + (cellsize_deg * (double)(ncols - 1ul))) * DEG_TO_RAD;
    args->lat_min_rad = yll_deg * DEG_TO_RAD;
    args->lat_max_rad = (yll_deg + (cellsize_deg * (double)(nrows - 1ul))) * DEG_TO_RAD;
    return args->lat_min_rad < args->lat_max_rad && args->lon_min_rad < args->lon_max_rad ?
        SIM_OK :
        SIM_ERR_CONFIG;
}

/** @brief 读取并量化 ASCII 高程样本网格。 */
static SimStatus read_samples(const MapPreprocessArgs *args, int16_t *samples, size_t count)
{
    FILE *file;
    size_t index;

    if (args == 0 || samples == 0 || count == 0u) {
        return SIM_ERR_INVALID_ARG;
    }
    file = fopen(args->input_path, args->input_format == INPUT_FORMAT_SRTM_HGT ? "rb" : "r");
    if (file == 0) {
        return SIM_ERR_IO;
    }
    if (args->input_format == INPUT_FORMAT_ESRI_ASCII) {
        MapPreprocessArgs header_args = *args;
        SimStatus status = read_esri_ascii_header(file, &header_args);

        if (status != SIM_OK) {
            (void)fclose(file);
            return status;
        }
    }
    for (index = 0u; index < count; ++index) {
        double height_m;
        double raw_value;
        long rounded;
        size_t output_index = index;

        if (args->input_format == INPUT_FORMAT_SRTM_HGT) {
            unsigned char bytes[2];
            uint16_t raw_be;
            int16_t signed_height;

            if (fread(bytes, 1u, sizeof(bytes), file) != sizeof(bytes)) {
                (void)fclose(file);
                return SIM_ERR_CONFIG;
            }
            raw_be = (uint16_t)((uint16_t)bytes[0] << 8u) | (uint16_t)bytes[1];
            signed_height = (int16_t)raw_be;
            height_m = (double)signed_height;
        } else {
            if (fscanf(file, "%lf", &height_m) != 1 || !isfinite(height_m)) {
                (void)fclose(file);
                return SIM_ERR_CONFIG;
            }
        }
        if (args->nodata_enabled != 0 && height_m == args->nodata_value) {
            if (args->nodata_fill_enabled == 0) {
                (void)fclose(file);
                return SIM_ERR_CONFIG;
            }
            height_m = args->nodata_fill_height_m;
        }
        raw_value = (height_m - args->height_offset) / args->height_scale;
        if (!isfinite(raw_value) ||
            raw_value < (double)INT16_MIN ||
            raw_value > (double)INT16_MAX) {
            (void)fclose(file);
            return SIM_ERR_OUT_OF_RANGE;
        }
        rounded = lround(raw_value);
        if (rounded < (long)INT16_MIN || rounded > (long)INT16_MAX) {
            (void)fclose(file);
            return SIM_ERR_OUT_OF_RANGE;
        }
        if (args->input_format == INPUT_FORMAT_ESRI_ASCII ||
            args->input_format == INPUT_FORMAT_SRTM_HGT) {
            const size_t row = index / (size_t)args->width;
            const size_t column = index % (size_t)args->width;
            const size_t flipped_row = ((size_t)args->height - 1u) - row;

            output_index = (flipped_row * (size_t)args->width) + column;
        }
        samples[output_index] = (int16_t)rounded;
    }
    if (args->input_format != INPUT_FORMAT_SRTM_HGT && fscanf(file, "%*s") != EOF) {
        (void)fclose(file);
        return SIM_ERR_CONFIG;
    }
    return fclose(file) == 0 ? SIM_OK : SIM_ERR_IO;
}

/** @brief 按小端字节序写入 16 位整数。 */
static SimStatus write_u16_le(FILE *file, uint16_t value)
{
    unsigned char bytes[2];

    if (file == 0) {
        return SIM_ERR_INVALID_ARG;
    }
    bytes[0] = (unsigned char)(value & UINT16_C(0xff));
    bytes[1] = (unsigned char)((value >> 8u) & UINT16_C(0xff));
    return fwrite(bytes, 1u, sizeof(bytes), file) == sizeof(bytes) ? SIM_OK : SIM_ERR_IO;
}

/** @brief 按小端字节序写入 32 位整数。 */
static SimStatus write_u32_le(FILE *file, uint32_t value)
{
    unsigned char bytes[4];
    size_t index;

    if (file == 0) {
        return SIM_ERR_INVALID_ARG;
    }
    for (index = 0u; index < sizeof(bytes); ++index) {
        bytes[index] = (unsigned char)((value >> (8u * index)) & UINT32_C(0xff));
    }
    return fwrite(bytes, 1u, sizeof(bytes), file) == sizeof(bytes) ? SIM_OK : SIM_ERR_IO;
}

/** @brief 按小端字节序写入 IEEE-754 双精度值。 */
static SimStatus write_double_le(FILE *file, double value)
{
    uint64_t bits = 0u;
    unsigned char bytes[8];
    size_t index;

    if (file == 0) {
        return SIM_ERR_INVALID_ARG;
    }
    (void)memcpy(&bits, &value, sizeof(bits));
    for (index = 0u; index < sizeof(bytes); ++index) {
        bytes[index] = (unsigned char)((bits >> (8u * index)) & UINT64_C(0xff));
    }
    return fwrite(bytes, 1u, sizeof(bytes), file) == sizeof(bytes) ? SIM_OK : SIM_ERR_IO;
}

/** @brief 写出包含单瓦片边界和路径的二进制空间索引。 */
static SimStatus write_tile_index_file(
    const char *index_path,
    const char *tile_path,
    const TerrainTileHeader *header)
{
    FILE *file;
    const size_t path_size = tile_path == 0 ? 0u : strlen(tile_path);
    SimStatus status;

    if (index_path == 0 || tile_path == 0 || header == 0 ||
        path_size == 0u ||
        path_size > UINT16_MAX) {
        return SIM_ERR_INVALID_ARG;
    }
    file = fopen(index_path, "wb");
    if (file == 0) {
        return SIM_ERR_IO;
    }
    status = write_u32_le(file, TERRAIN_TILE_INDEX_MAGIC);
    if (status == SIM_OK) {
        status = write_u16_le(file, (uint16_t)TERRAIN_TILE_INDEX_VERSION);
    }
    if (status == SIM_OK) {
        status = write_u16_le(file, 1u);
    }
    if (status == SIM_OK) {
        status = write_double_le(file, header->lat_min);
    }
    if (status == SIM_OK) {
        status = write_double_le(file, header->lat_max);
    }
    if (status == SIM_OK) {
        status = write_double_le(file, header->lon_min);
    }
    if (status == SIM_OK) {
        status = write_double_le(file, header->lon_max);
    }
    if (status == SIM_OK) {
        status = write_u16_le(file, (uint16_t)path_size);
    }
    if (status == SIM_OK &&
        fwrite(tile_path, 1u, path_size, file) != path_size) {
        status = SIM_ERR_IO;
    }
    if (fclose(file) != 0 && status == SIM_OK) {
        status = SIM_ERR_IO;
    }
    return status;
}

int main(int argc, char **argv)
{
    MapPreprocessArgs args;
    TerrainTileHeader header;
    TerrainTile tile;
    int16_t *samples;
    size_t sample_count;
    SimStatus status;

    if (!parse_args(argc, argv, &args)) {
        print_usage(argv[0]);
        return 2;
    }
    if (args.input_format == INPUT_FORMAT_ESRI_ASCII) {
        FILE *metadata = fopen(args.input_path, "r");

        if (metadata == 0) {
            (void)fprintf(stderr, "failed to open grid\n");
            return 1;
        }
        status = read_esri_ascii_header(metadata, &args);
        (void)fclose(metadata);
        if (status != SIM_OK) {
            (void)fprintf(stderr, "failed to read ESRI ASCII header: %s\n", sim_status_to_string(status));
            return 1;
        }
    } else if (args.input_format == INPUT_FORMAT_SRTM_HGT) {
        status = read_srtm_hgt_metadata(args.input_path, &args);
        if (status != SIM_OK) {
            (void)fprintf(stderr, "failed to read SRTM HGT metadata: %s\n", sim_status_to_string(status));
            return 1;
        }
    }
    if (args.width > SIZE_MAX / args.height ||
        (args.width * args.height) > (SIZE_MAX / sizeof(int16_t))) {
        (void)fprintf(stderr, "grid is too large\n");
        return 2;
    }
    sample_count = (size_t)(args.width * args.height);
    samples = (int16_t *)calloc(sample_count, sizeof(*samples));
    if (samples == 0) {
        (void)fprintf(stderr, "out of memory\n");
        return 1;
    }
    status = read_samples(&args, samples, sample_count);
    if (status != SIM_OK) {
        free(samples);
        (void)fprintf(stderr, "failed to read grid: %s\n", sim_status_to_string(status));
        return 1;
    }

    header.magic = TERRAIN_TILE_MAGIC;
    header.version = TERRAIN_TILE_VERSION;
    header.grid_width = (uint16_t)args.width;
    header.grid_height = (uint16_t)args.height;
    header.lat_min = args.lat_min_rad;
    header.lat_max = args.lat_max_rad;
    header.lon_min = args.lon_min_rad;
    header.lon_max = args.lon_max_rad;
    header.height_scale = args.height_scale;
    header.height_offset = args.height_offset;
    header.data_crc32 = 0u;
    status = map_tile_bind(&tile, &header, samples, sample_count);
    if (status == SIM_OK) {
        status = map_tile_write_file(args.output_path, &tile);
    }
    if (status == SIM_OK && args.index_output_path != 0) {
        status = write_tile_index_file(args.index_output_path, args.output_path, &header);
    }
    free(samples);
    if (status != SIM_OK) {
        (void)fprintf(stderr, "failed to write tile: %s\n", sim_status_to_string(status));
        return 1;
    }
    return 0;
}
