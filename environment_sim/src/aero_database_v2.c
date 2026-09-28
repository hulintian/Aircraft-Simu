/** @file aero_database_v2.c
 *  @brief 六维规则网格气动表文件与多线性插值实现。
 */
#include "env/aero_database_v2.h"

#include "common/crc32.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef struct ByteReader {
    const unsigned char *data;
    size_t size;
    size_t offset;
} ByteReader;

/** @brief 计算六维网格点数并检查 size_t 溢出。 */
static SimStatus grid_point_count(const size_t dimensions[6], size_t *out)
{
    size_t count = 1u;
    size_t axis;

    if (dimensions == 0 || out == 0) {
        return SIM_ERR_INVALID_ARG;
    }
    for (axis = 0u; axis < 6u; ++axis) {
        if (dimensions[axis] == 0u || count > SIZE_MAX / dimensions[axis]) {
            return SIM_ERR_OUT_OF_RANGE;
        }
        count *= dimensions[axis];
    }
    *out = count;
    return SIM_OK;
}

/** @brief 校验轴严格递增且系数全部有限。 */
static SimStatus validate_grid(const AeroDatabaseV2Grid *grid)
{
    const double *axes[6];
    size_t dimensions[6];
    size_t expected_count;
    size_t axis;
    size_t index;

    if (grid == 0 || grid->coefficients == 0) {
        return SIM_ERR_INVALID_ARG;
    }
    axes[0] = grid->mach_axis;
    axes[1] = grid->alpha_axis_rad;
    axes[2] = grid->beta_axis_rad;
    axes[3] = grid->height_axis_m;
    axes[4] = grid->pitch_actuator_axis_rad;
    axes[5] = grid->yaw_actuator_axis_rad;
    dimensions[0] = grid->mach_count;
    dimensions[1] = grid->alpha_count;
    dimensions[2] = grid->beta_count;
    dimensions[3] = grid->height_count;
    dimensions[4] = grid->pitch_actuator_count;
    dimensions[5] = grid->yaw_actuator_count;
    if (grid_point_count(dimensions, &expected_count) != SIM_OK ||
        expected_count != grid->coefficient_count) {
        return SIM_ERR_CONFIG;
    }
    for (axis = 0u; axis < 6u; ++axis) {
        if (axes[axis] == 0) {
            return SIM_ERR_INVALID_ARG;
        }
        for (index = 0u; index < dimensions[axis]; ++index) {
            if (!isfinite(axes[axis][index]) ||
                (index > 0u && axes[axis][index] <= axes[axis][index - 1u])) {
                return SIM_ERR_CONFIG;
            }
        }
    }
    if (axes[0][0] < 0.0) {
        return SIM_ERR_CONFIG;
    }
    for (index = 0u; index < grid->coefficient_count; ++index) {
        const AeroCoefficientSet *value = &grid->coefficients[index];

        if (!isfinite(value->cx) || !isfinite(value->cy) || !isfinite(value->cz) ||
            !isfinite(value->cl) || !isfinite(value->cm) || !isfinite(value->cn)) {
            return SIM_ERR_CONFIG;
        }
    }
    return SIM_OK;
}

/** @brief 将数据库字段暴露为只读网格视图。 */
static AeroDatabaseV2Grid database_grid_view(const AeroDatabaseV2 *database)
{
    AeroDatabaseV2Grid grid;

    (void)memset(&grid, 0, sizeof(grid));
    grid.mach_axis = database->axes[0];
    grid.mach_count = database->dimensions[0];
    grid.alpha_axis_rad = database->axes[1];
    grid.alpha_count = database->dimensions[1];
    grid.beta_axis_rad = database->axes[2];
    grid.beta_count = database->dimensions[2];
    grid.height_axis_m = database->axes[3];
    grid.height_count = database->dimensions[3];
    grid.pitch_actuator_axis_rad = database->axes[4];
    grid.pitch_actuator_count = database->dimensions[4];
    grid.yaw_actuator_axis_rad = database->axes[5];
    grid.yaw_actuator_count = database->dimensions[5];
    grid.coefficients = database->coefficients;
    grid.coefficient_count = database->coefficient_count;
    return grid;
}

/** @brief 深拷贝规则网格。 */
static SimStatus copy_grid(AeroDatabaseV2 *database, const AeroDatabaseV2Grid *grid)
{
    const double *axes[6] = {
        grid->mach_axis,
        grid->alpha_axis_rad,
        grid->beta_axis_rad,
        grid->height_axis_m,
        grid->pitch_actuator_axis_rad,
        grid->yaw_actuator_axis_rad
    };
    const size_t dimensions[6] = {
        grid->mach_count,
        grid->alpha_count,
        grid->beta_count,
        grid->height_count,
        grid->pitch_actuator_count,
        grid->yaw_actuator_count
    };
    size_t axis;

    for (axis = 0u; axis < 6u; ++axis) {
        if (dimensions[axis] > SIZE_MAX / sizeof(double)) {
            return SIM_ERR_OUT_OF_RANGE;
        }
        database->axes[axis] = (double *)malloc(dimensions[axis] * sizeof(double));
        if (database->axes[axis] == 0) {
            return SIM_ERR_INTERNAL;
        }
        (void)memcpy(database->axes[axis], axes[axis], dimensions[axis] * sizeof(double));
        database->dimensions[axis] = dimensions[axis];
    }
    if (grid->coefficient_count > SIZE_MAX / sizeof(AeroCoefficientSet)) {
        return SIM_ERR_OUT_OF_RANGE;
    }
    database->coefficients = (AeroCoefficientSet *)malloc(
        grid->coefficient_count * sizeof(AeroCoefficientSet));
    if (database->coefficients == 0) {
        return SIM_ERR_INTERNAL;
    }
    (void)memcpy(
        database->coefficients,
        grid->coefficients,
        grid->coefficient_count * sizeof(AeroCoefficientSet));
    database->coefficient_count = grid->coefficient_count;
    return SIM_OK;
}

SimStatus aero_database_v2_init(
    AeroDatabaseV2 *database,
    const AeroDatabaseV2Grid *grid,
    AeroDatabaseExtrapolationPolicy extrapolation_policy)
{
    SimStatus status;

    if (database == 0 ||
        (extrapolation_policy != AERO_DB_EXTRAPOLATION_ERROR &&
            extrapolation_policy != AERO_DB_EXTRAPOLATION_CLAMP_AND_WARN &&
            extrapolation_policy != AERO_DB_EXTRAPOLATION_HOLD_LAST_VALID)) {
        return SIM_ERR_INVALID_ARG;
    }
    status = validate_grid(grid);
    if (status != SIM_OK) {
        return status;
    }
    (void)memset(database, 0, sizeof(*database));
    status = copy_grid(database, grid);
    if (status != SIM_OK) {
        aero_database_v2_unload(database);
        return status;
    }
    database->extrapolation_policy = extrapolation_policy;
    return SIM_OK;
}

/** @brief 编码小端整数和双精度。 */
static void encode_u16(unsigned char *out, uint16_t value)
{
    out[0] = (unsigned char)(value & UINT16_C(0xff));
    out[1] = (unsigned char)((value >> 8u) & UINT16_C(0xff));
}

static void encode_u32(unsigned char *out, uint32_t value)
{
    size_t index;
    for (index = 0u; index < 4u; ++index) {
        out[index] = (unsigned char)((value >> (8u * index)) & UINT32_C(0xff));
    }
}

static void encode_double(unsigned char *out, double value)
{
    uint64_t bits;
    size_t index;
    (void)memcpy(&bits, &value, sizeof(bits));
    for (index = 0u; index < 8u; ++index) {
        out[index] = (unsigned char)((bits >> (8u * index)) & UINT64_C(0xff));
    }
}

/** @brief 从小端字节流解码整数和双精度。 */
static SimStatus read_u16(ByteReader *reader, uint16_t *out)
{
    if (reader == 0 || out == 0 || reader->offset + 2u > reader->size) {
        return SIM_ERR_CONFIG;
    }
    *out = (uint16_t)reader->data[reader->offset] |
        (uint16_t)((uint16_t)reader->data[reader->offset + 1u] << 8u);
    reader->offset += 2u;
    return SIM_OK;
}

static SimStatus read_u32(ByteReader *reader, uint32_t *out)
{
    size_t index;
    uint32_t value = 0u;
    if (reader == 0 || out == 0 || reader->offset + 4u > reader->size) {
        return SIM_ERR_CONFIG;
    }
    for (index = 0u; index < 4u; ++index) {
        value |= (uint32_t)reader->data[reader->offset + index] << (8u * index);
    }
    reader->offset += 4u;
    *out = value;
    return SIM_OK;
}

static SimStatus read_double(ByteReader *reader, double *out)
{
    uint64_t bits = 0u;
    size_t index;
    if (reader == 0 || out == 0 || reader->offset + 8u > reader->size) {
        return SIM_ERR_CONFIG;
    }
    for (index = 0u; index < 8u; ++index) {
        bits |= (uint64_t)reader->data[reader->offset + index] << (8u * index);
    }
    reader->offset += 8u;
    (void)memcpy(out, &bits, sizeof(bits));
    return SIM_OK;
}

SimStatus aero_database_v2_write_file(
    const char *path,
    const AeroDatabaseV2Grid *grid,
    AeroDatabaseExtrapolationPolicy extrapolation_policy)
{
    const double *axes[6];
    size_t dimensions[6];
    size_t axis_value_count = 0u;
    size_t data_size;
    unsigned char *bytes;
    size_t offset = 0u;
    size_t axis;
    size_t index;
    uint32_t crc;
    FILE *file;
    SimStatus status = validate_grid(grid);

    if (path == 0 || status != SIM_OK) {
        return path == 0 ? SIM_ERR_INVALID_ARG : status;
    }
    if (extrapolation_policy != AERO_DB_EXTRAPOLATION_ERROR &&
        extrapolation_policy != AERO_DB_EXTRAPOLATION_CLAMP_AND_WARN &&
        extrapolation_policy != AERO_DB_EXTRAPOLATION_HOLD_LAST_VALID) {
        return SIM_ERR_INVALID_ARG;
    }
    axes[0] = grid->mach_axis;
    axes[1] = grid->alpha_axis_rad;
    axes[2] = grid->beta_axis_rad;
    axes[3] = grid->height_axis_m;
    axes[4] = grid->pitch_actuator_axis_rad;
    axes[5] = grid->yaw_actuator_axis_rad;
    dimensions[0] = grid->mach_count;
    dimensions[1] = grid->alpha_count;
    dimensions[2] = grid->beta_count;
    dimensions[3] = grid->height_count;
    dimensions[4] = grid->pitch_actuator_count;
    dimensions[5] = grid->yaw_actuator_count;
    for (axis = 0u; axis < 6u; ++axis) {
        if (dimensions[axis] > UINT32_MAX || axis_value_count > SIZE_MAX - dimensions[axis]) {
            return SIM_ERR_OUT_OF_RANGE;
        }
        axis_value_count += dimensions[axis];
    }
    if (axis_value_count > SIZE_MAX / 8u ||
        grid->coefficient_count > SIZE_MAX / AERO_DATABASE_V2_COEFFICIENT_WIRE_SIZE ||
        axis_value_count * 8u > SIZE_MAX -
            grid->coefficient_count * AERO_DATABASE_V2_COEFFICIENT_WIRE_SIZE) {
        return SIM_ERR_OUT_OF_RANGE;
    }
    data_size = axis_value_count * 8u +
        grid->coefficient_count * AERO_DATABASE_V2_COEFFICIENT_WIRE_SIZE;
    bytes = (unsigned char *)malloc(data_size);
    if (bytes == 0) {
        return SIM_ERR_INTERNAL;
    }
    for (axis = 0u; axis < 6u; ++axis) {
        for (index = 0u; index < dimensions[axis]; ++index) {
            encode_double(bytes + offset, axes[axis][index]);
            offset += 8u;
        }
    }
    for (index = 0u; index < grid->coefficient_count; ++index) {
        const AeroCoefficientSet *value = &grid->coefficients[index];
        encode_double(bytes + offset + 0u, value->cx);
        encode_double(bytes + offset + 8u, value->cy);
        encode_double(bytes + offset + 16u, value->cz);
        encode_double(bytes + offset + 24u, value->cl);
        encode_double(bytes + offset + 32u, value->cm);
        encode_double(bytes + offset + 40u, value->cn);
        offset += AERO_DATABASE_V2_COEFFICIENT_WIRE_SIZE;
    }
    crc = crc32_compute(bytes, data_size);
    file = fopen(path, "wb");
    if (file == 0) {
        free(bytes);
        return SIM_ERR_IO;
    }
    {
        unsigned char header[AERO_DATABASE_V2_HEADER_WIRE_SIZE];
        size_t header_offset = 0u;

        encode_u32(header + header_offset, AERO_DATABASE_V2_MAGIC);
        header_offset += 4u;
        encode_u16(header + header_offset, AERO_DATABASE_V2_VERSION);
        header_offset += 2u;
        encode_u16(header + header_offset, AERO_DATABASE_V2_HEADER_WIRE_SIZE);
        header_offset += 2u;
        for (axis = 0u; axis < 6u; ++axis) {
            encode_u32(header + header_offset, (uint32_t)dimensions[axis]);
            header_offset += 4u;
        }
        encode_u32(header + header_offset, (uint32_t)extrapolation_policy);
        header_offset += 4u;
        encode_u32(header + header_offset, (uint32_t)AERO_DB_ANGLE_RADIANS);
        header_offset += 4u;
        encode_u32(header + header_offset, (uint32_t)AERO_DB_COEFFICIENT_DIMENSIONLESS);
        header_offset += 4u;
        encode_u32(header + header_offset, crc);
        header_offset += 4u;
        status = header_offset == sizeof(header) &&
                fwrite(header, 1u, sizeof(header), file) == sizeof(header) &&
                fwrite(bytes, 1u, data_size, file) == data_size ?
            SIM_OK :
            SIM_ERR_IO;
    }
    free(bytes);
    if (fclose(file) != 0 && status == SIM_OK) {
        status = SIM_ERR_IO;
    }
    return status;
}

SimStatus aero_database_v2_load_file(const char *path, AeroDatabaseV2 *database)
{
    FILE *file;
    long file_size;
    unsigned char *bytes;
    ByteReader reader;
    uint32_t magic;
    uint16_t version;
    uint16_t header_size;
    uint32_t dimensions_u32[6];
    uint32_t policy;
    uint32_t angle_unit;
    uint32_t coefficient_unit;
    uint32_t expected_crc;
    size_t dimensions[6];
    size_t axis_value_count = 0u;
    size_t coefficient_count;
    size_t expected_data_size;
    AeroDatabaseV2Grid grid;
    double *axes[6] = { 0 };
    AeroCoefficientSet *coefficients = 0;
    size_t axis;
    size_t index;
    SimStatus status;

    if (path == 0 || database == 0) {
        return SIM_ERR_INVALID_ARG;
    }
    (void)memset(database, 0, sizeof(*database));
    file = fopen(path, "rb");
    if (file == 0) {
        return SIM_ERR_IO;
    }
    if (fseek(file, 0, SEEK_END) != 0 || (file_size = ftell(file)) < 0 ||
        fseek(file, 0, SEEK_SET) != 0 || (size_t)file_size < AERO_DATABASE_V2_HEADER_WIRE_SIZE) {
        (void)fclose(file);
        return SIM_ERR_CONFIG;
    }
    bytes = (unsigned char *)malloc((size_t)file_size);
    if (bytes == 0) {
        (void)fclose(file);
        return SIM_ERR_INTERNAL;
    }
    if (fread(bytes, 1u, (size_t)file_size, file) != (size_t)file_size) {
        free(bytes);
        (void)fclose(file);
        return SIM_ERR_IO;
    }
    (void)fclose(file);
    reader.data = bytes;
    reader.size = AERO_DATABASE_V2_HEADER_WIRE_SIZE;
    reader.offset = 0u;
    status = read_u32(&reader, &magic);
    if (status == SIM_OK) status = read_u16(&reader, &version);
    if (status == SIM_OK) status = read_u16(&reader, &header_size);
    for (axis = 0u; axis < 6u && status == SIM_OK; ++axis) {
        status = read_u32(&reader, &dimensions_u32[axis]);
    }
    if (status == SIM_OK) status = read_u32(&reader, &policy);
    if (status == SIM_OK) status = read_u32(&reader, &angle_unit);
    if (status == SIM_OK) status = read_u32(&reader, &coefficient_unit);
    if (status == SIM_OK) status = read_u32(&reader, &expected_crc);
    if (status != SIM_OK || magic != AERO_DATABASE_V2_MAGIC ||
        version != AERO_DATABASE_V2_VERSION || header_size != AERO_DATABASE_V2_HEADER_WIRE_SIZE ||
        angle_unit != (uint32_t)AERO_DB_ANGLE_RADIANS ||
        coefficient_unit != (uint32_t)AERO_DB_COEFFICIENT_DIMENSIONLESS ||
        (policy != (uint32_t)AERO_DB_EXTRAPOLATION_ERROR &&
            policy != (uint32_t)AERO_DB_EXTRAPOLATION_CLAMP_AND_WARN &&
            policy != (uint32_t)AERO_DB_EXTRAPOLATION_HOLD_LAST_VALID)) {
        free(bytes);
        return SIM_ERR_CONFIG;
    }
    for (axis = 0u; axis < 6u; ++axis) {
        dimensions[axis] = (size_t)dimensions_u32[axis];
        if (axis_value_count > SIZE_MAX - dimensions[axis]) {
            free(bytes);
            return SIM_ERR_OUT_OF_RANGE;
        }
        axis_value_count += dimensions[axis];
    }
    status = grid_point_count(dimensions, &coefficient_count);
    if (status != SIM_OK || axis_value_count > SIZE_MAX / 8u ||
        coefficient_count > SIZE_MAX / AERO_DATABASE_V2_COEFFICIENT_WIRE_SIZE ||
        axis_value_count * 8u > SIZE_MAX -
            coefficient_count * AERO_DATABASE_V2_COEFFICIENT_WIRE_SIZE) {
        free(bytes);
        return SIM_ERR_OUT_OF_RANGE;
    }
    expected_data_size = axis_value_count * 8u +
        coefficient_count * AERO_DATABASE_V2_COEFFICIENT_WIRE_SIZE;
    if (expected_data_size > SIZE_MAX - AERO_DATABASE_V2_HEADER_WIRE_SIZE ||
        AERO_DATABASE_V2_HEADER_WIRE_SIZE + expected_data_size != (size_t)file_size ||
        crc32_compute(bytes + AERO_DATABASE_V2_HEADER_WIRE_SIZE, expected_data_size) != expected_crc) {
        free(bytes);
        return SIM_ERR_CONFIG;
    }
    reader.data = bytes + AERO_DATABASE_V2_HEADER_WIRE_SIZE;
    reader.size = expected_data_size;
    reader.offset = 0u;
    for (axis = 0u; axis < 6u && status == SIM_OK; ++axis) {
        axes[axis] = (double *)malloc(dimensions[axis] * sizeof(double));
        if (axes[axis] == 0) {
            status = SIM_ERR_INTERNAL;
            break;
        }
        for (index = 0u; index < dimensions[axis] && status == SIM_OK; ++index) {
            status = read_double(&reader, &axes[axis][index]);
        }
    }
    if (status == SIM_OK) {
        coefficients = (AeroCoefficientSet *)calloc(coefficient_count, sizeof(*coefficients));
        status = coefficients != 0 ? SIM_OK : SIM_ERR_INTERNAL;
    }
    for (index = 0u; index < coefficient_count && status == SIM_OK; ++index) {
        status = read_double(&reader, &coefficients[index].cx);
        if (status == SIM_OK) status = read_double(&reader, &coefficients[index].cy);
        if (status == SIM_OK) status = read_double(&reader, &coefficients[index].cz);
        if (status == SIM_OK) status = read_double(&reader, &coefficients[index].cl);
        if (status == SIM_OK) status = read_double(&reader, &coefficients[index].cm);
        if (status == SIM_OK) status = read_double(&reader, &coefficients[index].cn);
    }
    free(bytes);
    if (status != SIM_OK) {
        for (axis = 0u; axis < 6u; ++axis) free(axes[axis]);
        free(coefficients);
        return status;
    }
    (void)memset(&grid, 0, sizeof(grid));
    grid.mach_axis = axes[0]; grid.mach_count = dimensions[0];
    grid.alpha_axis_rad = axes[1]; grid.alpha_count = dimensions[1];
    grid.beta_axis_rad = axes[2]; grid.beta_count = dimensions[2];
    grid.height_axis_m = axes[3]; grid.height_count = dimensions[3];
    grid.pitch_actuator_axis_rad = axes[4]; grid.pitch_actuator_count = dimensions[4];
    grid.yaw_actuator_axis_rad = axes[5]; grid.yaw_actuator_count = dimensions[5];
    grid.coefficients = coefficients; grid.coefficient_count = coefficient_count;
    status = validate_grid(&grid);
    if (status == SIM_OK) {
        for (axis = 0u; axis < 6u; ++axis) {
            database->axes[axis] = axes[axis];
            database->dimensions[axis] = dimensions[axis];
        }
        database->coefficients = coefficients;
        database->coefficient_count = coefficient_count;
        database->extrapolation_policy = (AeroDatabaseExtrapolationPolicy)policy;
    } else {
        for (axis = 0u; axis < 6u; ++axis) free(axes[axis]);
        free(coefficients);
    }
    return status;
}

void aero_database_v2_unload(AeroDatabaseV2 *database)
{
    size_t axis;
    if (database != 0) {
        for (axis = 0u; axis < 6u; ++axis) free(database->axes[axis]);
        free(database->coefficients);
        (void)memset(database, 0, sizeof(*database));
    }
}

/** @brief 找到轴包围区间和右端权重。 */
static void find_bracket(
    const double *axis,
    size_t count,
    double value,
    size_t *lower,
    size_t *upper,
    double *weight)
{
    size_t index;
    if (count == 1u) {
        *lower = 0u; *upper = 0u; *weight = 0.0; return;
    }
    for (index = 0u; index + 1u < count; ++index) {
        if (value <= axis[index + 1u]) {
            *lower = index;
            *upper = index + 1u;
            *weight = (value - axis[index]) / (axis[index + 1u] - axis[index]);
            return;
        }
    }
    *lower = count - 2u; *upper = count - 1u; *weight = 1.0;
}

/** @brief 六维索引转最后轴最快的一维偏移。 */
static size_t flatten_index(const size_t indices[6], const size_t dimensions[6])
{
    size_t result = indices[0];
    size_t axis;
    for (axis = 1u; axis < 6u; ++axis) result = result * dimensions[axis] + indices[axis];
    return result;
}

SimStatus aero_database_v2_lookup(
    AeroDatabaseV2 *database,
    double mach,
    double alpha_rad,
    double beta_rad,
    double height_m,
    double pitch_actuator_rad,
    double yaw_actuator_rad,
    AeroCoefficientSet *out,
    uint32_t *flags)
{
    double values[6] = { mach, alpha_rad, beta_rad, height_m, pitch_actuator_rad, yaw_actuator_rad };
    size_t lower[6];
    size_t upper[6];
    double weights[6];
    size_t axis;
    unsigned int mask;
    int out_of_envelope = 0;
    AeroDatabaseV2Grid grid;

    if (database == 0 || out == 0 || flags == 0) return SIM_ERR_INVALID_ARG;
    grid = database_grid_view(database);
    if (validate_grid(&grid) != SIM_OK) return SIM_ERR_CONFIG;
    *flags = 0u;
    for (axis = 0u; axis < 6u; ++axis) {
        if (!isfinite(values[axis])) return SIM_ERR_INVALID_ARG;
        if (values[axis] < database->axes[axis][0] ||
            values[axis] > database->axes[axis][database->dimensions[axis] - 1u]) {
            out_of_envelope = 1;
        }
    }
    if (out_of_envelope != 0) {
        *flags |= AERO_DB_FLAG_EXTRAPOLATED;
        if (database->extrapolation_policy == AERO_DB_EXTRAPOLATION_ERROR) return SIM_ERR_OUT_OF_RANGE;
        if (database->extrapolation_policy == AERO_DB_EXTRAPOLATION_HOLD_LAST_VALID) {
            if (database->have_last_valid == 0) return SIM_ERR_OUT_OF_RANGE;
            *out = database->last_valid;
            return SIM_OK;
        }
        for (axis = 0u; axis < 6u; ++axis) {
            if (values[axis] < database->axes[axis][0]) values[axis] = database->axes[axis][0];
            if (values[axis] > database->axes[axis][database->dimensions[axis] - 1u])
                values[axis] = database->axes[axis][database->dimensions[axis] - 1u];
        }
    }
    for (axis = 0u; axis < 6u; ++axis) {
        find_bracket(database->axes[axis], database->dimensions[axis], values[axis],
            &lower[axis], &upper[axis], &weights[axis]);
    }
    (void)memset(out, 0, sizeof(*out));
    for (mask = 0u; mask < 64u; ++mask) {
        size_t indices[6];
        double corner_weight = 1.0;
        const AeroCoefficientSet *corner;
        for (axis = 0u; axis < 6u; ++axis) {
            const int use_upper = (mask & (1u << axis)) != 0u;
            indices[axis] = use_upper != 0 ? upper[axis] : lower[axis];
            corner_weight *= use_upper != 0 ? weights[axis] : (1.0 - weights[axis]);
        }
        if (corner_weight == 0.0) continue;
        corner = &database->coefficients[flatten_index(indices, database->dimensions)];
        out->cx += corner_weight * corner->cx;
        out->cy += corner_weight * corner->cy;
        out->cz += corner_weight * corner->cz;
        out->cl += corner_weight * corner->cl;
        out->cm += corner_weight * corner->cm;
        out->cn += corner_weight * corner->cn;
    }
    if (!isfinite(out->cx) || !isfinite(out->cy) || !isfinite(out->cz) ||
        !isfinite(out->cl) || !isfinite(out->cm) || !isfinite(out->cn)) return SIM_ERR_NUMERIC;
    database->last_valid = *out;
    database->have_last_valid = 1;
    return SIM_OK;
}
