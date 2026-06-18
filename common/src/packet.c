/** @file packet.c
 *  @brief 报文头和载荷校验实现。
 */
#include "common/packet.h"
#include "common/build_info.h"
#include "common/crc32.h"

#include <math.h>
#include <string.h>

typedef struct PacketWriter {
    /** @brief 调用方提供的目标字节缓冲区。 */
    unsigned char *data;
    /** @brief 目标缓冲区容量，单位字节。 */
    size_t capacity;
    /** @brief 下一次写入的字节偏移。 */
    size_t offset;
} PacketWriter;

typedef struct PacketReader {
    /** @brief 只读线格式报文起始地址。 */
    const unsigned char *data;
    /** @brief 可读取字节总数。 */
    size_t size;
    /** @brief 下一次读取的字节偏移。 */
    size_t offset;
} PacketReader;

/** @brief 按小端字节序写入 16 位整数。 */
static SimStatus writer_u16(PacketWriter *writer, uint16_t value)
{
    if (writer == 0 || writer->offset + 2u > writer->capacity) {
        return SIM_ERR_OUT_OF_RANGE;
    }
    writer->data[writer->offset] = (unsigned char)(value & UINT16_C(0x00ff));
    writer->data[writer->offset + 1u] = (unsigned char)((value >> 8u) & UINT16_C(0x00ff));
    writer->offset += 2u;
    return SIM_OK;
}

/** @brief 按小端字节序写入 32 位整数。 */
static SimStatus writer_u32(PacketWriter *writer, uint32_t value)
{
    size_t index;

    if (writer == 0 || writer->offset + 4u > writer->capacity) {
        return SIM_ERR_OUT_OF_RANGE;
    }
    for (index = 0u; index < 4u; ++index) {
        writer->data[writer->offset + index] =
            (unsigned char)((value >> (8u * index)) & UINT32_C(0xff));
    }
    writer->offset += 4u;
    return SIM_OK;
}

/** @brief 按小端字节序写入 64 位整数。 */
static SimStatus writer_u64(PacketWriter *writer, uint64_t value)
{
    size_t index;

    if (writer == 0 || writer->offset + 8u > writer->capacity) {
        return SIM_ERR_OUT_OF_RANGE;
    }
    for (index = 0u; index < 8u; ++index) {
        writer->data[writer->offset + index] =
            (unsigned char)((value >> (8u * index)) & UINT64_C(0xff));
    }
    writer->offset += 8u;
    return SIM_OK;
}

/** @brief 按 IEEE-754 原始位模式写入双精度值。 */
static SimStatus writer_double(PacketWriter *writer, double value)
{
    uint64_t bits;

    (void)memcpy(&bits, &value, sizeof(bits));
    return writer_u64(writer, bits);
}

/** @brief 依次写入三维向量的 x、y、z 分量。 */
static SimStatus writer_vec3(PacketWriter *writer, Vec3 value)
{
    SimStatus status = writer_double(writer, value.x);

    if (status == SIM_OK) {
        status = writer_double(writer, value.y);
    }
    if (status == SIM_OK) {
        status = writer_double(writer, value.z);
    }
    return status;
}

/** @brief 从小端线格式读取 16 位整数。 */
static SimStatus reader_u16(PacketReader *reader, uint16_t *out)
{
    if (reader == 0 || out == 0 || reader->offset + 2u > reader->size) {
        return SIM_ERR_BAD_PACKET;
    }
    *out = (uint16_t)reader->data[reader->offset] |
        (uint16_t)((uint16_t)reader->data[reader->offset + 1u] << 8u);
    reader->offset += 2u;
    return SIM_OK;
}

/** @brief 从小端线格式读取 32 位整数。 */
static SimStatus reader_u32(PacketReader *reader, uint32_t *out)
{
    size_t index;
    uint32_t value = 0u;

    if (reader == 0 || out == 0 || reader->offset + 4u > reader->size) {
        return SIM_ERR_BAD_PACKET;
    }
    for (index = 0u; index < 4u; ++index) {
        value |= (uint32_t)reader->data[reader->offset + index] << (8u * index);
    }
    reader->offset += 4u;
    *out = value;
    return SIM_OK;
}

/** @brief 从小端线格式读取 64 位整数。 */
static SimStatus reader_u64(PacketReader *reader, uint64_t *out)
{
    size_t index;
    uint64_t value = 0u;

    if (reader == 0 || out == 0 || reader->offset + 8u > reader->size) {
        return SIM_ERR_BAD_PACKET;
    }
    for (index = 0u; index < 8u; ++index) {
        value |= (uint64_t)reader->data[reader->offset + index] << (8u * index);
    }
    reader->offset += 8u;
    *out = value;
    return SIM_OK;
}

/** @brief 从 IEEE-754 原始位模式恢复双精度值。 */
static SimStatus reader_double(PacketReader *reader, double *out)
{
    uint64_t bits;
    SimStatus status = reader_u64(reader, &bits);

    if (status == SIM_OK) {
        (void)memcpy(out, &bits, sizeof(bits));
    }
    return status;
}

/** @brief 依次读取三维向量的 x、y、z 分量。 */
static SimStatus reader_vec3(PacketReader *reader, Vec3 *out)
{
    SimStatus status;

    if (out == 0) {
        return SIM_ERR_INVALID_ARG;
    }
    status = reader_double(reader, &out->x);
    if (status == SIM_OK) {
        status = reader_double(reader, &out->y);
    }
    if (status == SIM_OK) {
        status = reader_double(reader, &out->z);
    }
    return status;
}

/** @brief 将报文头编码为固定 36 字节线格式。 */
static SimStatus encode_header(PacketWriter *writer, const PacketHeader *header)
{
    SimStatus status;

    status = writer_u32(writer, header->magic);
    if (status == SIM_OK) {
        status = writer_u16(writer, header->version_major);
    }
    if (status == SIM_OK) {
        status = writer_u16(writer, header->version_minor);
    }
    if (status == SIM_OK) {
        status = writer_u16(writer, header->type);
    }
    if (status == SIM_OK) {
        status = writer_u16(writer, (uint16_t)SIM_PACKET_HEADER_WIRE_SIZE);
    }
    if (status == SIM_OK) {
        status = writer_u32(writer, header->instance_id);
    }
    if (status == SIM_OK) {
        status = writer_u32(writer, header->seq);
    }
    if (status == SIM_OK) {
        status = writer_double(writer, header->sim_time);
    }
    if (status == SIM_OK) {
        status = writer_u32(writer, header->payload_size);
    }
    if (status == SIM_OK) {
        status = writer_u32(writer, header->payload_crc32);
    }
    return status;
}

/** @brief 从固定 36 字节线格式恢复报文头。 */
static SimStatus decode_header(PacketReader *reader, PacketHeader *header)
{
    SimStatus status;

    status = reader_u32(reader, &header->magic);
    if (status == SIM_OK) {
        status = reader_u16(reader, &header->version_major);
    }
    if (status == SIM_OK) {
        status = reader_u16(reader, &header->version_minor);
    }
    if (status == SIM_OK) {
        status = reader_u16(reader, &header->type);
    }
    if (status == SIM_OK) {
        status = reader_u16(reader, &header->header_size);
    }
    if (status == SIM_OK) {
        status = reader_u32(reader, &header->instance_id);
    }
    if (status == SIM_OK) {
        status = reader_u32(reader, &header->seq);
    }
    if (status == SIM_OK) {
        status = reader_double(reader, &header->sim_time);
    }
    if (status == SIM_OK) {
        status = reader_u32(reader, &header->payload_size);
    }
    if (status == SIM_OK) {
        status = reader_u32(reader, &header->payload_crc32);
    }
    return status;
}

/** @brief 校验线格式报文头的类型、版本、实例号和长度。 */
static SimStatus validate_wire_header(
    const PacketHeader *header,
    PacketType type,
    uint32_t instance_id,
    uint32_t payload_size)
{
    if (header->magic != SIM_PACKET_MAGIC ||
        header->version_major != MISSILE_SIM_PROTOCOL_VERSION_MAJOR ||
        header->header_size != SIM_PACKET_HEADER_WIRE_SIZE ||
        header->type != (uint16_t)type ||
        header->instance_id != instance_id ||
        header->payload_size != payload_size ||
        !isfinite(header->sim_time)) {
        return SIM_ERR_BAD_PACKET;
    }
    return SIM_OK;
}

/** @brief 检查传感器帧所有浮点字段是否有效。 */
static int sensor_frame_isfinite(const SensorFrame *frame)
{
    return frame != 0 &&
        isfinite(frame->sim_time) &&
        isfinite(frame->dt) &&
        vec3_isfinite(frame->missile_vel_ecef_meas) &&
        vec3_isfinite(frame->missile_accel_ecef_meas) &&
        vec3_isfinite(frame->missile_gyro_b_meas) &&
        isfinite(frame->missile_lat_rad_meas) &&
        isfinite(frame->missile_lon_rad_meas) &&
        isfinite(frame->missile_height_m_meas) &&
        isfinite(frame->missile_height_agl_m_meas) &&
        isfinite(frame->target_range_meas) &&
        vec3_isfinite(frame->target_los_unit_ecef_meas) &&
        vec3_isfinite(frame->target_los_rate_ecef_meas) &&
        isfinite(frame->target_closing_velocity_meas);
}

/** @brief 检查控制指令所有浮点字段是否有效。 */
static int control_command_isfinite(const ControlCommand *command)
{
    size_t index;

    if (command == 0 ||
        !isfinite(command->sim_time) ||
        !vec3_isfinite(command->accel_cmd_ecef) ||
        !vec3_isfinite(command->attitude_cmd) ||
        !vec3_isfinite(command->body_rate_cmd)) {
        return 0;
    }
    for (index = 0u; index < SIM_MAX_ACTUATORS; ++index) {
        if (!isfinite(command->actuator_cmd[index])) {
            return 0;
        }
    }
    return 1;
}

/** @brief 将 SensorFrame 字段按 ICD 顺序编码到载荷区。 */
static SimStatus encode_sensor_payload(PacketWriter *writer, const SensorFrame *frame)
{
    SimStatus status = writer_u32(writer, frame->seq);

#define WRITE_SENSOR_DOUBLE(value) \
    do { if (status == SIM_OK) { status = writer_double(writer, (value)); } } while (0)
#define WRITE_SENSOR_VEC3(value) \
    do { if (status == SIM_OK) { status = writer_vec3(writer, (value)); } } while (0)

    WRITE_SENSOR_DOUBLE(frame->sim_time);
    WRITE_SENSOR_DOUBLE(frame->dt);
    WRITE_SENSOR_VEC3(frame->missile_vel_ecef_meas);
    WRITE_SENSOR_VEC3(frame->missile_accel_ecef_meas);
    WRITE_SENSOR_VEC3(frame->missile_gyro_b_meas);
    WRITE_SENSOR_DOUBLE(frame->missile_lat_rad_meas);
    WRITE_SENSOR_DOUBLE(frame->missile_lon_rad_meas);
    WRITE_SENSOR_DOUBLE(frame->missile_height_m_meas);
    WRITE_SENSOR_DOUBLE(frame->missile_height_agl_m_meas);
    WRITE_SENSOR_DOUBLE(frame->target_range_meas);
    WRITE_SENSOR_VEC3(frame->target_los_unit_ecef_meas);
    WRITE_SENSOR_VEC3(frame->target_los_rate_ecef_meas);
    WRITE_SENSOR_DOUBLE(frame->target_closing_velocity_meas);
    if (status == SIM_OK) {
        status = writer_u32(writer, frame->sensor_valid_flags);
    }
    if (status == SIM_OK) {
        status = writer_u32(writer, frame->sensor_fault_flags);
    }
#undef WRITE_SENSOR_DOUBLE
#undef WRITE_SENSOR_VEC3
    return status;
}

/** @brief 按 ICD 顺序解码 SensorFrame 载荷。 */
static SimStatus decode_sensor_payload(PacketReader *reader, SensorFrame *frame)
{
    SimStatus status = reader_u32(reader, &frame->seq);

#define READ_SENSOR_DOUBLE(value) \
    do { if (status == SIM_OK) { status = reader_double(reader, &(value)); } } while (0)
#define READ_SENSOR_VEC3(value) \
    do { if (status == SIM_OK) { status = reader_vec3(reader, &(value)); } } while (0)

    READ_SENSOR_DOUBLE(frame->sim_time);
    READ_SENSOR_DOUBLE(frame->dt);
    READ_SENSOR_VEC3(frame->missile_vel_ecef_meas);
    READ_SENSOR_VEC3(frame->missile_accel_ecef_meas);
    READ_SENSOR_VEC3(frame->missile_gyro_b_meas);
    READ_SENSOR_DOUBLE(frame->missile_lat_rad_meas);
    READ_SENSOR_DOUBLE(frame->missile_lon_rad_meas);
    READ_SENSOR_DOUBLE(frame->missile_height_m_meas);
    READ_SENSOR_DOUBLE(frame->missile_height_agl_m_meas);
    READ_SENSOR_DOUBLE(frame->target_range_meas);
    READ_SENSOR_VEC3(frame->target_los_unit_ecef_meas);
    READ_SENSOR_VEC3(frame->target_los_rate_ecef_meas);
    READ_SENSOR_DOUBLE(frame->target_closing_velocity_meas);
    if (status == SIM_OK) {
        status = reader_u32(reader, &frame->sensor_valid_flags);
    }
    if (status == SIM_OK) {
        status = reader_u32(reader, &frame->sensor_fault_flags);
    }
#undef READ_SENSOR_DOUBLE
#undef READ_SENSOR_VEC3
    return status;
}

/** @brief 将 ControlCommand 字段按 ICD 顺序编码到载荷区。 */
static SimStatus encode_control_payload(PacketWriter *writer, const ControlCommand *command)
{
    SimStatus status = writer_u32(writer, command->seq);
    size_t index;

    if (status == SIM_OK) {
        status = writer_double(writer, command->sim_time);
    }
    if (status == SIM_OK) {
        status = writer_vec3(writer, command->accel_cmd_ecef);
    }
    if (status == SIM_OK) {
        status = writer_vec3(writer, command->attitude_cmd);
    }
    if (status == SIM_OK) {
        status = writer_vec3(writer, command->body_rate_cmd);
    }
    for (index = 0u; index < SIM_MAX_ACTUATORS && status == SIM_OK; ++index) {
        status = writer_double(writer, command->actuator_cmd[index]);
    }
    if (status == SIM_OK) {
        status = writer_u32(writer, command->command_mode);
    }
    if (status == SIM_OK) {
        status = writer_u32(writer, command->command_status);
    }
    return status;
}

/** @brief 按 ICD 顺序解码 ControlCommand 载荷。 */
static SimStatus decode_control_payload(PacketReader *reader, ControlCommand *command)
{
    SimStatus status = reader_u32(reader, &command->seq);
    size_t index;

    if (status == SIM_OK) {
        status = reader_double(reader, &command->sim_time);
    }
    if (status == SIM_OK) {
        status = reader_vec3(reader, &command->accel_cmd_ecef);
    }
    if (status == SIM_OK) {
        status = reader_vec3(reader, &command->attitude_cmd);
    }
    if (status == SIM_OK) {
        status = reader_vec3(reader, &command->body_rate_cmd);
    }
    for (index = 0u; index < SIM_MAX_ACTUATORS && status == SIM_OK; ++index) {
        status = reader_double(reader, &command->actuator_cmd[index]);
    }
    if (status == SIM_OK) {
        status = reader_u32(reader, &command->command_mode);
    }
    if (status == SIM_OK) {
        status = reader_u32(reader, &command->command_status);
    }
    return status;
}

/** @brief 校验报文头字段。 */
SimStatus packet_validate_header(
    const PacketHeader *header,
    PacketType expected_type,
    uint32_t expected_instance_id,
    uint32_t expected_payload_size)
{
    if (header == 0) {
        return SIM_ERR_INVALID_ARG;
    }
    if (header->magic != SIM_PACKET_MAGIC) {
        return SIM_ERR_BAD_PACKET;
    }
    if (header->version_major != MISSILE_SIM_PROTOCOL_VERSION_MAJOR) {
        return SIM_ERR_BAD_PACKET;
    }
    if (header->header_size != sizeof(PacketHeader)) {
        return SIM_ERR_BAD_PACKET;
    }
    if (header->type != (uint16_t)expected_type) {
        return SIM_ERR_BAD_PACKET;
    }
    if (header->instance_id != expected_instance_id) {
        return SIM_ERR_BAD_PACKET;
    }
    if (header->payload_size != expected_payload_size) {
        return SIM_ERR_BAD_PACKET;
    }
    if (!isfinite(header->sim_time)) {
        return SIM_ERR_BAD_PACKET;
    }

    return SIM_OK;
}

/** @brief 校验报文载荷 CRC32。 */
SimStatus packet_validate_payload(const PacketHeader *header, const void *payload)
{
    uint32_t crc;

    if (header == 0 || (payload == 0 && header->payload_size != 0u)) {
        return SIM_ERR_INVALID_ARG;
    }

    crc = crc32_compute(payload, header->payload_size);
    if (crc != header->payload_crc32) {
        return SIM_ERR_BAD_PACKET;
    }

    return SIM_OK;
}

/** @brief 编码带固定头和 CRC 的完整传感器报文。 */
SimStatus packet_encode_sensor_frame(
    uint32_t instance_id,
    const SensorFrame *frame,
    unsigned char *out,
    size_t out_capacity,
    size_t *out_size)
{
    unsigned char payload[SIM_SENSOR_FRAME_WIRE_SIZE];
    PacketWriter payload_writer = { payload, sizeof(payload), 0u };
    PacketWriter packet_writer = { out, out_capacity, 0u };
    PacketHeader header;
    SimStatus status;

    if (frame == 0 || out == 0 || out_size == 0) {
        return SIM_ERR_INVALID_ARG;
    }
    if (out_capacity < SIM_SENSOR_PACKET_WIRE_SIZE) {
        return SIM_ERR_OUT_OF_RANGE;
    }
    if (!sensor_frame_isfinite(frame)) {
        return SIM_ERR_NUMERIC;
    }
    status = encode_sensor_payload(&payload_writer, frame);
    if (status != SIM_OK || payload_writer.offset != SIM_SENSOR_FRAME_WIRE_SIZE) {
        return status == SIM_OK ? SIM_ERR_INTERNAL : status;
    }
    header = packet_header_make(
        PACKET_SENSOR_FRAME,
        instance_id,
        frame->seq,
        frame->sim_time,
        SIM_SENSOR_FRAME_WIRE_SIZE,
        crc32_compute(payload, sizeof(payload)));
    status = encode_header(&packet_writer, &header);
    if (status == SIM_OK) {
        (void)memcpy(out + packet_writer.offset, payload, sizeof(payload));
        packet_writer.offset += sizeof(payload);
    }
    *out_size = packet_writer.offset;
    return status;
}

/** @brief 校验并解码完整传感器报文。 */
SimStatus packet_decode_sensor_frame(
    const unsigned char *data,
    size_t data_size,
    uint32_t expected_instance_id,
    SensorFrame *out)
{
    PacketReader reader = { data, data_size, 0u };
    PacketHeader header;
    SimStatus status;

    if (data == 0 || out == 0) {
        return SIM_ERR_INVALID_ARG;
    }
    if (data_size != SIM_SENSOR_PACKET_WIRE_SIZE) {
        return SIM_ERR_BAD_PACKET;
    }
    status = decode_header(&reader, &header);
    if (status == SIM_OK) {
        status = validate_wire_header(
            &header,
            PACKET_SENSOR_FRAME,
            expected_instance_id,
            SIM_SENSOR_FRAME_WIRE_SIZE);
    }
    if (status == SIM_OK &&
        crc32_compute(data + SIM_PACKET_HEADER_WIRE_SIZE, SIM_SENSOR_FRAME_WIRE_SIZE) !=
            header.payload_crc32) {
        status = SIM_ERR_BAD_PACKET;
    }
    if (status == SIM_OK) {
        status = decode_sensor_payload(&reader, out);
    }
    if (status == SIM_OK &&
        (reader.offset != data_size || out->seq != header.seq ||
         out->sim_time != header.sim_time || !sensor_frame_isfinite(out))) {
        status = SIM_ERR_BAD_PACKET;
    }
    return status;
}

/** @brief 编码带固定头和 CRC 的完整控制指令报文。 */
SimStatus packet_encode_control_command(
    uint32_t instance_id,
    const ControlCommand *command,
    unsigned char *out,
    size_t out_capacity,
    size_t *out_size)
{
    unsigned char payload[SIM_CONTROL_COMMAND_WIRE_SIZE];
    PacketWriter payload_writer = { payload, sizeof(payload), 0u };
    PacketWriter packet_writer = { out, out_capacity, 0u };
    PacketHeader header;
    SimStatus status;

    if (command == 0 || out == 0 || out_size == 0) {
        return SIM_ERR_INVALID_ARG;
    }
    if (out_capacity < SIM_CONTROL_PACKET_WIRE_SIZE) {
        return SIM_ERR_OUT_OF_RANGE;
    }
    if (!control_command_isfinite(command)) {
        return SIM_ERR_NUMERIC;
    }
    status = encode_control_payload(&payload_writer, command);
    if (status != SIM_OK || payload_writer.offset != SIM_CONTROL_COMMAND_WIRE_SIZE) {
        return status == SIM_OK ? SIM_ERR_INTERNAL : status;
    }
    header = packet_header_make(
        PACKET_CONTROL_COMMAND,
        instance_id,
        command->seq,
        command->sim_time,
        SIM_CONTROL_COMMAND_WIRE_SIZE,
        crc32_compute(payload, sizeof(payload)));
    status = encode_header(&packet_writer, &header);
    if (status == SIM_OK) {
        (void)memcpy(out + packet_writer.offset, payload, sizeof(payload));
        packet_writer.offset += sizeof(payload);
    }
    *out_size = packet_writer.offset;
    return status;
}

/** @brief 校验并解码完整控制指令报文。 */
SimStatus packet_decode_control_command(
    const unsigned char *data,
    size_t data_size,
    uint32_t expected_instance_id,
    ControlCommand *out)
{
    PacketReader reader = { data, data_size, 0u };
    PacketHeader header;
    SimStatus status;

    if (data == 0 || out == 0) {
        return SIM_ERR_INVALID_ARG;
    }
    if (data_size != SIM_CONTROL_PACKET_WIRE_SIZE) {
        return SIM_ERR_BAD_PACKET;
    }
    status = decode_header(&reader, &header);
    if (status == SIM_OK) {
        status = validate_wire_header(
            &header,
            PACKET_CONTROL_COMMAND,
            expected_instance_id,
            SIM_CONTROL_COMMAND_WIRE_SIZE);
    }
    if (status == SIM_OK &&
        crc32_compute(data + SIM_PACKET_HEADER_WIRE_SIZE, SIM_CONTROL_COMMAND_WIRE_SIZE) !=
            header.payload_crc32) {
        status = SIM_ERR_BAD_PACKET;
    }
    if (status == SIM_OK) {
        status = decode_control_payload(&reader, out);
    }
    if (status == SIM_OK &&
        (reader.offset != data_size || out->seq != header.seq ||
         out->sim_time != header.sim_time || !control_command_isfinite(out))) {
        status = SIM_ERR_BAD_PACKET;
    }
    return status;
}
