/** @file common_tests.c
 *  @brief common 基础库的确定性单元测试。
 */
#include "common/config.h"
#include "common/crc32.h"
#include "common/math_constants.h"
#include "common/matrix3.h"
#include "common/packet.h"
#include "common/quaternion.h"
#include "common/random.h"
#include "common/ring_buffer.h"
#include "common/vec3.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

/** @brief 记录布尔断言结果并返回失败计数增量。 */
static int expect_int(int condition, const char *name)
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
    return expect_int(fabs(actual - expected) <= tolerance, name);
}

/** @brief 验证向量点积、叉积和范数。 */
static int test_vec3(void)
{
    int failures = 0;
    Vec3 x = vec3_make(1.0, 0.0, 0.0);
    Vec3 y = vec3_make(0.0, 1.0, 0.0);
    Vec3 z = vec3_cross(x, y);

    failures += expect_near(vec3_dot(x, y), 0.0, 1.0e-12, "vec3_dot");
    failures += expect_near(z.z, 1.0, 1.0e-12, "vec3_cross");
    failures += expect_near(vec3_norm(vec3_make(3.0, 4.0, 0.0)), 5.0, 1.0e-12, "vec3_norm");
    return failures;
}

/** @brief 验证姿态积分、旋转方向、DCM 正交性和行列式。 */
static int test_matrix_and_quaternion(void)
{
    int failures = 0;
    Quat input = { 2.0, 0.0, 0.0, 0.0 };
    Quat normalized;
    Quat advanced;
    Matrix3 dcm;
    Matrix3 orthogonality;
    Vec3 rotated;
    SimStatus status;
    size_t row;
    size_t column;

    status = quat_normalize(input, &normalized);
    failures += expect_int(status == SIM_OK, "quat_normalize_status");
    failures += expect_near(normalized.w, 1.0, 1.0e-12, "quat_normalize_value");

    status = quat_integrate(
        quat_identity(),
        vec3_make(0.0, 0.0, SIM_PI),
        0.5,
        &advanced);
    failures += expect_int(status == SIM_OK, "quat_integrate_status");
    status = quat_to_dcm(advanced, &dcm);
    failures += expect_int(status == SIM_OK, "quat_to_dcm_status");
    rotated = matrix3_multiply_vec3(dcm, vec3_make(1.0, 0.0, 0.0));
    failures += expect_near(rotated.x, 0.0, 1.0e-12, "quat_rotation_x");
    failures += expect_near(rotated.y, 1.0, 1.0e-12, "quat_rotation_y");

    orthogonality = matrix3_multiply(dcm, matrix3_transpose(dcm));
    for (row = 0u; row < 3u; ++row) {
        for (column = 0u; column < 3u; ++column) {
            const double expected = row == column ? 1.0 : 0.0;
            failures += expect_near(
                orthogonality.m[row][column],
                expected,
                1.0e-12,
                "dcm_orthogonality");
        }
    }
    failures += expect_near(matrix3_determinant(dcm), 1.0, 1.0e-12, "dcm_determinant");
    {
        Matrix3 inverse;
        Matrix3 identity;

        failures += expect_int(matrix3_inverse(dcm, &inverse) == SIM_OK, "matrix_inverse");
        identity = matrix3_multiply(dcm, inverse);
        failures += expect_near(identity.m[0][0], 1.0, 1.0e-12, "matrix_inverse_identity");
    }
    return failures;
}

/** @brief 验证固定种子产生可重复随机序列。 */
static int test_random(void)
{
    int failures = 0;
    SimRandom first;
    SimRandom second;
    size_t index;

    sim_random_seed(&first, UINT64_C(12345));
    sim_random_seed(&second, UINT64_C(12345));
    for (index = 0u; index < 16u; ++index) {
        failures += expect_int(
            sim_random_next_u64(&first) == sim_random_next_u64(&second),
            "random_repeatability");
    }
    return failures;
}

/** @brief 验证覆盖语义、逻辑顺序和延迟读取。 */
static int test_ring_buffer(void)
{
    int failures = 0;
    int storage[3];
    RingBufferView buffer;
    int value;
    int input;

    failures += expect_int(
        ring_buffer_init(&buffer, storage, 3u, sizeof(storage[0])) == SIM_OK,
        "ring_buffer_init");
    input = 10;
    failures += expect_int(ring_buffer_push(&buffer, &input) == SIM_OK, "ring_buffer_push_10");
    input = 20;
    failures += expect_int(ring_buffer_push(&buffer, &input) == SIM_OK, "ring_buffer_push_20");
    input = 30;
    failures += expect_int(ring_buffer_push(&buffer, &input) == SIM_OK, "ring_buffer_push_30");
    input = 40;
    failures += expect_int(ring_buffer_push(&buffer, &input) == SIM_OK, "ring_buffer_overwrite");
    failures += expect_int(ring_buffer_count(&buffer) == 3u, "ring_buffer_count");
    failures += expect_int(
        ring_buffer_get(&buffer, 0u, &value) == SIM_OK && value == 20,
        "ring_buffer_oldest");
    failures += expect_int(
        ring_buffer_get_delayed(&buffer, 1u, &value) == SIM_OK && value == 30,
        "ring_buffer_delay");
    return failures;
}

/** @brief 验证报文校验、线格式往返、CRC 和实例隔离。 */
static int test_packet(void)
{
    int failures = 0;
    const char payload[] = "packet";
    uint32_t crc = crc32_compute(payload, sizeof(payload));
    PacketHeader header = packet_header_make(
        PACKET_SENSOR_FRAME,
        7u,
        1u,
        0.01,
        (uint32_t)sizeof(payload),
        crc);

    failures += expect_int(packet_validate_header(
                                &header,
                                PACKET_SENSOR_FRAME,
                                7u,
                                (uint32_t)sizeof(payload)) == SIM_OK,
                            "packet_validate_header");
    failures += expect_int(packet_validate_payload(&header, payload) == SIM_OK, "packet_validate_payload");
    header.instance_id = 8u;
    failures += expect_int(packet_validate_header(
                                &header,
                                PACKET_SENSOR_FRAME,
                                7u,
                                (uint32_t)sizeof(payload)) == SIM_ERR_BAD_PACKET,
                            "packet_reject_instance");
    header.instance_id = 7u;
    header.payload_crc32 ^= 1u;
    failures += expect_int(
        packet_validate_payload(&header, payload) == SIM_ERR_BAD_PACKET,
        "packet_reject_crc");

    {
        SensorFrame input;
        SensorFrame decoded;
        unsigned char wire[SIM_SENSOR_PACKET_WIRE_SIZE];
        size_t wire_size = 0u;

        (void)memset(&input, 0, sizeof(input));
        input.seq = 42u;
        input.sim_time = 12.5;
        input.dt = 0.01;
        input.missile_vel_ecef_meas = vec3_make(1.0, 2.0, 3.0);
        input.target_range_meas = 5000.0;
        input.target_los_unit_ecef_meas = vec3_make(0.0, 1.0, 0.0);
        input.target_los_rate_ecef_meas = vec3_make(0.0, 0.0, 0.02);
        input.target_closing_velocity_meas = 400.0;
        input.sensor_valid_flags = 7u;

        failures += expect_int(
            packet_encode_sensor_frame(9u, &input, wire, sizeof(wire), &wire_size) == SIM_OK &&
                wire_size == SIM_SENSOR_PACKET_WIRE_SIZE,
            "sensor_packet_encode");
        failures += expect_int(
            packet_decode_sensor_frame(wire, wire_size, 9u, &decoded) == SIM_OK,
            "sensor_packet_decode");
        failures += expect_int(decoded.seq == input.seq, "sensor_packet_seq");
        failures += expect_near(
            decoded.target_los_rate_ecef_meas.z,
            input.target_los_rate_ecef_meas.z,
            0.0,
            "sensor_packet_value");
        failures += expect_int(
            packet_decode_sensor_frame(wire, wire_size, 10u, &decoded) == SIM_ERR_BAD_PACKET,
            "sensor_packet_instance");
        wire[wire_size - 1u] ^= 1u;
        failures += expect_int(
            packet_decode_sensor_frame(wire, wire_size, 9u, &decoded) == SIM_ERR_BAD_PACKET,
            "sensor_packet_wire_crc");
    }

    {
        ControlCommand input;
        ControlCommand decoded;
        unsigned char wire[SIM_CONTROL_PACKET_WIRE_SIZE];
        size_t wire_size = 0u;

        (void)memset(&input, 0, sizeof(input));
        input.seq = 77u;
        input.sim_time = 3.25;
        input.accel_cmd_ecef = vec3_make(10.0, -20.0, 30.0);
        input.actuator_cmd[2] = 0.15;
        input.command_mode = 4u;
        failures += expect_int(
            packet_encode_control_command(3u, &input, wire, sizeof(wire), &wire_size) == SIM_OK &&
                wire_size == SIM_CONTROL_PACKET_WIRE_SIZE,
            "control_packet_encode");
        failures += expect_int(
            packet_decode_control_command(wire, wire_size, 3u, &decoded) == SIM_OK,
            "control_packet_decode");
        failures += expect_near(decoded.accel_cmd_ecef.y, -20.0, 0.0, "control_packet_value");
        failures += expect_near(decoded.actuator_cmd[2], 0.15, 0.0, "control_packet_actuator");
    }
    return failures;
}

/** @brief 验证 JSON 语法、Schema 和常用字段读取。 */
static int test_config(void)
{
    int failures = 0;
    char valid_json[] =
        "{\"schema_version\":1,\"simulation\":{\"dt\":0.01},"
        "\"enabled\":true,\"offset\":-3,\"values\":[1.0,2.0,3.0],"
        "\"faults\":[{\"time_s\":1.0,\"target\":\"sensor.seeker\"},"
        "{\"time_s\":2.0,\"target\":\"sensor.imu\"}]}";
    char invalid_json[] = "{\"schema_version\":1,\"simulation\":[1,2,]}";
    ConfigTree valid = { valid_json, sizeof(valid_json) - 1u };
    ConfigTree invalid = { invalid_json, sizeof(invalid_json) - 1u };
    double dt = 0.0;
    double values[3];
    double fault_time = 0.0;
    char target[32];
    size_t fault_count = 0u;
    int offset = 0;
    int enabled = 0;

    failures += expect_int(config_validate_json(&valid) == SIM_OK, "config_valid_json");
    failures += expect_int(config_validate_json(&invalid) == SIM_ERR_CONFIG, "config_invalid_json");
    failures += expect_int(config_validate_schema(&valid, 1u) == SIM_OK, "config_schema");
    failures += expect_int(
        config_validate_schema(&valid, 2u) == SIM_ERR_CONFIG,
        "config_schema_reject");
    failures += expect_int(
        config_require_section(&valid, "simulation") == SIM_OK,
        "config_require_section");
    failures += expect_int(
        config_get_double(&valid, "simulation.dt", &dt) == SIM_OK,
        "config_get_double");
    failures += expect_near(dt, 0.01, 0.0, "config_double_value");
    failures += expect_int(
        config_get_int(&valid, "offset", &offset) == SIM_OK && offset == -3,
        "config_get_int");
    failures += expect_int(
        config_get_bool(&valid, "enabled", &enabled) == SIM_OK && enabled != 0,
        "config_get_bool");
    failures += expect_int(
        config_get_double_array(&valid, "values", values, 3u) == SIM_OK,
        "config_get_array");
    failures += expect_near(values[2], 3.0, 0.0, "config_array_value");
    failures += expect_int(
        config_get_array_count(&valid, "faults", &fault_count) == SIM_OK &&
            fault_count == 2u,
        "config_array_count");
    failures += expect_int(
        config_get_double(&valid, "faults[1].time_s", &fault_time) == SIM_OK,
        "config_array_object_double");
    failures += expect_near(fault_time, 2.0, 0.0, "config_array_object_double_value");
    failures += expect_int(
        config_get_string(&valid, "faults[0].target", target, sizeof(target)) == SIM_OK &&
            strcmp(target, "sensor.seeker") == 0,
        "config_array_object_string");
    return failures;
}

/** @brief 运行所有 common 单元测试。 */
int main(void)
{
    int failures = 0;

    failures += test_vec3();
    failures += test_matrix_and_quaternion();
    failures += test_random();
    failures += test_ring_buffer();
    failures += test_packet();
    failures += test_config();

    return failures == 0 ? 0 : 1;
}
