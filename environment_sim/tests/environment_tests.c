/** @file environment_tests.c
 *  @brief 地球坐标和地形模型单元测试。
 */
#include "common/math_constants.h"
#include "common/config.h"
#include "common/random.h"
#include "env/actuator_model.h"
#include "env/aero_model.h"
#include "env/atmosphere_model.h"
#include "env/earth_model.h"
#include "env/environment_force_model.h"
#include "env/fault_injection.h"
#include "env/geo_coordinate.h"
#include "env/gravity_model.h"
#include "env/map_tile.h"
#include "env/mass_model.h"
#include "env/missile_plant_6dof.h"
#include "env/propulsion_model.h"
#include "env/sensor_accel.h"
#include "env/sensor_seeker.h"
#include "env/sensor_noise.h"
#include "env/terrain_model.h"

#include <math.h>
#include <stdio.h>
#include <string.h>
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

/** @brief 验证 WGS-84 LLA/ECEF 往返精度。 */
static int test_geo_round_trip(void)
{
    EarthModel earth = earth_model_wgs84();
    LlaCoord input = { 30.0 * SIM_DEG_TO_RAD, 120.0 * SIM_DEG_TO_RAD, 1234.5 };
    LlaCoord output;
    EcefCoord ecef;
    int failures = 0;

    failures += expect(geo_lla_to_ecef(&earth, &input, &ecef) == SIM_OK, "lla_to_ecef");
    failures += expect(geo_ecef_to_lla(&earth, &ecef, &output) == SIM_OK, "ecef_to_lla");
    failures += expect_near(output.lat_rad, input.lat_rad, 1.0e-11, "latitude_round_trip");
    failures += expect_near(output.lon_rad, input.lon_rad, 1.0e-11, "longitude_round_trip");
    failures += expect_near(output.height_m, input.height_m, 1.0e-4, "height_round_trip");
    return failures;
}

/** @brief 验证赤道参考点处 ENU/NED 轴方向。 */
static int test_local_frames(void)
{
    LlaCoord reference = { 0.0, 0.0, 0.0 };
    Matrix3 enu;
    Matrix3 ned;
    Vec3 east;
    Vec3 north;
    Vec3 up;
    Vec3 down;
    int failures = 0;

    failures += expect(geo_ecef_to_enu_matrix(&reference, &enu) == SIM_OK, "enu_matrix");
    east = matrix3_multiply_vec3(enu, vec3_make(0.0, 1.0, 0.0));
    north = matrix3_multiply_vec3(enu, vec3_make(0.0, 0.0, 1.0));
    up = matrix3_multiply_vec3(enu, vec3_make(1.0, 0.0, 0.0));
    failures += expect_near(east.x, 1.0, 1.0e-12, "enu_east");
    failures += expect_near(north.y, 1.0, 1.0e-12, "enu_north");
    failures += expect_near(up.z, 1.0, 1.0e-12, "enu_up");

    failures += expect(geo_ecef_to_ned_matrix(&reference, &ned) == SIM_OK, "ned_matrix");
    down = matrix3_multiply_vec3(ned, vec3_make(-1.0, 0.0, 0.0));
    failures += expect_near(down.z, 1.0, 1.0e-12, "ned_down");
    return failures;
}

/** @brief 验证 DEM 插值、边界、AGL、碰撞和缺瓦片策略。 */
static int test_terrain(void)
{
    int16_t samples[4] = { 0, 100, 200, 300 };
    TerrainTileHeader header = {
        TERRAIN_TILE_MAGIC,
        TERRAIN_TILE_VERSION,
        2u,
        2u,
        0.0,
        1.0,
        0.0,
        1.0,
        1.0,
        0.0,
        0u
    };
    TerrainTile tile;
    TerrainModel terrain;
    LlaCoord position = { 0.5, 0.5, 200.0 };
    double height;
    double agl;
    int collision;
    int failures = 0;

    failures += expect(map_tile_bind(&tile, &header, samples, 4u) == SIM_OK, "tile_bind");
    failures += expect(
        map_tile_get_height(&tile, 0.5, 0.5, &height) == SIM_OK,
        "terrain_interpolate");
    failures += expect_near(height, 150.0, 1.0e-12, "terrain_center_height");
    failures += expect(
        map_tile_get_height(&tile, 1.0, 1.0, &height) == SIM_OK,
        "terrain_boundary");
    failures += expect_near(height, 300.0, 1.0e-12, "terrain_boundary_height");

    failures += expect(
        terrain_model_init(&terrain, &tile, 1u, MAP_MISSING_ERROR, 0.0) == SIM_OK,
        "terrain_init");
    failures += expect(terrain_get_agl(&terrain, &position, &agl) == SIM_OK, "terrain_agl");
    failures += expect_near(agl, 50.0, 1.0e-12, "terrain_agl_value");
    position.height_m = 150.0;
    failures += expect(
        terrain_is_surface_collision(&terrain, &position, &collision) == SIM_OK && collision != 0,
        "terrain_collision");

    failures += expect(
        terrain_model_init(&terrain, &tile, 1u, MAP_MISSING_FLAT_FILL, 25.0) == SIM_OK,
        "terrain_flat_init");
    failures += expect(
        terrain_get_height(&terrain, 2.0, 2.0, &height) == SIM_OK,
        "terrain_missing_flat");
    failures += expect_near(height, 25.0, 0.0, "terrain_missing_flat_value");
    return failures;
}

/** @brief 验证瓦片文件写入、加载、CRC 和样本所有权。 */
static int test_tile_file_round_trip(void)
{
    char path[128];
    int16_t samples[6] = { -10, 0, 10, 20, 30, 40 };
    TerrainTileHeader header = {
        TERRAIN_TILE_MAGIC,
        TERRAIN_TILE_VERSION,
        3u,
        2u,
        -0.1,
        0.1,
        1.0,
        1.2,
        0.5,
        100.0,
        0u
    };
    TerrainTile source;
    TerrainTile loaded;
    double height;
    int failures = 0;

    (void)snprintf(path, sizeof(path), "/tmp/missile_tile_%ld.bin", (long)getpid());
    (void)memset(&source, 0, sizeof(source));
    (void)memset(&loaded, 0, sizeof(loaded));
    failures += expect(
        map_tile_bind(&source, &header, samples, 6u) == SIM_OK,
        "tile_file_bind");
    failures += expect(
        map_tile_write_file(path, &source) == SIM_OK,
        "tile_file_write");
    failures += expect(
        map_tile_load_file(path, &loaded) == SIM_OK,
        "tile_file_load");
    failures += expect(
        loaded.owns_samples != 0 && loaded.sample_count == 6u,
        "tile_file_ownership");
    failures += expect(
        map_tile_get_height(&loaded, 0.1, 1.2, &height) == SIM_OK,
        "tile_file_query");
    failures += expect_near(height, 120.0, 1.0e-12, "tile_file_height");
    map_tile_unload(&loaded);
    (void)unlink(path);
    return failures;
}

/** @brief 验证球形曲率下短程可见和长程地表遮挡。 */
static int test_line_of_sight(void)
{
    EarthModel earth = earth_model_wgs84();
    TerrainModel terrain;
    LlaCoord clear_start = { 0.0, -0.01 * SIM_DEG_TO_RAD, 1000.0 };
    LlaCoord clear_end = { 0.0, 0.01 * SIM_DEG_TO_RAD, 1000.0 };
    LlaCoord blocked_start = { 0.0, -1.0 * SIM_DEG_TO_RAD, 100.0 };
    LlaCoord blocked_end = { 0.0, 1.0 * SIM_DEG_TO_RAD, 100.0 };
    EcefCoord start_ecef;
    EcefCoord end_ecef;
    int occluded = 0;
    int failures = 0;

    failures += expect(
        terrain_model_init(&terrain, 0, 0u, MAP_MISSING_FLAT_FILL, 0.0) == SIM_OK,
        "los_terrain_init");
    failures += expect(
        geo_lla_to_ecef(&earth, &clear_start, &start_ecef) == SIM_OK &&
            geo_lla_to_ecef(&earth, &clear_end, &end_ecef) == SIM_OK,
        "los_clear_ecef");
    failures += expect(
        terrain_line_of_sight_occluded(
            &terrain,
            &earth,
            start_ecef.position_m,
            end_ecef.position_m,
            20u,
            &occluded) == SIM_OK &&
            occluded == 0,
        "los_clear");

    failures += expect(
        geo_lla_to_ecef(&earth, &blocked_start, &start_ecef) == SIM_OK &&
            geo_lla_to_ecef(&earth, &blocked_end, &end_ecef) == SIM_OK,
        "los_blocked_ecef");
    failures += expect(
        terrain_line_of_sight_occluded(
            &terrain,
            &earth,
            start_ecef.position_m,
            end_ecef.position_m,
            20u,
            &occluded) == SIM_OK &&
            occluded != 0,
        "los_blocked");
    return failures;
}

/** @brief 验证执行机构位置、速率限幅和卡滞故障。 */
static int test_actuator(void)
{
    ActuatorState actuator = {
        0.0, 0.0, 0.0, -0.5, 0.5, 0.2, 0.1, 0u
    };
    int failures = 0;

    failures += expect(
        actuator_model_step(&actuator, 1.0, 0.1) == SIM_OK,
        "actuator_step");
    failures += expect_near(actuator.pos, 0.02, 1.0e-12, "actuator_rate_limit");
    actuator.fault_flags = ACTUATOR_FAULT_STUCK;
    failures += expect(
        actuator_model_step(&actuator, -0.5, 0.1) == SIM_OK,
        "actuator_stuck_step");
    failures += expect_near(actuator.pos, 0.02, 1.0e-12, "actuator_stuck_position");
    return failures;
}

/** @brief 验证重力、大气、推进、质量和气动力基础模型。 */
static int test_force_models(void)
{
    EarthModel earth = earth_model_wgs84();
    GravityModel gravity = gravity_model_wgs84();
    AtmosphereModel atmosphere = atmosphere_model_isa();
    AtmosphereState air;
    MassModel mass;
    PropulsionModel propulsion = {
        1, 1000.0, 2.0, 5.0, { 1.0, 0.0, 0.0 }
    };
    AeroModel aero = {
        1, 0.1, 1.0, 0.5, 0.2, 0.1
    };
    Vec3 gravity_accel;
    Vec3 propulsion_force;
    Vec3 aero_force;
    Vec3 aero_moment;
    double mass_flow;
    int failures = 0;

    failures += expect(
        gravity_model_acceleration(
            &gravity,
            vec3_make(earth.semi_major_axis_m, 0.0, 0.0),
            &gravity_accel) == SIM_OK,
        "gravity_evaluate");
    failures += expect_near(gravity_accel.x, -9.798285479, 1.0e-6, "gravity_surface");
    failures += expect(
        atmosphere_model_evaluate(&atmosphere, 0.0, &air) == SIM_OK,
        "atmosphere_evaluate");
    failures += expect_near(air.density_kgpm3, 1.225, 1.0e-3, "atmosphere_density");
    failures += expect(
        mass_model_init(&mass, 80.0, 20.0) == SIM_OK &&
            mass_model_step(&mass, 2.0, 3.0) == SIM_OK,
        "mass_model");
    failures += expect_near(mass.mass_kg, 94.0, 1.0e-12, "mass_value");
    failures += expect(
        propulsion_model_evaluate(
            &propulsion,
            1.0,
            &propulsion_force,
            &mass_flow) == SIM_OK,
        "propulsion_evaluate");
    failures += expect_near(propulsion_force.x, 1000.0, 1.0e-12, "propulsion_force");
    failures += expect_near(mass_flow, 2.0, 1.0e-12, "propulsion_flow");
    failures += expect(
        aero_model_evaluate(
            &aero,
            air.density_kgpm3,
            vec3_make(100.0, 0.0, 0.0),
            0.1,
            -0.1,
            &aero_force,
            &aero_moment) == SIM_OK,
        "aero_evaluate");
    failures += expect(aero_force.x < 0.0, "aero_drag_direction");
    failures += expect(aero_moment.y > 0.0 && aero_moment.z < 0.0, "aero_moment_direction");
    return failures;
}

/** @brief 验证各环境模型汇总为统一 6DOF 力、力矩和重力输入。 */
static int test_environment_force_model(void)
{
    PlantState6Dof plant;
    EnvironmentForceModel model;
    EnvironmentForceInput input;
    EnvironmentForceOutput output;
    double expected_density;
    double expected_drag;
    int failures = 0;

    (void)memset(&plant, 0, sizeof(plant));
    plant.pos_ecef = vec3_make(6378137.0, 0.0, 0.0);
    plant.vel_ecef = vec3_make(100.0, 0.0, 0.0);
    plant.q_bi = quat_identity();
    plant.mass = 100.0;
    plant.inertia_b = matrix3_identity();

    (void)memset(&model, 0, sizeof(model));
    model.gravity = gravity_model_wgs84();
    model.atmosphere = atmosphere_model_isa();
    model.propulsion.enabled = 1;
    model.propulsion.thrust_n = 1000.0;
    model.propulsion.mass_flow_kgps = 2.0;
    model.propulsion.burn_time_s = 5.0;
    model.propulsion.thrust_direction_b = vec3_make(1.0, 0.0, 0.0);
    model.aerodynamics.enabled = 1;
    model.aerodynamics.reference_area_m2 = 0.1;
    model.aerodynamics.reference_length_m = 1.0;
    model.aerodynamics.drag_coefficient = 0.5;
    model.aerodynamics.control_force_coefficient = 0.2;
    model.aerodynamics.control_moment_coefficient = 0.1;
    model.enable_earth_rotation = 1;
    model.earth_rotation_rate_radps = ENV_WGS84_EARTH_ROTATION_RADPS;

    (void)memset(&input, 0, sizeof(input));
    input.plant = &plant;
    input.height_m = 0.0;
    input.virtual_acceleration_ecef_mps2 = vec3_make(1.0, 2.0, 3.0);
    input.pitch_actuator_rad = 0.1;
    input.yaw_actuator_rad = -0.1;
    input.propellant_mass_kg = 1.0;
    input.dt_s = 1.0;

    failures += expect(
        environment_force_model_evaluate(&model, &input, &output) == SIM_OK,
        "environment_force_evaluate");
    expected_density = output.atmosphere.density_kgpm3;
    expected_drag = 0.5 * expected_density * 100.0 * 100.0 *
        model.aerodynamics.reference_area_m2 *
        model.aerodynamics.drag_coefficient;
    failures += expect_near(
        output.propulsion_force_b_n.x,
        500.0,
        1.0e-12,
        "environment_force_propellant_limited_thrust");
    failures += expect_near(
        output.mass_flow_kgps,
        1.0,
        1.0e-12,
        "environment_force_propellant_limited_flow");
    failures += expect_near(
        output.plant_input.force_b_n.x,
        100.0 + 500.0 - expected_drag,
        1.0e-9,
        "environment_force_total_x");
    failures += expect_near(
        output.plant_input.force_b_n.y,
        200.0 -
            (0.5 * expected_density * 100.0 * 100.0 *
                model.aerodynamics.reference_area_m2 *
                model.aerodynamics.control_force_coefficient * 0.1),
        1.0e-9,
        "environment_force_total_y");
    failures += expect(
        output.plant_input.gravity_ecef_mps2.x < -9.0,
        "environment_force_gravity");
    failures += expect(
        output.plant_input.enable_earth_rotation != 0 &&
            output.plant_input.earth_rotation_ecef_radps.z ==
                ENV_WGS84_EARTH_ROTATION_RADPS,
        "environment_force_earth_rotation");
    failures += expect(
        output.plant_input.moment_b_nm.y > 0.0 &&
            output.plant_input.moment_b_nm.z < 0.0,
        "environment_force_moment");
    return failures;
}

/** @brief 验证六自由度 RK4 平动和转动方程。 */
static int test_missile_plant(void)
{
    PlantState6Dof state;
    PlantState6Dof rotating_state;
    PlantInput6Dof input;
    PlantInput6Dof rotating_input;
    int failures = 0;

    (void)memset(&state, 0, sizeof(state));
    (void)memset(&input, 0, sizeof(input));
    state.q_bi = quat_identity();
    state.mass = 2.0;
    state.inertia_b = matrix3_identity();
    input.force_b_n = vec3_make(4.0, 0.0, 0.0);
    failures += expect(
        missile_plant_step(&state, &input, 1.0, INTEGRATOR_RK4) == SIM_OK,
        "plant_rk4_step");
    failures += expect_near(state.pos_ecef.x, 1.0, 1.0e-12, "plant_position");
    failures += expect_near(state.vel_ecef.x, 2.0, 1.0e-12, "plant_velocity");

    (void)memset(&rotating_state, 0, sizeof(rotating_state));
    (void)memset(&rotating_input, 0, sizeof(rotating_input));
    rotating_state.q_bi = quat_identity();
    rotating_state.mass = 2.0;
    rotating_state.inertia_b = matrix3_identity();
    rotating_input.moment_b_nm = vec3_make(0.0, 0.0, 1.0);
    failures += expect(
        missile_plant_step(
            &rotating_state,
            &rotating_input,
            1.0,
            INTEGRATOR_RK4) == SIM_OK,
        "plant_rotation_step");
    failures += expect_near(rotating_state.omega_b.z, 1.0, 1.0e-12, "plant_body_rate");
    failures += expect_near(
        sqrt(
            (rotating_state.q_bi.w * rotating_state.q_bi.w) +
            (rotating_state.q_bi.x * rotating_state.q_bi.x) +
            (rotating_state.q_bi.y * rotating_state.q_bi.y) +
            (rotating_state.q_bi.z * rotating_state.q_bi.z)),
        1.0,
        1.0e-12,
        "plant_quaternion_norm");
    return failures;
}

/** @brief 验证固定种子传感器误差和量化输出可复现。 */
static int test_sensor_noise(void)
{
    SensorNoiseConfig config = {
        1.0, 0.1, 0.01, -100.0, 100.0, 0.01, 0.0
    };
    SensorNoiseState first_state = { 0.0 };
    SensorNoiseState second_state = { 0.0 };
    SimRandom first_random;
    SimRandom second_random;
    double first_measurement = 0.0;
    double second_measurement = 0.0;
    int first_dropped = 0;
    int second_dropped = 0;
    int failures = 0;

    sim_random_seed(&first_random, 123u);
    sim_random_seed(&second_random, 123u);
    failures += expect(
        sensor_noise_apply(
            &config,
            &first_state,
            &first_random,
            10.0,
            0.01,
            &first_measurement,
            &first_dropped) == SIM_OK &&
            sensor_noise_apply(
                &config,
                &second_state,
                &second_random,
                10.0,
                0.01,
                &second_measurement,
                &second_dropped) == SIM_OK,
        "sensor_noise_apply");
    failures += expect_near(first_measurement, second_measurement, 0.0, "sensor_noise_repeat");
    failures += expect(first_dropped == 0 && second_dropped == 0, "sensor_noise_not_dropped");
    failures += expect_near(
        first_measurement / config.resolution,
        round(first_measurement / config.resolution),
        1.0e-9,
        "sensor_noise_quantized");
    return failures;
}

/** @brief 构造测试使用的无误差三轴传感器配置。 */
static SensorVector3Config make_vector_sensor_test_config(double delay_s)
{
    SensorVector3Config config;
    size_t index;

    (void)memset(&config, 0, sizeof(config));
    config.enabled = 1;
    config.sample_period_s = 0.01;
    config.delay_s = delay_s;
    for (index = 0u; index < 3u; ++index) {
        config.axis[index].min_value = -1.0e6;
        config.axis[index].max_value = 1.0e6;
    }
    return config;
}

/** @brief 验证三轴传感器确定性噪声、两步延迟和整帧丢包。 */
static int test_vector_sensor_pipeline(void)
{
    SensorVector3Config config = make_vector_sensor_test_config(0.02);
    AccelSensor first;
    AccelSensor second;
    Vec3 first_measurement;
    Vec3 second_measurement;
    SensorSampleStatus first_status;
    SensorSampleStatus second_status;
    int failures = 0;
    size_t index;

    for (index = 0u; index < 3u; ++index) {
        config.axis[index].white_noise_std = 0.1;
    }
    failures += expect(
        sensor_accel_init(&first, &config, 777u, 0.01) == SIM_OK &&
            sensor_accel_init(&second, &config, 777u, 0.01) == SIM_OK,
        "vector_sensor_init");
    failures += expect(
        sensor_accel_update(
            &first,
            0.0,
            0.01,
            vec3_make(1.0, 2.0, 3.0),
            &first_measurement,
            &first_status) == SIM_OK &&
            sensor_accel_update(
                &second,
                0.0,
                0.01,
                vec3_make(1.0, 2.0, 3.0),
                &second_measurement,
                &second_status) == SIM_OK,
        "vector_sensor_first_update");
    failures += expect(
        first_status.valid == 0 && first_status.delay_warmup != 0,
        "vector_sensor_delay_warmup");
    failures += expect(
        sensor_accel_update(
            &first,
            0.01,
            0.01,
            vec3_make(4.0, 5.0, 6.0),
            &first_measurement,
            &first_status) == SIM_OK &&
            sensor_accel_update(
                &second,
                0.01,
                0.01,
                vec3_make(4.0, 5.0, 6.0),
                &second_measurement,
                &second_status) == SIM_OK,
        "vector_sensor_second_update");
    failures += expect(
        sensor_accel_update(
            &first,
            0.02,
            0.01,
            vec3_make(7.0, 8.0, 9.0),
            &first_measurement,
            &first_status) == SIM_OK &&
            sensor_accel_update(
                &second,
                0.02,
                0.01,
                vec3_make(7.0, 8.0, 9.0),
                &second_measurement,
                &second_status) == SIM_OK,
        "vector_sensor_delayed_update");
    failures += expect(
        first_status.valid != 0 && first_status.delay_warmup == 0,
        "vector_sensor_delayed_valid");
    failures += expect_near(
        first_measurement.x,
        second_measurement.x,
        0.0,
        "vector_sensor_repeat_x");
    failures += expect_near(
        first_measurement.y,
        second_measurement.y,
        0.0,
        "vector_sensor_repeat_y");
    failures += expect_near(
        first_measurement.z,
        second_measurement.z,
        0.0,
        "vector_sensor_repeat_z");

    config = make_vector_sensor_test_config(0.0);
    config.dropout_probability = 1.0;
    failures += expect(
        sensor_accel_init(&first, &config, 778u, 0.01) == SIM_OK &&
            sensor_accel_update(
                &first,
                0.0,
                0.01,
                vec3_make(1.0, 2.0, 3.0),
                &first_measurement,
                &first_status) == SIM_OK,
        "vector_sensor_dropout_update");
    failures += expect(
        first_status.valid == 0 && first_status.dropped != 0,
        "vector_sensor_dropout_status");
    return failures;
}

/** @brief 构造无误差导引头配置。 */
static SeekerSensorConfig make_seeker_test_config(double delay_s)
{
    SeekerSensorConfig config;
    size_t index;

    (void)memset(&config, 0, sizeof(config));
    config.enabled = 1;
    config.sample_period_s = 0.01;
    config.delay_s = delay_s;
    config.range.min_value = 0.0;
    config.range.max_value = 1.0e9;
    config.closing_velocity.min_value = -1.0e5;
    config.closing_velocity.max_value = 1.0e5;
    for (index = 0u; index < 3u; ++index) {
        config.los_unit_axis[index].min_value = -2.0;
        config.los_unit_axis[index].max_value = 2.0;
        config.los_rate_axis[index].min_value = -100.0;
        config.los_rate_axis[index].max_value = 100.0;
    }
    return config;
}

/** @brief 验证导引头相对几何、延迟和丢包状态。 */
static int test_seeker_sensor_pipeline(void)
{
    SeekerSensorConfig config = make_seeker_test_config(0.01);
    SeekerSensor sensor;
    SeekerTruth truth;
    SeekerMeasurement measurement;
    SensorSampleStatus status;
    int failures = 0;

    (void)memset(&truth, 0, sizeof(truth));
    truth.target_position_ecef_m = vec3_make(1000.0, 0.0, 0.0);
    truth.target_velocity_ecef_mps = vec3_make(-100.0, 10.0, 0.0);
    failures += expect(
        sensor_seeker_init(&sensor, &config, 900u, 0.01) == SIM_OK,
        "seeker_sensor_init");
    failures += expect(
        sensor_seeker_update(
            &sensor,
            0.0,
            0.01,
            &truth,
            &measurement,
            &status) == SIM_OK &&
            status.valid == 0 &&
            status.delay_warmup != 0,
        "seeker_sensor_warmup");
    truth.target_position_ecef_m = vec3_make(999.0, 0.1, 0.0);
    failures += expect(
        sensor_seeker_update(
            &sensor,
            0.01,
            0.01,
            &truth,
            &measurement,
            &status) == SIM_OK &&
            status.valid != 0,
        "seeker_sensor_delayed_output");
    failures += expect_near(measurement.range_m, 1000.0, 1.0e-12, "seeker_range");
    failures += expect_near(measurement.los_unit_ecef.x, 1.0, 1.0e-12, "seeker_los");
    failures += expect_near(
        measurement.los_rate_ecef_radps.z,
        0.01,
        1.0e-12,
        "seeker_los_rate");
    failures += expect_near(
        measurement.closing_velocity_mps,
        100.0,
        1.0e-12,
        "seeker_closing_velocity");

    config = make_seeker_test_config(0.0);
    config.dropout_probability = 1.0;
    failures += expect(
        sensor_seeker_init(&sensor, &config, 901u, 0.01) == SIM_OK &&
            sensor_seeker_update(
                &sensor,
                0.0,
                0.01,
                &truth,
                &measurement,
                &status) == SIM_OK,
        "seeker_dropout_update");
    failures += expect(
        status.valid == 0 && status.dropped != 0,
        "seeker_dropout_status");
    return failures;
}

/** @brief 验证故障脚本解析、激活窗口和故障效果应用。 */
static int test_fault_injection(void)
{
    char json[] =
        "{"
        "\"schema_version\":1,"
        "\"faults\":["
        "{"
        "\"id\":\"los_bias\","
        "\"start_time_s\":1.0,"
        "\"duration_s\":2.0,"
        "\"target\":\"sensor.seeker.los_rate\","
        "\"type\":\"BIAS\","
        "\"value_xyz\":[0.0,0.0,0.01]"
        "},"
        "{"
        "\"id\":\"speed_drop\","
        "\"time_s\":1.0,"
        "\"duration_s\":2.0,"
        "\"target\":\"sensor.speedometer\","
        "\"type\":\"DROPOUT\""
        "},"
        "{"
        "\"id\":\"x_stuck\","
        "\"time_s\":1.0,"
        "\"duration_s\":2.0,"
        "\"target\":\"actuator.accel_x\","
        "\"type\":\"STUCK\""
        "},"
        "{"
        "\"id\":\"y_scale\","
        "\"time_s\":1.0,"
        "\"duration_s\":2.0,"
        "\"target\":\"actuator.accel_y\","
        "\"type\":\"SCALE\","
        "\"scale\":0.5"
        "}"
        "]"
        "}";
    ConfigTree config = { json, sizeof(json) - 1u };
    FaultInjection faults;
    FaultStepEffects effects;
    FaultTransition transitions[ENV_MAX_FAULT_TRANSITIONS];
    size_t transition_count = 0u;
    SensorFrame sensor;
    ActuatorState actuators[3];
    double commands[3] = { 10.0, 20.0, 30.0 };
    int failures = 0;

    failures += expect(
        fault_injection_load_config(&config, &faults) == SIM_OK &&
            faults.fault_count == 4u,
        "fault_load");
    failures += expect(
        fault_injection_update(
            &faults,
            0.5,
            &effects,
            transitions,
            ENV_MAX_FAULT_TRANSITIONS,
            &transition_count) == SIM_OK &&
            effects.active_fault_count == 0u &&
            transition_count == 0u,
        "fault_inactive_step");
    failures += expect(
        fault_injection_update(
            &faults,
            1.0,
            &effects,
            transitions,
            ENV_MAX_FAULT_TRANSITIONS,
            &transition_count) == SIM_OK &&
            effects.active_fault_count == 4u &&
            transition_count == 4u,
        "fault_active_step");
    failures += expect(
        transitions[0].active != 0 &&
            strcmp(transitions[0].id, "los_bias") == 0,
        "fault_start_transition");

    (void)memset(&sensor, 0, sizeof(sensor));
    sensor.sensor_valid_flags = SIM_SENSOR_VALID_SEEKER | SIM_SENSOR_VALID_SPEED;
    sensor.target_los_unit_ecef_meas = vec3_make(1.0, 0.0, 0.0);
    sensor.target_los_rate_ecef_meas = vec3_make(0.0, 0.0, 1.0);
    sensor.missile_vel_ecef_meas = vec3_make(100.0, 0.0, 0.0);
    fault_injection_apply_sensor(&effects, &sensor);
    failures += expect_near(
        sensor.target_los_rate_ecef_meas.z,
        1.01,
        1.0e-12,
        "fault_sensor_bias");
    failures += expect(
        (sensor.sensor_valid_flags & SIM_SENSOR_VALID_SPEED) == 0u &&
            (sensor.sensor_fault_flags & SIM_SENSOR_FAULT_SPEED_DROPOUT) != 0u,
        "fault_sensor_dropout");

    (void)memset(actuators, 0, sizeof(actuators));
    fault_injection_apply_actuators(&effects, actuators, commands);
    failures += expect(
        (actuators[0].fault_flags & ACTUATOR_FAULT_STUCK) != 0u,
        "fault_actuator_stuck");
    failures += expect_near(commands[1], 10.0, 1.0e-12, "fault_actuator_scale");

    failures += expect(
        fault_injection_update(
            &faults,
            3.0,
            &effects,
            transitions,
            ENV_MAX_FAULT_TRANSITIONS,
            &transition_count) == SIM_OK &&
            effects.active_fault_count == 0u &&
            transition_count == 4u,
        "fault_recovery_step");
    commands[0] = 10.0;
    commands[1] = 20.0;
    commands[2] = 30.0;
    fault_injection_apply_actuators(&effects, actuators, commands);
    failures += expect(
        (actuators[0].fault_flags & ACTUATOR_FAULT_STUCK) == 0u,
        "fault_actuator_recovered");
    failures += expect_near(commands[1], 20.0, 1.0e-12, "fault_scale_recovered");
    return failures;
}

/** @brief 验证故障脚本错误路径在加载阶段被拒绝。 */
static int test_fault_injection_rejects_bad_config(void)
{
    char bad_target_json[] =
        "{\"schema_version\":1,\"faults\":[{"
        "\"time_s\":1.0,"
        "\"duration_s\":1.0,"
        "\"target\":\"sensor.unknown\","
        "\"type\":\"DROPOUT\""
        "}]}";
    char bad_duration_json[] =
        "{\"schema_version\":1,\"faults\":[{"
        "\"time_s\":1.0,"
        "\"duration_s\":-1.0,"
        "\"target\":\"sensor.speedometer\","
        "\"type\":\"DROPOUT\""
        "}]}";
    char bad_pair_json[] =
        "{\"schema_version\":1,\"faults\":[{"
        "\"time_s\":1.0,"
        "\"duration_s\":1.0,"
        "\"target\":\"sensor.speedometer\","
        "\"type\":\"STUCK\""
        "}]}";
    ConfigTree bad_target = { bad_target_json, sizeof(bad_target_json) - 1u };
    ConfigTree bad_duration = { bad_duration_json, sizeof(bad_duration_json) - 1u };
    ConfigTree bad_pair = { bad_pair_json, sizeof(bad_pair_json) - 1u };
    FaultInjection faults;
    int failures = 0;

    failures += expect(
        fault_injection_load_config(&bad_target, &faults) == SIM_ERR_CONFIG,
        "fault_reject_bad_target");
    failures += expect(
        fault_injection_load_config(&bad_duration, &faults) == SIM_ERR_OUT_OF_RANGE,
        "fault_reject_bad_duration");
    failures += expect(
        fault_injection_load_config(&bad_pair, &faults) == SIM_ERR_CONFIG,
        "fault_reject_bad_type_target_pair");
    return failures;
}

/** @brief 运行环境基础模型单元测试。 */
int main(void)
{
    int failures = 0;

    failures += test_geo_round_trip();
    failures += test_local_frames();
    failures += test_terrain();
    failures += test_tile_file_round_trip();
    failures += test_line_of_sight();
    failures += test_actuator();
    failures += test_force_models();
    failures += test_environment_force_model();
    failures += test_missile_plant();
    failures += test_sensor_noise();
    failures += test_vector_sensor_pipeline();
    failures += test_seeker_sensor_pipeline();
    failures += test_fault_injection();
    failures += test_fault_injection_rejects_bad_config();
    return failures == 0 ? 0 : 1;
}
