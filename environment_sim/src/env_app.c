/** @file env_app.c
 *  @brief 环境仿真主循环实现。
 *
 *  该模块维护真实世界状态，推进导弹和目标运动，并在每个仿真步向飞控
 *  提供带噪声的传感器数据。
 */
#include "env/env_app.h"

#include "common/config.h"
#include "common/build_info.h"
#include "common/logger.h"
#include "common/math_constants.h"
#include "common/packet.h"
#include "common/protocol.h"
#include "common/status.h"
#include "common/vec3.h"
#include "env/actuator_model.h"
#include "env/earth_model.h"
#include "env/environment_force_model.h"
#include "env/fault_injection.h"
#include "env/geo_coordinate.h"
#include "env/mass_model.h"
#include "env/missile_plant_6dof.h"
#include "env/sensor_accel.h"
#include "env/sensor_imu.h"
#include "env/sensor_seeker.h"
#include "env/sensor_speed.h"
#include "env/terrain_model.h"

#include <arpa/inet.h>
#include <errno.h>
#include <math.h>
#include <netinet/in.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <unistd.h>

#define ENV_PACKET_BUFFER_SIZE 1024u
typedef struct EnvRuntimeConfig {
    /** @brief 环境进程 UDP 基础端口。 */
    unsigned int env_base_port;
    /** @brief 飞控进程 UDP 基础端口。 */
    unsigned int fc_base_port;
    /** @brief 飞控目标 IPv4 地址。 */
    char host[64];
    /** @brief 本次任务输出根目录。 */
    char output_dir[256];
    /** @brief 实例目录模板；当前版本保留字段，目录仍使用固定编号格式。 */
    char instance_dir_template[64];
    /** @brief 日志定期刷新步数。 */
    unsigned int flush_every_steps;
    /** @brief 是否输出传感器和指令二进制报文。 */
    int binary_logs;
    /** @brief 是否输出文本事件日志。 */
    int event_log;
    /** @brief 本次任务的随机种子基值。 */
    uint64_t base_random_seed;
} EnvRuntimeConfig;

typedef struct EnvScenarioConfig {
    /** @brief 固定仿真步长，单位秒。 */
    double dt;
    /** @brief 最大仿真时长，单位秒。 */
    double max_time;
    /** @brief 命中判定距离，单位米。 */
    double hit_radius_m;
    /** @brief 当前质点模型的加速度一阶响应时间常数。 */
    double command_tau_s;
    /** @brief 六自由度刚体质量，单位千克。 */
    double mass_kg;
    /** @brief 初始推进剂质量，单位千克。 */
    double propellant_mass_kg;
    /** @brief 机体系惯量矩阵对角线，单位 kg*m^2。 */
    double inertia_diag[3];
    /** @brief 飞控加速度接口的幅值限制，单位 m/s^2。 */
    double acceleration_limit_mps2;
    /** @brief 飞控加速度接口的变化率限制，单位 m/s^3。 */
    double acceleration_rate_limit_mps3;
    /** @brief 六自由度状态积分算法。 */
    IntegratorType integrator;
    /** @brief 是否启用 ECEF 地球自转修正项。 */
    int enable_earth_rotation;
    /** @brief 环境力、力矩和重力模型配置快照。 */
    EnvironmentForceModel force_model;
    /** @brief IMU 三轴陀螺仪配置。 */
    ImuSensorConfig imu_config;
    /** @brief ECEF 三轴加速度计配置。 */
    AccelSensorConfig accel_config;
    /** @brief ECEF 三轴速度计配置。 */
    SpeedSensorConfig speed_config;
    /** @brief 导引头相对测量配置。 */
    SeekerSensorConfig seeker_config;
    /** @brief 是否查询地形高程和地表碰撞。 */
    int terrain_enabled;
    /** @brief 是否对弹目视线执行地形遮挡采样。 */
    int los_occlusion_enabled;
    /** @brief 无瓦片覆盖时采用的处理策略。 */
    TerrainMissingPolicy terrain_missing_policy;
    /** @brief 平坦填充策略的椭球高，单位米。 */
    double terrain_flat_fill_height_m;
    /** @brief 导弹初始纬度、经度和椭球高，单位度、度、米。 */
    double missile_lla[3];
    /** @brief 导弹初始 ECEF 速度，单位 m/s。 */
    double missile_vel[3];
    /** @brief 目标初始纬度、经度和椭球高，单位度、度、米。 */
    double target_lla[3];
    /** @brief 目标初始 ECEF 速度，单位 m/s。 */
    double target_vel[3];
} EnvScenarioConfig;

typedef struct EnvTruthState {
    /** @brief 当前仿真时间，单位秒。 */
    double time;
    /** @brief 导弹 ECEF 位置，单位米。 */
    Vec3 missile_pos;
    /** @brief 导弹 ECEF 速度，单位 m/s。 */
    Vec3 missile_vel;
    /** @brief 导弹 ECEF 加速度，单位 m/s^2。 */
    Vec3 missile_accel;
    /** @brief 一阶执行机构响应后的实际 ECEF 加速度。 */
    Vec3 missile_actual_accel;
    /** @brief 六自由度导弹刚体状态。 */
    PlantState6Dof missile_plant;
    /** @brief 导弹干质量和推进剂质量状态。 */
    MassModel missile_mass;
    /** @brief 初始惯量矩阵，用于按质量比例近似更新惯量。 */
    Matrix3 initial_inertia_b;
    /** @brief 初始总质量，单位千克。 */
    double initial_mass_kg;
    /** @brief ECEF 三轴加速度级虚拟执行机构。 */
    ActuatorState acceleration_actuators[3];
    /** @brief 目标 ECEF 位置，单位米。 */
    Vec3 target_pos;
    /** @brief 目标 ECEF 速度，单位 m/s。 */
    Vec3 target_vel;
    /** @brief 导弹当前大地经纬高。 */
    LlaCoord missile_lla;
    /** @brief 目标当前大地经纬高。 */
    LlaCoord target_lla;
    /** @brief 导弹当前离地高度，单位米。 */
    double missile_agl_m;
    /** @brief 仿真期间最小弹目距离，单位米。 */
    double min_range;
    /** @brief 达到最小距离的仿真时间，单位秒。 */
    double time_of_closest;
} EnvTruthState;

/** @brief 单个仿真实例拥有的全部传感器运行状态。 */
typedef struct EnvSensorState {
    ImuSensor imu;
    AccelSensor accelerometer;
    SpeedSensor speedometer;
    SeekerSensor seeker;
    uint64_t instance_random_seed;
} EnvSensorState;

/** @brief 单实例故障脚本运行统计，用于 summary.json 和批量汇总。 */
typedef struct FaultRunStats {
    /** @brief 配置中启用和加载的故障定义数量。 */
    size_t configured_fault_count;
    /** @brief 仿真期间故障进入激活窗口的次数。 */
    size_t fault_start_count;
    /** @brief 仿真期间故障离开激活窗口的次数。 */
    size_t fault_end_count;
    /** @brief 至少一个故障处于激活状态的仿真步数。 */
    size_t active_step_count;
    /** @brief 传感器测量或有效位被故障影响的仿真步数。 */
    size_t sensor_affected_step_count;
    /** @brief 虚拟执行机构命令或状态被故障影响的仿真步数。 */
    size_t actuator_affected_step_count;
    /** @brief 同一仿真步内最大并发激活故障数量。 */
    size_t max_concurrent_active;
} FaultRunStats;

/** @brief 从运行时配置读取网络和输出目录。 */
static SimStatus load_runtime_config(const ConfigTree *runtime, EnvRuntimeConfig *out)
{
    SimStatus status;

    if (runtime == 0 || out == 0) {
        return SIM_ERR_INVALID_ARG;
    }
    memset(out, 0, sizeof(*out));

    status = config_get_uint32(runtime, "network.environment_base_port", &out->env_base_port);
    if (status != SIM_OK) {
        return status;
    }
    status = config_get_uint32(runtime, "network.flight_control_base_port", &out->fc_base_port);
    if (status != SIM_OK) {
        return status;
    }
    status = config_get_string(runtime, "network.host", out->host, sizeof(out->host));
    if (status != SIM_OK) {
        return status;
    }
    status = config_get_string(runtime, "logging.output_dir", out->output_dir, sizeof(out->output_dir));
    if (status != SIM_OK) {
        return status;
    }
    status = config_get_string(
        runtime,
        "logging.instance_dir_template",
        out->instance_dir_template,
        sizeof(out->instance_dir_template));
    if (status != SIM_OK) {
        return status;
    }
    status = config_get_uint32(runtime, "logging.flush_every_steps", &out->flush_every_steps);
    if (status != SIM_OK) {
        out->flush_every_steps = 100u;
    }
    status = config_get_bool(runtime, "logging.binary_logs", &out->binary_logs);
    if (status != SIM_OK) {
        out->binary_logs = 1;
    }
    status = config_get_bool(runtime, "logging.event_log", &out->event_log);
    if (status != SIM_OK) {
        out->event_log = 1;
    }
    {
        unsigned int base_seed;

        status = config_get_uint32(runtime, "campaign.base_random_seed", &base_seed);
        out->base_random_seed = status == SIM_OK ? (uint64_t)base_seed : UINT64_C(1);
    }
    return SIM_OK;
}

/** @brief 返回给定量程的无误差标量传感器配置。 */
static SensorNoiseConfig make_noise_config(double minimum, double maximum)
{
    SensorNoiseConfig config;

    memset(&config, 0, sizeof(config));
    config.min_value = minimum;
    config.max_value = maximum;
    return config;
}

/** @brief 读取通用标量噪声对象；缺省字段保留调用方默认值。 */
static SimStatus load_scalar_noise_config(
    const ConfigTree *config,
    const char *prefix,
    SensorNoiseConfig *out)
{
    char path[192];
    double value;

    if (config == 0 || prefix == 0 || out == 0) {
        return SIM_ERR_INVALID_ARG;
    }
#define LOAD_NOISE_FIELD(json_name, member) \
    do { \
        (void)snprintf(path, sizeof(path), "%s.%s", prefix, json_name); \
        if (config_get_double(config, path, &value) == SIM_OK) { \
            out->member = value; \
        } \
    } while (0)
    LOAD_NOISE_FIELD("bias", bias);
    LOAD_NOISE_FIELD("white_noise_std", white_noise_std);
    LOAD_NOISE_FIELD("random_walk_std", random_walk_std);
    LOAD_NOISE_FIELD("min_value", min_value);
    LOAD_NOISE_FIELD("max_value", max_value);
    LOAD_NOISE_FIELD("resolution", resolution);
#undef LOAD_NOISE_FIELD
    out->dropout_probability = 0.0;
    return sensor_noise_validate(out);
}

/** @brief 读取三轴共用噪声幅值和独立轴偏置。 */
static SimStatus load_vector_noise_config(
    const ConfigTree *config,
    const char *prefix,
    SensorNoiseConfig axis[3],
    double minimum,
    double maximum)
{
    char path[192];
    double bias[3] = { 0.0, 0.0, 0.0 };
    SensorNoiseConfig common = make_noise_config(minimum, maximum);
    size_t index;
    SimStatus status;

    status = load_scalar_noise_config(config, prefix, &common);
    if (status != SIM_OK) {
        return status;
    }
    (void)snprintf(path, sizeof(path), "%s.bias_xyz", prefix);
    (void)config_get_double_array(config, path, bias, 3u);
    for (index = 0u; index < 3u; ++index) {
        axis[index] = common;
        axis[index].bias = bias[index];
    }
    return SIM_OK;
}

/** @brief 读取三轴传感器通用采样配置和误差对象。 */
static SimStatus load_vector_sensor_config(
    const ConfigTree *config,
    const char *prefix,
    double simulation_dt_s,
    double minimum,
    double maximum,
    SensorVector3Config *out)
{
    char path[192];
    SimStatus status;

    if (config == 0 || prefix == 0 || out == 0) {
        return SIM_ERR_INVALID_ARG;
    }
    memset(out, 0, sizeof(*out));
    out->sample_period_s = simulation_dt_s;
    (void)snprintf(path, sizeof(path), "%s.enabled", prefix);
    status = config_get_bool(config, path, &out->enabled);
    if (status != SIM_OK) {
        return status;
    }
    (void)snprintf(path, sizeof(path), "%s.sample_period_s", prefix);
    (void)config_get_double(config, path, &out->sample_period_s);
    (void)snprintf(path, sizeof(path), "%s.delay_s", prefix);
    (void)config_get_double(config, path, &out->delay_s);
    (void)snprintf(path, sizeof(path), "%s.dropout_probability", prefix);
    (void)config_get_double(config, path, &out->dropout_probability);
    (void)snprintf(path, sizeof(path), "%s.noise", prefix);
    status = load_vector_noise_config(
        config,
        path,
        out->axis,
        minimum,
        maximum);
    if (status != SIM_OK) {
        return status;
    }
    return sensor_vector3_validate(out, simulation_dt_s);
}

/** @brief 读取导引头采样配置及距离、LOS 和闭合速度误差。 */
static SimStatus load_seeker_config(
    const ConfigTree *config,
    double simulation_dt_s,
    SeekerSensorConfig *out)
{
    SimStatus status;

    if (config == 0 || out == 0) {
        return SIM_ERR_INVALID_ARG;
    }
    memset(out, 0, sizeof(*out));
    out->sample_period_s = simulation_dt_s;
    status = config_get_bool(config, "sensors.seeker.enabled", &out->enabled);
    if (status != SIM_OK) {
        return status;
    }
    (void)config_get_double(
        config,
        "sensors.seeker.sample_period_s",
        &out->sample_period_s);
    (void)config_get_double(config, "sensors.seeker.delay_s", &out->delay_s);
    (void)config_get_double(
        config,
        "sensors.seeker.dropout_probability",
        &out->dropout_probability);

    out->range = make_noise_config(0.0, 1.0e9);
    out->closing_velocity = make_noise_config(-1.0e5, 1.0e5);
    status = load_scalar_noise_config(
        config,
        "sensors.seeker.range_noise",
        &out->range);
    if (status == SIM_OK) {
        status = load_vector_noise_config(
            config,
            "sensors.seeker.los_unit_noise",
            out->los_unit_axis,
            -2.0,
            2.0);
    }
    if (status == SIM_OK) {
        status = load_vector_noise_config(
            config,
            "sensors.seeker.los_rate_noise",
            out->los_rate_axis,
            -100.0,
            100.0);
    }
    if (status == SIM_OK) {
        status = load_scalar_noise_config(
            config,
            "sensors.seeker.closing_velocity_noise",
            &out->closing_velocity);
    }
    if (status != SIM_OK) {
        return status;
    }
    return sensor_seeker_validate(out, simulation_dt_s);
}

/** @brief 从场景配置读取动力学和初始条件。 */
static SimStatus load_scenario_config(const ConfigTree *scenario, EnvScenarioConfig *out)
{
    double vector_values[3];
    char integrator[16];
    char missing_policy[32];
    SimStatus status;

    if (scenario == 0 || out == 0) {
        return SIM_ERR_INVALID_ARG;
    }
    memset(out, 0, sizeof(*out));

    status = config_get_double(scenario, "simulation.dt", &out->dt);
    if (status != SIM_OK) {
        return status;
    }
    status = config_get_double(scenario, "simulation.max_time", &out->max_time);
    if (status != SIM_OK) {
        return status;
    }
    status = config_get_double(scenario, "simulation.hit_radius_m", &out->hit_radius_m);
    if (status != SIM_OK) {
        out->hit_radius_m = 5.0;
    }
    status = config_get_double(scenario, "plant.command_tau_s", &out->command_tau_s);
    if (status != SIM_OK) {
        out->command_tau_s = 0.25;
    }
    status = config_get_double(scenario, "plant.mass_kg", &out->mass_kg);
    if (status != SIM_OK || out->mass_kg <= 0.0) {
        return SIM_ERR_CONFIG;
    }
    status = config_get_double(
        scenario,
        "plant.propellant_mass_kg",
        &out->propellant_mass_kg);
    if (status != SIM_OK) {
        out->propellant_mass_kg = 0.0;
    }
    if (out->propellant_mass_kg < 0.0 ||
        out->propellant_mass_kg >= out->mass_kg) {
        return SIM_ERR_CONFIG;
    }
    status = config_get_double_array(
        scenario,
        "plant.inertia_diag",
        out->inertia_diag,
        3u);
    if (status != SIM_OK ||
        out->inertia_diag[0] <= 0.0 ||
        out->inertia_diag[1] <= 0.0 ||
        out->inertia_diag[2] <= 0.0) {
        return SIM_ERR_CONFIG;
    }
    status = config_get_double(
        scenario,
        "plant.acceleration_limit_mps2",
        &out->acceleration_limit_mps2);
    if (status != SIM_OK) {
        out->acceleration_limit_mps2 = 500.0;
    }
    status = config_get_double(
        scenario,
        "plant.acceleration_rate_limit_mps3",
        &out->acceleration_rate_limit_mps3);
    if (status != SIM_OK) {
        out->acceleration_rate_limit_mps3 = 2000.0;
    }
    status = config_get_string(scenario, "plant.integrator", integrator, sizeof(integrator));
    if (status != SIM_OK || strcmp(integrator, "RK4") == 0) {
        out->integrator = INTEGRATOR_RK4;
    } else if (strcmp(integrator, "RK2") == 0) {
        out->integrator = INTEGRATOR_RK2;
    } else if (strcmp(integrator, "EULER") == 0) {
        out->integrator = INTEGRATOR_EULER;
    } else {
        return SIM_ERR_CONFIG;
    }
    status = config_get_bool(
        scenario,
        "earth.enable_rotation_terms",
        &out->enable_earth_rotation);
    if (status != SIM_OK) {
        out->enable_earth_rotation = 0;
    }
    out->force_model.gravity = gravity_model_wgs84();
    status = config_get_bool(
        scenario,
        "gravity.enabled",
        &out->force_model.gravity.enabled);
    if (status != SIM_OK) {
        out->force_model.gravity.enabled = 0;
    }
    out->force_model.atmosphere = atmosphere_model_isa();
    status = config_get_bool(
        scenario,
        "atmosphere.enabled",
        &out->force_model.atmosphere.enabled);
    if (status != SIM_OK) {
        out->force_model.atmosphere.enabled = 0;
    }
    status = config_get_double(
        scenario,
        "atmosphere.maximum_model_height_m",
        &out->force_model.atmosphere.maximum_model_height_m);
    if (status != SIM_OK) {
        out->force_model.atmosphere.maximum_model_height_m = 11000.0;
    }
    status = config_get_double_array(
        scenario,
        "atmosphere.wind_velocity_ecef_mps",
        vector_values,
        3u);
    if (status == SIM_OK) {
        out->force_model.wind_velocity_ecef_mps =
            vec3_make(vector_values[0], vector_values[1], vector_values[2]);
    }
    status = config_get_bool(
        scenario,
        "propulsion.enabled",
        &out->force_model.propulsion.enabled);
    if (status != SIM_OK) {
        out->force_model.propulsion.enabled = 0;
    }
    status = config_get_double(
        scenario,
        "propulsion.thrust_n",
        &out->force_model.propulsion.thrust_n);
    if (status != SIM_OK) {
        out->force_model.propulsion.thrust_n = 0.0;
    }
    status = config_get_double(
        scenario,
        "propulsion.mass_flow_kgps",
        &out->force_model.propulsion.mass_flow_kgps);
    if (status != SIM_OK) {
        out->force_model.propulsion.mass_flow_kgps = 0.0;
    }
    status = config_get_double(
        scenario,
        "propulsion.burn_time_s",
        &out->force_model.propulsion.burn_time_s);
    if (status != SIM_OK) {
        out->force_model.propulsion.burn_time_s = 0.0;
    }
    out->force_model.propulsion.thrust_direction_b = vec3_make(1.0, 0.0, 0.0);
    status = config_get_double_array(
        scenario,
        "propulsion.thrust_direction_b",
        vector_values,
        3u);
    if (status == SIM_OK) {
        out->force_model.propulsion.thrust_direction_b =
            vec3_make(vector_values[0], vector_values[1], vector_values[2]);
    }
    status = config_get_bool(
        scenario,
        "aerodynamics.enabled",
        &out->force_model.aerodynamics.enabled);
    if (status != SIM_OK) {
        out->force_model.aerodynamics.enabled = 0;
    }
    status = config_get_double(
        scenario,
        "aerodynamics.reference_area_m2",
        &out->force_model.aerodynamics.reference_area_m2);
    if (status != SIM_OK) {
        out->force_model.aerodynamics.reference_area_m2 = 0.0;
    }
    status = config_get_double(
        scenario,
        "aerodynamics.reference_length_m",
        &out->force_model.aerodynamics.reference_length_m);
    if (status != SIM_OK) {
        out->force_model.aerodynamics.reference_length_m = 0.0;
    }
    status = config_get_double(
        scenario,
        "aerodynamics.drag_coefficient",
        &out->force_model.aerodynamics.drag_coefficient);
    if (status != SIM_OK) {
        out->force_model.aerodynamics.drag_coefficient = 0.0;
    }
    status = config_get_double(
        scenario,
        "aerodynamics.control_force_coefficient",
        &out->force_model.aerodynamics.control_force_coefficient);
    if (status != SIM_OK) {
        out->force_model.aerodynamics.control_force_coefficient = 0.0;
    }
    status = config_get_double(
        scenario,
        "aerodynamics.control_moment_coefficient",
        &out->force_model.aerodynamics.control_moment_coefficient);
    if (status != SIM_OK) {
        out->force_model.aerodynamics.control_moment_coefficient = 0.0;
    }
    out->force_model.enable_earth_rotation = out->enable_earth_rotation;
    out->force_model.earth_rotation_rate_radps =
        ENV_WGS84_EARTH_ROTATION_RADPS;
    status = environment_force_model_validate(&out->force_model);
    if (status != SIM_OK) {
        return SIM_ERR_CONFIG;
    }
    status = load_vector_sensor_config(
        scenario,
        "sensors.imu",
        out->dt,
        -100.0,
        100.0,
        &out->imu_config);
    if (status == SIM_OK) {
        status = load_vector_sensor_config(
            scenario,
            "sensors.accelerometer",
            out->dt,
            -1.0e5,
            1.0e5,
            &out->accel_config);
    }
    if (status == SIM_OK) {
        status = load_vector_sensor_config(
            scenario,
            "sensors.speedometer",
            out->dt,
            -1.0e5,
            1.0e5,
            &out->speed_config);
    }
    if (status == SIM_OK) {
        status = load_seeker_config(
            scenario,
            out->dt,
            &out->seeker_config);
    }
    if (status != SIM_OK) {
        return SIM_ERR_CONFIG;
    }
    status = config_get_bool(scenario, "map.enable_terrain", &out->terrain_enabled);
    if (status != SIM_OK) {
        out->terrain_enabled = 0;
    }
    status = config_get_bool(scenario, "map.enable_los_occlusion", &out->los_occlusion_enabled);
    if (status != SIM_OK) {
        out->los_occlusion_enabled = 0;
    }
    status = config_get_string(
        scenario,
        "map.missing_tile_policy",
        missing_policy,
        sizeof(missing_policy));
    if (status != SIM_OK || strcmp(missing_policy, "FLAT_FILL") == 0) {
        out->terrain_missing_policy = MAP_MISSING_FLAT_FILL;
    } else if (strcmp(missing_policy, "ERROR") == 0) {
        out->terrain_missing_policy = MAP_MISSING_ERROR;
    } else if (strcmp(missing_policy, "NEAREST") == 0) {
        out->terrain_missing_policy = MAP_MISSING_NEAREST;
    } else {
        return SIM_ERR_CONFIG;
    }
    status = config_get_double(
        scenario,
        "map.terrain.flat_fill_height_m",
        &out->terrain_flat_fill_height_m);
    if (status != SIM_OK) {
        out->terrain_flat_fill_height_m = 0.0;
    }
    status = config_get_double_array(scenario, "missile.initial_lla_deg_m", out->missile_lla, 3u);
    if (status != SIM_OK) {
        return status;
    }
    status = config_get_double_array(scenario, "missile.initial_velocity_ecef_mps", out->missile_vel, 3u);
    if (status != SIM_OK) {
        return status;
    }
    status = config_get_double_array(scenario, "target.initial_lla_deg_m", out->target_lla, 3u);
    if (status != SIM_OK) {
        return status;
    }
    status = config_get_double_array(scenario, "target.initial_velocity_ecef_mps", out->target_vel, 3u);
    return status;
}

/** @brief 将配置中的度、度、米三元组转换为 LLA 结构。 */
static LlaCoord lla_deg_m_from_array(const double values[3])
{
    LlaCoord result;

    result.lat_rad = values[0] * SIM_DEG_TO_RAD;
    result.lon_rad = values[1] * SIM_DEG_TO_RAD;
    result.height_m = values[2];
    return result;
}

/** @brief 绑定环境侧 UDP 套接字。 */
static SimStatus bind_udp_socket(unsigned int port, int *sock_out)
{
    int sock;
    struct sockaddr_in addr;
    struct timeval timeout;

    if (sock_out == 0 || port > 65535u) {
        return SIM_ERR_INVALID_ARG;
    }
    sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
        return SIM_ERR_IO;
    }

    {
        int reuse = 1;
        (void)setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
    }
    timeout.tv_sec = 1;
    timeout.tv_usec = 0;
    (void)setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));

    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons((uint16_t)port);
    if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) != 0) {
        (void)close(sock);
        return SIM_ERR_IO;
    }
    *sock_out = sock;
    return SIM_OK;
}

/** @brief 组帧并发送传感器数据。 */
static SimStatus send_sensor_frame(
    int sock,
    const struct sockaddr_in *peer,
    uint32_t instance_id,
    const SensorFrame *sensor)
{
    unsigned char buffer[SIM_SENSOR_PACKET_WIRE_SIZE];
    size_t packet_size;
    SimStatus status;
    ssize_t sent;

    if (peer == 0 || sensor == 0) {
        return SIM_ERR_INVALID_ARG;
    }
    status = packet_encode_sensor_frame(
        instance_id,
        sensor,
        buffer,
        sizeof(buffer),
        &packet_size);
    if (status != SIM_OK) {
        return status;
    }
    sent = sendto(sock, buffer, packet_size, 0, (const struct sockaddr *)peer, sizeof(*peer));
    if (sent != (ssize_t)packet_size) {
        return SIM_ERR_IO;
    }
    return SIM_OK;
}

/** @brief 接收并校验飞控控制指令。 */
static SimStatus receive_control_command(
    int sock,
    uint32_t instance_id,
    ControlCommand *command)
{
    unsigned char buffer[ENV_PACKET_BUFFER_SIZE];
    struct sockaddr_in from;
    socklen_t from_len = sizeof(from);
    ssize_t got = recvfrom(sock, buffer, sizeof(buffer), 0, (struct sockaddr *)&from, &from_len);

    if (command == 0) {
        return SIM_ERR_INVALID_ARG;
    }
    if (got < 0) {
        return errno == EAGAIN || errno == EWOULDBLOCK ? SIM_ERR_TIMEOUT : SIM_ERR_IO;
    }
    return packet_decode_control_command(buffer, (size_t)got, instance_id, command);
}

/** @brief 初始化单实例四类传感器及相互独立的确定性随机流。 */
static SimStatus init_sensor_state(
    EnvSensorState *sensors,
    const EnvScenarioConfig *scenario,
    uint64_t instance_random_seed)
{
    SimStatus status;

    if (sensors == 0 || scenario == 0) {
        return SIM_ERR_INVALID_ARG;
    }
    memset(sensors, 0, sizeof(*sensors));
    sensors->instance_random_seed = instance_random_seed;
    status = sensor_imu_init(
        &sensors->imu,
        &scenario->imu_config,
        instance_random_seed ^ UINT64_C(0x494D5501),
        scenario->dt);
    if (status == SIM_OK) {
        status = sensor_accel_init(
            &sensors->accelerometer,
            &scenario->accel_config,
            instance_random_seed ^ UINT64_C(0x41434301),
            scenario->dt);
    }
    if (status == SIM_OK) {
        status = sensor_speed_init(
            &sensors->speedometer,
            &scenario->speed_config,
            instance_random_seed ^ UINT64_C(0x53504401),
            scenario->dt);
    }
    if (status == SIM_OK) {
        status = sensor_seeker_init(
            &sensors->seeker,
            &scenario->seeker_config,
            instance_random_seed ^ UINT64_C(0x53454B01),
            scenario->dt);
    }
    return status;
}

/** @brief 将单个传感器状态映射到协议有效位和故障位。 */
static void apply_sample_status(
    SensorFrame *sensor,
    const SensorSampleStatus *sample_status,
    uint32_t valid_flag,
    uint32_t dropout_fault)
{
    if (sample_status->valid != 0) {
        sensor->sensor_valid_flags |= valid_flag;
    }
    if (sample_status->dropped != 0) {
        sensor->sensor_fault_flags |= dropout_fault;
    }
    if (sample_status->delay_warmup != 0) {
        sensor->sensor_fault_flags |= SIM_SENSOR_FAULT_DELAY_WARMUP;
    }
}

/** @brief 从环境真值生成带误差、采样保持、延迟和丢包的传感器帧。 */
static SimStatus build_sensor_frame(
    const EnvTruthState *state,
    EnvSensorState *sensors,
    double dt,
    uint32_t seq,
    SensorFrame *sensor)
{
    SeekerTruth seeker_truth;
    SeekerMeasurement seeker_measurement;
    SensorSampleStatus imu_status;
    SensorSampleStatus accel_status;
    SensorSampleStatus speed_status;
    SensorSampleStatus seeker_status;
    SimStatus status;

    if (state == 0 || sensors == 0 || sensor == 0) {
        return SIM_ERR_INVALID_ARG;
    }
    memset(sensor, 0, sizeof(*sensor));
    sensor->seq = seq;
    sensor->sim_time = state->time;
    sensor->dt = dt;
    sensor->missile_lat_rad_meas = state->missile_lla.lat_rad;
    sensor->missile_lon_rad_meas = state->missile_lla.lon_rad;
    sensor->missile_height_m_meas = state->missile_lla.height_m;
    sensor->missile_height_agl_m_meas = state->missile_agl_m;
    sensor->sensor_valid_flags |= SIM_SENSOR_VALID_GEODETIC;

    status = sensor_imu_update(
        &sensors->imu,
        state->time,
        dt,
        state->missile_plant.omega_b,
        &sensor->missile_gyro_b_meas,
        &imu_status);
    if (status == SIM_OK) {
        status = sensor_accel_update(
            &sensors->accelerometer,
            state->time,
            dt,
            state->missile_accel,
            &sensor->missile_accel_ecef_meas,
            &accel_status);
    }
    if (status == SIM_OK) {
        status = sensor_speed_update(
            &sensors->speedometer,
            state->time,
            dt,
            state->missile_vel,
            &sensor->missile_vel_ecef_meas,
            &speed_status);
    }
    if (status != SIM_OK) {
        return status;
    }

    seeker_truth.missile_position_ecef_m = state->missile_pos;
    seeker_truth.missile_velocity_ecef_mps = state->missile_vel;
    seeker_truth.target_position_ecef_m = state->target_pos;
    seeker_truth.target_velocity_ecef_mps = state->target_vel;
    status = sensor_seeker_update(
        &sensors->seeker,
        state->time,
        dt,
        &seeker_truth,
        &seeker_measurement,
        &seeker_status);
    if (status != SIM_OK) {
        return status;
    }
    sensor->target_range_meas = seeker_measurement.range_m;
    sensor->target_los_unit_ecef_meas = seeker_measurement.los_unit_ecef;
    sensor->target_los_rate_ecef_meas = seeker_measurement.los_rate_ecef_radps;
    sensor->target_closing_velocity_meas =
        seeker_measurement.closing_velocity_mps;

    apply_sample_status(
        sensor,
        &imu_status,
        SIM_SENSOR_VALID_IMU_GYRO,
        SIM_SENSOR_FAULT_IMU_DROPOUT);
    apply_sample_status(
        sensor,
        &accel_status,
        SIM_SENSOR_VALID_ACCEL,
        SIM_SENSOR_FAULT_ACCEL_DROPOUT);
    apply_sample_status(
        sensor,
        &speed_status,
        SIM_SENSOR_VALID_SPEED,
        SIM_SENSOR_FAULT_SPEED_DROPOUT);
    apply_sample_status(
        sensor,
        &seeker_status,
        SIM_SENSOR_VALID_SEEKER,
        SIM_SENSOR_FAULT_SEEKER_DROPOUT);
    return SIM_OK;
}

/** @brief 按统一环境力链推进六自由度真值状态。
 *
 *  飞控 ECEF 加速度指令先经过三个虚拟执行机构，再由环境力模型转换为
 *  等效机体系力，并与气动力、推进力、气动力矩和重力组合。积分完成后
 *  消耗推进剂，并按总质量比例近似更新惯量矩阵。
 */
static SimStatus update_truth(
    EnvTruthState *state,
    const ControlCommand *command,
    const EnvScenarioConfig *cfg,
    const EnvironmentForceModel *force_model,
    const FaultStepEffects *fault_effects)
{
    EnvironmentForceInput force_input;
    EnvironmentForceOutput force_output;
    double commands[3] = {
        command->accel_cmd_ecef.x,
        command->accel_cmd_ecef.y,
        command->accel_cmd_ecef.z
    };
    double actual[3];
    size_t index;
    SimStatus status = SIM_OK;

    if (state == 0 || command == 0 || cfg == 0 || force_model == 0) {
        return SIM_ERR_INVALID_ARG;
    }
    if (fault_effects != 0) {
        fault_injection_apply_actuators(
            fault_effects,
            state->acceleration_actuators,
            commands);
    }
    for (index = 0u; index < 3u && status == SIM_OK; ++index) {
        status = actuator_model_step(
            &state->acceleration_actuators[index],
            commands[index],
            cfg->dt);
        actual[index] = state->acceleration_actuators[index].pos;
    }
    if (status != SIM_OK) {
        return status;
    }
    state->missile_actual_accel = vec3_make(actual[0], actual[1], actual[2]);
    memset(&force_input, 0, sizeof(force_input));
    force_input.plant = &state->missile_plant;
    force_input.height_m = state->missile_lla.height_m;
    force_input.virtual_acceleration_ecef_mps2 = state->missile_actual_accel;
    force_input.propellant_mass_kg = state->missile_mass.propellant_mass_kg;
    force_input.dt_s = cfg->dt;
    status = environment_force_model_evaluate(
        force_model,
        &force_input,
        &force_output);
    if (status != SIM_OK) {
        return status;
    }
    status = missile_plant_step(
        &state->missile_plant,
        &force_output.plant_input,
        cfg->dt,
        cfg->integrator);
    if (status != SIM_OK) {
        return status;
    }
    status = mass_model_step(
        &state->missile_mass,
        force_output.mass_flow_kgps,
        cfg->dt);
    if (status != SIM_OK) {
        return status;
    }
    state->missile_plant.mass = state->missile_mass.mass_kg;
    if (state->initial_mass_kg > 0.0) {
        const double inertia_scale =
            state->missile_mass.mass_kg / state->initial_mass_kg;
        size_t row;
        size_t column;

        for (row = 0u; row < 3u; ++row) {
            for (column = 0u; column < 3u; ++column) {
                state->missile_plant.inertia_b.m[row][column] =
                    state->initial_inertia_b.m[row][column] * inertia_scale;
            }
        }
    }
    for (index = 0u; index < 3u; ++index) {
        state->missile_plant.actuator_pos[index] =
            state->acceleration_actuators[index].pos;
        state->missile_plant.actuator_rate[index] =
            state->acceleration_actuators[index].rate;
    }
    state->missile_pos = state->missile_plant.pos_ecef;
    state->missile_vel = state->missile_plant.vel_ecef;
    state->missile_accel = state->missile_plant.accel_ecef;
    state->target_pos = vec3_add(state->target_pos, vec3_scale(state->target_vel, cfg->dt));
    state->time = state->missile_plant.time;
    return SIM_OK;
}

/** @brief 从 ECEF 真值刷新 LLA 和 AGL 派生状态。 */
static SimStatus update_geodetic_state(
    EnvTruthState *state,
    const EarthModel *earth,
    TerrainModel *terrain)
{
    EcefCoord missile_ecef;
    EcefCoord target_ecef;
    SimStatus status;

    if (state == 0 || earth == 0 || terrain == 0) {
        return SIM_ERR_INVALID_ARG;
    }
    missile_ecef.position_m = state->missile_pos;
    target_ecef.position_m = state->target_pos;
    status = geo_ecef_to_lla(earth, &missile_ecef, &state->missile_lla);
    if (status == SIM_OK) {
        status = geo_ecef_to_lla(earth, &target_ecef, &state->target_lla);
    }
    if (status == SIM_OK) {
        status = terrain_get_agl(terrain, &state->missile_lla, &state->missile_agl_m);
    }
    return status;
}

/** @brief 创建任务根目录和实例独立输出目录。 */
static SimStatus make_run_dirs(const char *base_dir, uint32_t instance_id, char *instance_dir, size_t instance_dir_size)
{
    int written;

    if (base_dir == 0 || instance_dir == 0 || instance_dir_size == 0u) {
        return SIM_ERR_INVALID_ARG;
    }
    (void)mkdir("runs", 0777);
    if (mkdir(base_dir, 0777) != 0 && errno != EEXIST) {
        return SIM_ERR_IO;
    }
    written = snprintf(instance_dir, instance_dir_size, "%s/instance_%04u", base_dir, instance_id);
    if (written < 0 || (size_t)written >= instance_dir_size) {
        return SIM_ERR_OUT_OF_RANGE;
    }
    if (mkdir(instance_dir, 0777) != 0 && errno != EEXIST) {
        return SIM_ERR_IO;
    }
    return SIM_OK;
}

/** @brief 写出可复现实例所需的软件、配置、端口和时间参数。 */
static SimStatus write_run_manifest(
    const char *instance_dir,
    const EnvContext *ctx,
    const EnvScenarioConfig *scenario,
    const EnvRuntimeConfig *runtime,
    unsigned int env_port,
    unsigned int fc_port)
{
    char path[1024];
    FILE *file;

    (void)snprintf(path, sizeof(path), "%s/run_manifest.json", instance_dir);
    file = fopen(path, "wb");
    if (file == 0) {
        return SIM_ERR_IO;
    }
    (void)fprintf(file, "{\n");
    (void)fprintf(file, "  \"schema_version\": 1,\n");
    (void)fprintf(file, "  \"instance_id\": %u,\n", ctx->instance_id);
    (void)fprintf(file, "  \"software_version\": \"%d.%d.%d\",\n",
        MISSILE_SIM_VERSION_MAJOR,
        MISSILE_SIM_VERSION_MINOR,
        MISSILE_SIM_VERSION_PATCH);
    (void)fprintf(file, "  \"protocol_version\": \"%d.%d\",\n",
        MISSILE_SIM_PROTOCOL_VERSION_MAJOR,
        MISSILE_SIM_PROTOCOL_VERSION_MINOR);
    (void)fprintf(file, "  \"scenario_path\": \"%s\",\n", ctx->scenario_path);
    (void)fprintf(file, "  \"runtime_path\": \"%s\",\n", ctx->runtime_path);
    (void)fprintf(file, "  \"faults_path\": \"%s\",\n", ctx->faults_path);
    (void)fprintf(file, "  \"dt_s\": %.17g,\n", scenario->dt);
    (void)fprintf(file, "  \"max_time_s\": %.17g,\n", scenario->max_time);
    (void)fprintf(file, "  \"environment_port\": %u,\n", env_port);
    (void)fprintf(file, "  \"flight_control_port\": %u,\n", fc_port);
    (void)fprintf(
        file,
        "  \"random_seed\": %llu,\n",
        (unsigned long long)(runtime->base_random_seed + ctx->instance_id));
    (void)fprintf(file, "  \"initial_mass_kg\": %.17g,\n", scenario->mass_kg);
    (void)fprintf(
        file,
        "  \"initial_propellant_mass_kg\": %.17g,\n",
        scenario->propellant_mass_kg);
    (void)fprintf(
        file,
        "  \"gravity_enabled\": %s,\n",
        scenario->force_model.gravity.enabled != 0 ? "true" : "false");
    (void)fprintf(
        file,
        "  \"atmosphere_enabled\": %s,\n",
        scenario->force_model.atmosphere.enabled != 0 ? "true" : "false");
    (void)fprintf(
        file,
        "  \"aerodynamics_enabled\": %s,\n",
        scenario->force_model.aerodynamics.enabled != 0 ? "true" : "false");
    (void)fprintf(
        file,
        "  \"propulsion_enabled\": %s,\n",
        scenario->force_model.propulsion.enabled != 0 ? "true" : "false");
    (void)fprintf(
        file,
        "  \"earth_rotation_enabled\": %s,\n",
        scenario->force_model.enable_earth_rotation != 0 ? "true" : "false");
    (void)fprintf(file, "  \"binary_logs\": %s,\n", runtime->binary_logs != 0 ? "true" : "false");
    (void)fprintf(file, "  \"event_log\": %s\n", runtime->event_log != 0 ? "true" : "false");
    (void)fprintf(file, "}\n");
    if (fclose(file) != 0) {
        return SIM_ERR_IO;
    }
    return SIM_OK;
}

/** @brief 以单行稳定格式追加一个仿真事件。 */
static void write_event(FILE *file, double sim_time, const char *event, const char *detail)
{
    if (file != 0) {
        (void)fprintf(
            file,
            "%.6f level=INFO event=%s detail=%s\n",
            sim_time,
            event,
            detail == 0 ? "-" : detail);
    }
}

/** @brief 将完整线格式传感器报文写入二进制日志。 */
static SimStatus write_sensor_log(FILE *file, uint32_t instance_id, const SensorFrame *sensor)
{
    unsigned char packet[SIM_SENSOR_PACKET_WIRE_SIZE];
    size_t packet_size;
    SimStatus status;

    if (file == 0) {
        return SIM_OK;
    }
    status = packet_encode_sensor_frame(
        instance_id,
        sensor,
        packet,
        sizeof(packet),
        &packet_size);
    if (status != SIM_OK) {
        return status;
    }
    return fwrite(packet, 1u, packet_size, file) == packet_size ? SIM_OK : SIM_ERR_IO;
}

/** @brief 将完整线格式控制报文写入二进制日志。 */
static SimStatus write_command_log(FILE *file, uint32_t instance_id, const ControlCommand *command)
{
    unsigned char packet[SIM_CONTROL_PACKET_WIRE_SIZE];
    size_t packet_size;
    SimStatus status;

    if (file == 0) {
        return SIM_OK;
    }
    status = packet_encode_control_command(
        instance_id,
        command,
        packet,
        sizeof(packet),
        &packet_size);
    if (status != SIM_OK) {
        return status;
    }
    return fwrite(packet, 1u, packet_size, file) == packet_size ? SIM_OK : SIM_ERR_IO;
}

/** @brief 判断当前步故障效果是否改变了传感器输出或有效位。 */
static int fault_effects_affect_sensor(const FaultStepEffects *effects)
{
    if (effects == 0) {
        return 0;
    }
    return effects->sensor_valid_clear_mask != 0u ||
        effects->sensor_fault_set_mask != 0u ||
        effects->seeker_range_bias_m != 0.0 ||
        vec3_norm(effects->seeker_los_unit_bias) > 0.0 ||
        vec3_norm(effects->seeker_los_rate_bias_radps) > 0.0 ||
        effects->seeker_closing_velocity_bias_mps != 0.0 ||
        vec3_norm(effects->gyro_bias_b_radps) > 0.0 ||
        vec3_norm(effects->accel_bias_ecef_mps2) > 0.0 ||
        vec3_norm(effects->speed_bias_ecef_mps) > 0.0;
}

/** @brief 判断当前步故障效果是否改变了虚拟执行机构命令或状态。 */
static int fault_effects_affect_actuator(const FaultStepEffects *effects)
{
    size_t index;

    if (effects == 0) {
        return 0;
    }
    for (index = 0u; index < 3u; ++index) {
        if (effects->actuator_stuck[index] != 0 ||
            effects->actuator_command_scale[index] != 1.0) {
            return 1;
        }
    }
    return 0;
}

/** @brief 写出单实例终止结果、最近点和故障统计。 */
static void write_summary(
    const char *instance_dir,
    int hit,
    const EnvTruthState *state,
    uint32_t steps,
    const char *exit_reason,
    const FaultRunStats *fault_stats)
{
    char path[1024];
    FILE *file;

    (void)snprintf(path, sizeof(path), "%s/summary.json", instance_dir);
    file = fopen(path, "wb");
    if (file == 0) {
        return;
    }
    (void)fprintf(file, "{\n");
    (void)fprintf(file, "  \"hit_flag\": %s,\n", hit != 0 ? "true" : "false");
    (void)fprintf(file, "  \"miss_distance\": %.6f,\n", state->min_range);
    (void)fprintf(file, "  \"time_of_closest_approach\": %.6f,\n", state->time_of_closest);
    (void)fprintf(file, "  \"simulation_steps\": %u,\n", steps);
    (void)fprintf(file, "  \"exit_reason\": \"%s\",\n", exit_reason);
    if (fault_stats != 0) {
        (void)fprintf(
            file,
            "  \"fault_configured_count\": %lu,\n",
            (unsigned long)fault_stats->configured_fault_count);
        (void)fprintf(
            file,
            "  \"fault_start_count\": %lu,\n",
            (unsigned long)fault_stats->fault_start_count);
        (void)fprintf(
            file,
            "  \"fault_end_count\": %lu,\n",
            (unsigned long)fault_stats->fault_end_count);
        (void)fprintf(
            file,
            "  \"fault_active_step_count\": %lu,\n",
            (unsigned long)fault_stats->active_step_count);
        (void)fprintf(
            file,
            "  \"fault_sensor_affected_step_count\": %lu,\n",
            (unsigned long)fault_stats->sensor_affected_step_count);
        (void)fprintf(
            file,
            "  \"fault_actuator_affected_step_count\": %lu,\n",
            (unsigned long)fault_stats->actuator_affected_step_count);
        (void)fprintf(
            file,
            "  \"fault_max_concurrent_active\": %lu\n",
            (unsigned long)fault_stats->max_concurrent_active);
    } else {
        (void)fprintf(file, "  \"fault_configured_count\": 0,\n");
        (void)fprintf(file, "  \"fault_start_count\": 0,\n");
        (void)fprintf(file, "  \"fault_end_count\": 0,\n");
        (void)fprintf(file, "  \"fault_active_step_count\": 0,\n");
        (void)fprintf(file, "  \"fault_sensor_affected_step_count\": 0,\n");
        (void)fprintf(file, "  \"fault_actuator_affected_step_count\": 0,\n");
        (void)fprintf(file, "  \"fault_max_concurrent_active\": 0\n");
    }
    (void)fprintf(file, "}\n");
    (void)fclose(file);
}

/** @brief 写入轨迹 CSV 的字段名称和单位。 */
static void write_trajectory_header(FILE *file)
{
    (void)fprintf(
        file,
        "time_s,missile_x_ecef_m,missile_y_ecef_m,missile_z_ecef_m,"
        "missile_vx_ecef_mps,missile_vy_ecef_mps,missile_vz_ecef_mps,"
        "missile_ax_ecef_mps2,missile_ay_ecef_mps2,missile_az_ecef_mps2,"
        "missile_lat_deg,missile_lon_deg,missile_height_m,missile_agl_m,"
        "missile_mass_kg,missile_propellant_mass_kg,"
        "force_b_x_n,force_b_y_n,force_b_z_n,"
        "moment_b_x_nm,moment_b_y_nm,moment_b_z_nm,"
        "target_x_ecef_m,target_y_ecef_m,target_z_ecef_m,"
        "target_lat_deg,target_lon_deg,target_height_m,"
        "range_m,closing_velocity_mps\n");
}

/** @brief 写入一个 ECEF 真值轨迹采样点。 */
static void write_trajectory_row(FILE *file, const EnvTruthState *state)
{
    Vec3 r = vec3_sub(state->target_pos, state->missile_pos);
    Vec3 los_unit = vec3_make(1.0, 0.0, 0.0);
    double range = vec3_norm(r);
    double closing_velocity = 0.0;

    if (range > 1.0e-6) {
        (void)vec3_normalize(r, &los_unit);
        closing_velocity = -vec3_dot(los_unit, vec3_sub(state->target_vel, state->missile_vel));
    }

    (void)fprintf(
        file,
        "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,"
        "%.9f,%.9f,%.6f,%.6f,%.6f,%.6f,"
        "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,"
        "%.6f,%.6f,%.6f,%.9f,%.9f,%.6f,%.6f,%.6f\n",
        state->time,
        state->missile_pos.x,
        state->missile_pos.y,
        state->missile_pos.z,
        state->missile_vel.x,
        state->missile_vel.y,
        state->missile_vel.z,
        state->missile_accel.x,
        state->missile_accel.y,
        state->missile_accel.z,
        state->missile_lla.lat_rad * SIM_RAD_TO_DEG,
        state->missile_lla.lon_rad * SIM_RAD_TO_DEG,
        state->missile_lla.height_m,
        state->missile_agl_m,
        state->missile_mass.mass_kg,
        state->missile_mass.propellant_mass_kg,
        state->missile_plant.force_b.x,
        state->missile_plant.force_b.y,
        state->missile_plant.force_b.z,
        state->missile_plant.moment_b.x,
        state->missile_plant.moment_b.y,
        state->missile_plant.moment_b.z,
        state->target_pos.x,
        state->target_pos.y,
        state->target_pos.z,
        state->target_lla.lat_rad * SIM_RAD_TO_DEG,
        state->target_lla.lon_rad * SIM_RAD_TO_DEG,
        state->target_lla.height_m,
        range,
        closing_velocity);
}

SimStatus env_app_run(const EnvContext *ctx)
{
    ConfigTree scenario_tree;
    ConfigTree runtime_tree;
    EnvScenarioConfig scenario;
    EnvRuntimeConfig runtime;
    EnvTruthState state;
    EnvSensorState sensors;
    FaultInjection faults;
    FaultRunStats fault_stats;
    EarthModel earth;
    TerrainModel terrain;
    Logger logger;
    SimStatus status;
    int sock = -1;
    unsigned int env_port;
    unsigned int fc_port;
    struct sockaddr_in fc_addr;
    char instance_dir[512];
    char path[1024];
    FILE *sensor_log = 0;
    FILE *command_log = 0;
    FILE *trajectory_log = 0;
    FILE *event_log = 0;
    uint32_t seq = 0u;
    int hit = 0;
    const char *exit_reason = "timeout";

    if (ctx == 0 || ctx->scenario_path == 0 ||
        ctx->runtime_path == 0 || ctx->faults_path == 0) {
        return SIM_ERR_INVALID_ARG;
    }

    memset(&scenario_tree, 0, sizeof(scenario_tree));
    memset(&runtime_tree, 0, sizeof(runtime_tree));
    memset(&fault_stats, 0, sizeof(fault_stats));
    status = logger_open_stdout(&logger);
    if (status != SIM_OK) {
        return status;
    }
    status = config_load_file(ctx->scenario_path, &scenario_tree);
    if (status != SIM_OK) {
        return status;
    }
    status = config_load_file(ctx->runtime_path, &runtime_tree);
    if (status != SIM_OK) {
        (void)fprintf(stderr, "environment_sim: failed to load %s: %s\n",
            ctx->runtime_path,
            sim_status_to_string(status));
        config_free(&scenario_tree);
        return status;
    }
    status = config_validate_schema(&scenario_tree, 1u);
    if (status == SIM_OK) {
        status = config_require_section(&scenario_tree, "simulation");
    }
    if (status == SIM_OK) {
        status = config_require_section(&scenario_tree, "missile");
    }
    if (status == SIM_OK) {
        status = config_require_section(&scenario_tree, "target");
    }
    if (status == SIM_OK) {
        status = config_require_section(&scenario_tree, "earth");
    }
    if (status == SIM_OK) {
        status = config_require_section(&scenario_tree, "plant");
    }
    if (status == SIM_OK) {
        status = config_require_section(&scenario_tree, "gravity");
    }
    if (status == SIM_OK) {
        status = config_require_section(&scenario_tree, "atmosphere");
    }
    if (status == SIM_OK) {
        status = config_require_section(&scenario_tree, "propulsion");
    }
    if (status == SIM_OK) {
        status = config_require_section(&scenario_tree, "aerodynamics");
    }
    if (status == SIM_OK) {
        status = config_require_section(&scenario_tree, "sensors");
    }
    if (status == SIM_OK) {
        status = config_require_section(&scenario_tree, "sensors.imu");
    }
    if (status == SIM_OK) {
        status = config_require_section(&scenario_tree, "sensors.accelerometer");
    }
    if (status == SIM_OK) {
        status = config_require_section(&scenario_tree, "sensors.speedometer");
    }
    if (status == SIM_OK) {
        status = config_require_section(&scenario_tree, "sensors.seeker");
    }
    if (status == SIM_OK) {
        status = config_require_section(&scenario_tree, "map");
    }
    if (status == SIM_OK) {
        status = config_validate_schema(&runtime_tree, 1u);
    }
    if (status == SIM_OK) {
        status = config_require_section(&runtime_tree, "network");
    }
    if (status == SIM_OK) {
        status = config_require_section(&runtime_tree, "logging");
    }
    if (status == SIM_OK) {
        status = config_require_section(&runtime_tree, "campaign");
    }
    if (status != SIM_OK) {
        (void)fprintf(stderr, "environment_sim: schema validation failed: %s\n",
            sim_status_to_string(status));
        config_free(&scenario_tree);
        config_free(&runtime_tree);
        return status;
    }
    status = load_scenario_config(&scenario_tree, &scenario);
    if (status != SIM_OK) {
        (void)fprintf(stderr, "environment_sim: invalid scenario config: %s\n",
            sim_status_to_string(status));
        config_free(&scenario_tree);
        config_free(&runtime_tree);
        return status;
    }
    status = load_runtime_config(&runtime_tree, &runtime);
    if (status != SIM_OK) {
        (void)fprintf(stderr, "environment_sim: invalid runtime config: %s\n",
            sim_status_to_string(status));
        config_free(&scenario_tree);
        config_free(&runtime_tree);
        return status;
    }
    {
        ConfigTree faults_tree;

        memset(&faults_tree, 0, sizeof(faults_tree));
        status = config_load_file(ctx->faults_path, &faults_tree);
        if (status != SIM_OK) {
            (void)fprintf(stderr, "environment_sim: failed to load %s: %s\n",
                ctx->faults_path,
                sim_status_to_string(status));
            config_free(&scenario_tree);
            config_free(&runtime_tree);
            return status;
        }
        status = fault_injection_load_config(&faults_tree, &faults);
        config_free(&faults_tree);
        if (status != SIM_OK) {
            (void)fprintf(stderr, "environment_sim: invalid faults config: %s\n",
                sim_status_to_string(status));
            config_free(&scenario_tree);
            config_free(&runtime_tree);
            return status;
        }
        fault_stats.configured_fault_count = faults.fault_count;
    }
    status = init_sensor_state(
        &sensors,
        &scenario,
        runtime.base_random_seed + ctx->instance_id);
    if (status != SIM_OK) {
        (void)fprintf(stderr, "environment_sim: invalid sensor config: %s\n",
            sim_status_to_string(status));
        config_free(&scenario_tree);
        config_free(&runtime_tree);
        return status;
    }

    env_port = runtime.env_base_port + (2u * ctx->instance_id);
    fc_port = runtime.fc_base_port + (2u * ctx->instance_id);
    status = bind_udp_socket(env_port, &sock);
    if (status != SIM_OK) {
        (void)fprintf(stderr, "environment_sim: failed to bind UDP port %u: %s\n",
            env_port,
            sim_status_to_string(status));
        config_free(&scenario_tree);
        config_free(&runtime_tree);
        return status;
    }

    memset(&fc_addr, 0, sizeof(fc_addr));
    fc_addr.sin_family = AF_INET;
    fc_addr.sin_port = htons((uint16_t)fc_port);
    if (inet_pton(AF_INET, runtime.host, &fc_addr.sin_addr) != 1) {
        (void)close(sock);
        config_free(&scenario_tree);
        config_free(&runtime_tree);
        return SIM_ERR_CONFIG;
    }

    status = make_run_dirs(runtime.output_dir, ctx->instance_id, instance_dir, sizeof(instance_dir));
    if (status != SIM_OK) {
        (void)close(sock);
        config_free(&scenario_tree);
        config_free(&runtime_tree);
        return status;
    }
    status = write_run_manifest(
        instance_dir,
        ctx,
        &scenario,
        &runtime,
        env_port,
        fc_port);
    if (status != SIM_OK) {
        (void)close(sock);
        config_free(&scenario_tree);
        config_free(&runtime_tree);
        return status;
    }
    if (runtime.binary_logs != 0) {
        (void)snprintf(path, sizeof(path), "%s/sensor_log.bin", instance_dir);
        sensor_log = fopen(path, "wb");
        (void)snprintf(path, sizeof(path), "%s/command_log.bin", instance_dir);
        command_log = fopen(path, "wb");
        if (sensor_log == 0 || command_log == 0) {
            if (sensor_log != 0) {
                (void)fclose(sensor_log);
            }
            if (command_log != 0) {
                (void)fclose(command_log);
            }
            (void)close(sock);
            config_free(&scenario_tree);
            config_free(&runtime_tree);
            return SIM_ERR_IO;
        }
    }
    if (runtime.event_log != 0) {
        (void)snprintf(path, sizeof(path), "%s/event_log.txt", instance_dir);
        event_log = fopen(path, "wb");
        if (event_log == 0) {
            if (sensor_log != 0) {
                (void)fclose(sensor_log);
            }
            if (command_log != 0) {
                (void)fclose(command_log);
            }
            (void)close(sock);
            config_free(&scenario_tree);
            config_free(&runtime_tree);
            return SIM_ERR_IO;
        }
    }
    (void)snprintf(path, sizeof(path), "%s/trajectory.csv", instance_dir);
    trajectory_log = fopen(path, "wb");
    if (trajectory_log == 0) {
        if (sensor_log != 0) {
            (void)fclose(sensor_log);
        }
        if (command_log != 0) {
            (void)fclose(command_log);
        }
        if (event_log != 0) {
            (void)fclose(event_log);
        }
        (void)close(sock);
        config_free(&scenario_tree);
        config_free(&runtime_tree);
        return SIM_ERR_IO;
    }

    earth = earth_model_wgs84();
    status = terrain_model_init(
        &terrain,
        0,
        0u,
        scenario.terrain_missing_policy,
        scenario.terrain_flat_fill_height_m);
    if (status != SIM_OK) {
        (void)fclose(trajectory_log);
        if (sensor_log != 0) {
            (void)fclose(sensor_log);
        }
        if (command_log != 0) {
            (void)fclose(command_log);
        }
        if (event_log != 0) {
            (void)fclose(event_log);
        }
        (void)close(sock);
        config_free(&scenario_tree);
        config_free(&runtime_tree);
        return status;
    }
    terrain.enabled = scenario.terrain_enabled;

    memset(&state, 0, sizeof(state));
    state.missile_lla = lla_deg_m_from_array(scenario.missile_lla);
    state.target_lla = lla_deg_m_from_array(scenario.target_lla);
    {
        EcefCoord missile_ecef;
        EcefCoord target_ecef;

        status = geo_lla_to_ecef(&earth, &state.missile_lla, &missile_ecef);
        if (status == SIM_OK) {
            status = geo_lla_to_ecef(&earth, &state.target_lla, &target_ecef);
        }
        if (status != SIM_OK) {
            (void)fclose(trajectory_log);
            if (sensor_log != 0) {
                (void)fclose(sensor_log);
            }
            if (command_log != 0) {
                (void)fclose(command_log);
            }
            if (event_log != 0) {
                (void)fclose(event_log);
            }
            (void)close(sock);
            config_free(&scenario_tree);
            config_free(&runtime_tree);
            return status;
        }
        state.missile_pos = missile_ecef.position_m;
        state.target_pos = target_ecef.position_m;
    }
    state.missile_vel = vec3_make(scenario.missile_vel[0], scenario.missile_vel[1], scenario.missile_vel[2]);
    state.target_vel = vec3_make(scenario.target_vel[0], scenario.target_vel[1], scenario.target_vel[2]);
    state.missile_plant.time = 0.0;
    state.missile_plant.pos_ecef = state.missile_pos;
    state.missile_plant.vel_ecef = state.missile_vel;
    state.missile_plant.q_bi = quat_identity();
    status = mass_model_init(
        &state.missile_mass,
        scenario.mass_kg - scenario.propellant_mass_kg,
        scenario.propellant_mass_kg);
    if (status != SIM_OK) {
        (void)fclose(trajectory_log);
        if (sensor_log != 0) {
            (void)fclose(sensor_log);
        }
        if (command_log != 0) {
            (void)fclose(command_log);
        }
        if (event_log != 0) {
            (void)fclose(event_log);
        }
        (void)close(sock);
        config_free(&scenario_tree);
        config_free(&runtime_tree);
        return status;
    }
    state.initial_mass_kg = state.missile_mass.mass_kg;
    state.missile_plant.mass = state.missile_mass.mass_kg;
    state.missile_plant.inertia_b = matrix3_zero();
    state.missile_plant.inertia_b.m[0][0] = scenario.inertia_diag[0];
    state.missile_plant.inertia_b.m[1][1] = scenario.inertia_diag[1];
    state.missile_plant.inertia_b.m[2][2] = scenario.inertia_diag[2];
    state.initial_inertia_b = state.missile_plant.inertia_b;
    {
        size_t index;

        for (index = 0u; index < 3u; ++index) {
            state.acceleration_actuators[index].pos_min = -scenario.acceleration_limit_mps2;
            state.acceleration_actuators[index].pos_max = scenario.acceleration_limit_mps2;
            state.acceleration_actuators[index].rate_limit =
                scenario.acceleration_rate_limit_mps3;
            state.acceleration_actuators[index].time_constant = scenario.command_tau_s;
        }
    }
    status = update_geodetic_state(&state, &earth, &terrain);
    if (status != SIM_OK) {
        exit_reason = "terrain_initialization_failed";
        write_event(event_log, state.time, "TERRAIN_ERROR", sim_status_to_string(status));
    }
    state.min_range = vec3_norm(vec3_sub(state.target_pos, state.missile_pos));
    if (trajectory_log != 0) {
        write_trajectory_header(trajectory_log);
        write_trajectory_row(trajectory_log, &state);
    }

    (void)logger_info(&logger, "environment_sim UDP loop started");
    write_event(event_log, state.time, "SIMULATION_START", "lockstep");
    (void)printf("instance_id=%u env_port=%u fc_port=%u\n", ctx->instance_id, env_port, fc_port);

    while (status == SIM_OK && state.time <= scenario.max_time) {
        SensorFrame sensor;
        ControlCommand command;
        FaultStepEffects fault_effects;
        FaultTransition fault_transitions[ENV_MAX_FAULT_TRANSITIONS];
        size_t fault_transition_count = 0u;
        double range = vec3_norm(vec3_sub(state.target_pos, state.missile_pos));
        int surface_collision = 0;

        status = terrain_is_surface_collision(&terrain, &state.missile_lla, &surface_collision);
        if (status != SIM_OK) {
            exit_reason = "terrain_query_failed";
            write_event(event_log, state.time, "TERRAIN_ERROR", sim_status_to_string(status));
            break;
        }
        if (surface_collision != 0) {
            exit_reason = "surface_collision";
            write_event(event_log, state.time, "SURFACE_COLLISION", "agl_non_positive");
            break;
        }
        if (range < state.min_range) {
            state.min_range = range;
            state.time_of_closest = state.time;
        }
        if (range <= scenario.hit_radius_m) {
            hit = 1;
            exit_reason = "hit";
            write_event(event_log, state.time, "HIT", "truth_range_threshold");
            break;
        }
        status = fault_injection_update(
            &faults,
            state.time,
            &fault_effects,
            fault_transitions,
            ENV_MAX_FAULT_TRANSITIONS,
            &fault_transition_count);
        if (status != SIM_OK) {
            exit_reason = "fault_update_failed";
            write_event(event_log, state.time, "FAULT_ERROR", sim_status_to_string(status));
            break;
        }
        {
            size_t transition_index;

            for (transition_index = 0u;
                 transition_index < fault_transition_count &&
                     transition_index < ENV_MAX_FAULT_TRANSITIONS;
                 ++transition_index) {
                char detail[192];

                if (fault_transitions[transition_index].active != 0) {
                    ++fault_stats.fault_start_count;
                } else {
                    ++fault_stats.fault_end_count;
                }
                (void)snprintf(
                    detail,
                    sizeof(detail),
                    "id=%s target=%s type=%s",
                    fault_transitions[transition_index].id,
                    fault_transitions[transition_index].target,
                    fault_transitions[transition_index].type);
                write_event(
                    event_log,
                    state.time,
                    fault_transitions[transition_index].active != 0 ?
                        "FAULT_START" :
                        "FAULT_END",
                    detail);
            }
        }
        if (fault_effects.active_fault_count > 0u) {
            ++fault_stats.active_step_count;
        }
        if (fault_effects.active_fault_count > fault_stats.max_concurrent_active) {
            fault_stats.max_concurrent_active = fault_effects.active_fault_count;
        }
        if (fault_effects_affect_sensor(&fault_effects) != 0) {
            ++fault_stats.sensor_affected_step_count;
        }
        if (fault_effects_affect_actuator(&fault_effects) != 0) {
            ++fault_stats.actuator_affected_step_count;
        }
        status = build_sensor_frame(
            &state,
            &sensors,
            scenario.dt,
            seq,
            &sensor);
        if (status != SIM_OK) {
            exit_reason = "sensor_update_failed";
            write_event(event_log, state.time, "SENSOR_ERROR", sim_status_to_string(status));
            break;
        }
        if (scenario.los_occlusion_enabled != 0) {
            int occluded = 0;

            status = terrain_line_of_sight_occluded(
                &terrain,
                &earth,
                state.missile_pos,
                state.target_pos,
                16u,
                &occluded);
            if (status != SIM_OK) {
                exit_reason = "los_terrain_query_failed";
                write_event(event_log, state.time, "TERRAIN_ERROR", sim_status_to_string(status));
                break;
            }
            if (occluded != 0) {
                sensor.sensor_valid_flags &= ~SIM_SENSOR_VALID_SEEKER;
                sensor.sensor_fault_flags |= SIM_SENSOR_FAULT_LOS_OCCLUDED;
            }
        }
        fault_injection_apply_sensor(&fault_effects, &sensor);

        status = send_sensor_frame(sock, &fc_addr, ctx->instance_id, &sensor);
        if (status != SIM_OK) {
            exit_reason = "send_failed";
            break;
        }
        if (seq == 0u) {
            (void)printf("sent first SensorFrame range=%.3f closing=%.3f\n",
                sensor.target_range_meas,
                sensor.target_closing_velocity_meas);
        }
        status = write_sensor_log(sensor_log, ctx->instance_id, &sensor);
        if (status != SIM_OK) {
            exit_reason = "sensor_log_failed";
            write_event(event_log, state.time, "IO_ERROR", exit_reason);
            break;
        }

        status = receive_control_command(sock, ctx->instance_id, &command);
        if (status != SIM_OK) {
            exit_reason = "control_timeout";
            write_event(event_log, state.time, "CONTROL_RECEIVE_FAILED", sim_status_to_string(status));
            break;
        }
        status = write_command_log(command_log, ctx->instance_id, &command);
        if (status != SIM_OK) {
            exit_reason = "command_log_failed";
            write_event(event_log, state.time, "IO_ERROR", exit_reason);
            break;
        }
        status = update_truth(
            &state,
            &command,
            &scenario,
            &scenario.force_model,
            &fault_effects);
        if (status != SIM_OK) {
            exit_reason = "plant_update_failed";
            write_event(event_log, state.time, "PLANT_ERROR", sim_status_to_string(status));
            break;
        }
        status = update_geodetic_state(&state, &earth, &terrain);
        if (status != SIM_OK) {
            exit_reason = "geodetic_update_failed";
            write_event(event_log, state.time, "NUMERIC_ERROR", sim_status_to_string(status));
            break;
        }
        if (trajectory_log != 0) {
            write_trajectory_row(trajectory_log, &state);
        }
        ++seq;
        if (runtime.flush_every_steps > 0u && (seq % runtime.flush_every_steps) == 0u) {
            if (sensor_log != 0) {
                (void)fflush(sensor_log);
            }
            if (command_log != 0) {
                (void)fflush(command_log);
            }
            (void)fflush(trajectory_log);
            if (event_log != 0) {
                (void)fflush(event_log);
            }
        }
    }

    if (sensor_log != 0) {
        (void)fclose(sensor_log);
    }
    if (command_log != 0) {
        (void)fclose(command_log);
    }
    if (trajectory_log != 0) {
        (void)fclose(trajectory_log);
    }
    write_event(event_log, state.time, "SIMULATION_STOP", exit_reason);
    if (event_log != 0) {
        (void)fclose(event_log);
    }
    write_summary(instance_dir, hit, &state, seq, exit_reason, &fault_stats);

    if (sock >= 0) {
        (void)close(sock);
    }
    config_free(&scenario_tree);
    config_free(&runtime_tree);
    (void)printf("summary: hit=%d min_range=%.3f exit=%s steps=%u\n", hit, state.min_range, exit_reason, seq);
    return status == SIM_ERR_TIMEOUT && hit != 0 ? SIM_OK : status;
}
