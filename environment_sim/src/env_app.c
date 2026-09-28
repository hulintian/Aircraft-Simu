/** @file env_app.c
 *  @brief 环境仿真主循环实现。
 *
 *  该模块维护真实世界状态，推进导弹和目标运动，并在每个仿真步向飞控
 *  提供带噪声的传感器数据。
 */
#include "env/env_app.h"

#include "common/config.h"
#include "common/build_info.h"
#include "common/crc32.h"
#include "common/logger.h"
#include "common/math_constants.h"
#include "common/matrix3.h"
#include "common/packet.h"
#include "common/protocol.h"
#include "common/quaternion.h"
#include "common/realtime_pacer.h"
#include "common/status.h"
#include "common/vec3.h"
#include "env/actuator_model.h"
#include "env/aero_surrogate.h"
#include "env/earth_model.h"
#include "env/environment_force_model.h"
#include "env/fault_injection.h"
#include "env/geo_coordinate.h"
#include "env/hit_detect.h"
#include "env/mass_model.h"
#include "env/missile_plant_6dof.h"
#include "env/sensor_accel.h"
#include "env/sensor_imu.h"
#include "env/sensor_seeker.h"
#include "env/sensor_speed.h"
#include "env/terrain_model.h"
#include "env/target_model.h"
#include "env/wind_model.h"

#include <arpa/inet.h>
#include <ctype.h>
#include <errno.h>
#include <math.h>
#include <netinet/in.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <time.h>
#include <unistd.h>

#define ENV_PACKET_BUFFER_SIZE 1024u
#define ENV_MAX_DRAINED_COMMANDS_PER_STEP 64u
#define ENV_MAX_CONFIG_WARNINGS 64u
#define ENV_MAX_TERRAIN_TILES 16u
#define ENV_COMM_DELAY_BUFFER_CAPACITY (ENV_MAX_COMMUNICATION_DELAY_STEPS + 1u)
#define ENV_ACTUATOR_DELAY_BUFFER_CAPACITY (ENV_MAX_ACTUATOR_DELAY_STEPS + 1u)
#define ENV_SENSOR_FAULT_DELAY_BUFFER_CAPACITY (ENV_MAX_SENSOR_DELAY_STEPS + 1u)
#define ENV_MODEL_DEGRADATION_AERO_FLAGS UINT32_C(0x00000001)
#define ENV_MODEL_DEGRADATION_TERRAIN_WARNING UINT32_C(0x00000002)
#define ENV_MODEL_DEGRADATION_MASS_INVALID UINT32_C(0x00000004)
#define ENV_MODEL_DEGRADATION_INERTIA_INVALID UINT32_C(0x00000008)
#define ENV_MODEL_DEGRADATION_ATTITUDE_ERROR UINT32_C(0x00000010)
#define ENV_MODEL_DEGRADATION_DCM_ERROR UINT32_C(0x00000020)

/** @brief 环境进程墙钟执行模式；物理同步仍保持 LOCKSTEP。 */
typedef enum EnvRunMode {
    ENV_RUN_MODE_SIL_FAST = 0,
    ENV_RUN_MODE_SIL_REALTIME = 1
} EnvRunMode;

/** @brief 环境与飞控之间的命令同步策略。 */
typedef enum EnvSynchronizationMode {
    ENV_SYNC_LOCKSTEP = 0,
    ENV_SYNC_FREE_RUNNING = 1
} EnvSynchronizationMode;

typedef struct EnvRuntimeConfig {
    /** @brief 本次任务稳定标识。 */
    char campaign_id[128];
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
    /** @brief 最快推进或按墙钟节拍推进。 */
    EnvRunMode run_mode;
    /** @brief 每帧等待命令或非阻塞保持最新命令。 */
    EnvSynchronizationMode synchronization_mode;
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
    /** @brief 是否启用质心迁移和完整惯量张量模型。 */
    int mass_properties_enabled;
    /** @brief 干体质心，机体系，单位米。 */
    Vec3 dry_center_of_mass_b_m;
    /** @brief 满装推进剂质心，机体系，单位米。 */
    Vec3 propellant_center_of_mass_full_b_m;
    /** @brief 空箱推进剂等效质心，机体系，单位米。 */
    Vec3 propellant_center_of_mass_empty_b_m;
    /** @brief 干体绕自身质心的完整惯量张量。 */
    Matrix3 dry_inertia_centroid_b_kgm2;
    /** @brief 满装推进剂绕自身质心的完整惯量张量。 */
    Matrix3 propellant_inertia_full_centroid_b_kgm2;
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
    /** @brief 实例私有风切变、阵风和湍流配置。 */
    WindModelConfig wind_model;
    /** @brief 是否配置了内部气动表文件。 */
    int aero_table_path_enabled;
    /** @brief 内部气动表文件路径。 */
    char aero_table_path[512];
    /** @brief 是否配置了六维 v2 气动表文件。 */
    int aero_table_v2_path_enabled;
    /** @brief 六维 v2 气动表文件路径。 */
    char aero_table_v2_path[512];
    /** @brief 已加载 v2 气动表的六个轴维度。 */
    size_t aero_table_v2_dimensions[AERO_DATABASE_V2_AXIS_COUNT];
    /** @brief 是否用配置覆盖气动表文件中的包络外策略。 */
    int aero_table_policy_override_enabled;
    /** @brief 配置覆盖的气动表包络外策略。 */
    AeroDatabaseExtrapolationPolicy aero_table_policy_override;
    /** @brief 是否配置气动表高度包线元数据。 */
    int aero_table_height_envelope_enabled;
    /** @brief 气动表高度包线下界，单位米。 */
    double aero_table_height_min_m;
    /** @brief 气动表高度包线上界，单位米。 */
    double aero_table_height_max_m;
    /** @brief 是否配置气动表舵偏包线元数据。 */
    int aero_table_actuator_envelope_enabled;
    /** @brief 气动表舵偏包线下界，单位 rad。 */
    double aero_table_actuator_min_rad;
    /** @brief 气动表舵偏包线上界，单位 rad。 */
    double aero_table_actuator_max_rad;
    /** @brief 是否配置代理气动模型资源路径。 */
    int aero_surrogate_model_path_enabled;
    /** @brief 代理气动模型资源路径。 */
    char aero_surrogate_model_path[512];
    /** @brief 代理气动模型版本字符串。 */
    char aero_surrogate_model_version[128];
    /** @brief 代理模型训练数据版本字符串。 */
    char aero_surrogate_training_data_version[128];
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
    /** @brief 本实例允许加载的地形瓦片容量。 */
    size_t terrain_cache_tile_count;
    /** @brief 是否配置地形资源来源清单。 */
    int terrain_resource_manifest_path_enabled;
    /** @brief 地形资源来源、许可和派生过程清单路径。 */
    char terrain_resource_manifest_path[512];
    /** @brief 是否配置地形瓦片索引文件。 */
    int terrain_tile_index_path_enabled;
    /** @brief 地形瓦片索引文件路径。 */
    char terrain_tile_index_path[512];
    /** @brief 配置的内部地形瓦片文件数量。 */
    size_t terrain_tile_path_count;
    /** @brief 内部地形瓦片文件路径数组。 */
    char terrain_tile_paths[ENV_MAX_TERRAIN_TILES][512];
    /** @brief 内部地形瓦片索引元数据数组。 */
    TerrainTileIndexEntry terrain_tile_index_entries[ENV_MAX_TERRAIN_TILES];
    /** @brief 导弹初始纬度、经度和椭球高，单位度、度、米。 */
    double missile_lla[3];
    /** @brief 导弹初始 ECEF 速度，单位 m/s。 */
    double missile_vel[3];
    /** @brief 目标初始纬度、经度和椭球高，单位度、度、米。 */
    double target_lla[3];
    /** @brief 目标初始 ECEF 速度，单位 m/s。 */
    double target_vel[3];
    /** @brief 目标匀速或脚本加速度真值模型。 */
    TargetModel target_model;
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
    /** @brief 实例私有风场运行状态。 */
    WindModel wind_model;
    /** @brief 最近一步 ECEF 风速，单位 m/s。 */
    Vec3 wind_velocity_ecef_mps;
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
    /** @brief 最近一次力模型计算返回的气动诊断标志。 */
    uint32_t aero_model_flags;
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

typedef struct CommunicationDelayLine {
    SensorFrame frames[ENV_COMM_DELAY_BUFFER_CAPACITY];
    size_t head;
    size_t count;
} CommunicationDelayLine;

typedef struct CommunicationReorderState {
    SensorFrame previous;
    int has_previous;
} CommunicationReorderState;

typedef struct ActuatorCommandDelayLine {
    double commands[ENV_ACTUATOR_DELAY_BUFFER_CAPACITY];
    size_t head;
    size_t count;
} ActuatorCommandDelayLine;

typedef struct SensorFaultDelayLine {
    SensorFrame frames[ENV_SENSOR_FAULT_DELAY_BUFFER_CAPACITY];
    size_t head;
    size_t count;
} SensorFaultDelayLine;

typedef struct SensorFaultStuckState {
    SensorFrame held;
    int active;
} SensorFaultStuckState;

/** @brief 单实例数值诊断聚合，用于 summary.json 和批量统计。 */
typedef struct DiagnosticRunStats {
    /** @brief 最大四元数范数误差。 */
    double max_quat_norm_error;
    /** @brief 最大 DCM 正交性 Frobenius 误差。 */
    double max_dcm_orthogonality_error;
    /** @brief 仿真期间最小总质量，单位 kg。 */
    double min_mass_kg;
    /** @brief 仿真期间最小惯量对角线元素，单位 kg*m^2。 */
    double min_inertia_diag_kgm2;
    /** @brief 本次运行中所有气动模型诊断标志的按位或。 */
    uint32_t aero_model_flags_or;
    /** @brief 发生气动表包线外处理的诊断采样数。 */
    size_t aero_extrapolated_sample_count;
    /** @brief 本次运行中所有模型降级分类标志的按位或。 */
    uint32_t model_degradation_flags_or;
    /** @brief 已采样诊断行数。 */
    size_t sample_count;
} DiagnosticRunStats;

/** @brief 一类墙钟耗时的固定内存统计。 */
typedef struct TimingAccumulator {
    size_t count;
    double total_s;
    double max_s;
} TimingAccumulator;

/** @brief 单实例闭环运行指标，用于 summary/performance 输出。 */
typedef struct OperationalRunStats {
    /** @brief 飞控 ECEF 加速度指令最大范数，单位 m/s^2。 */
    double max_command_norm;
    /** @brief 执行机构响应后 ECEF 加速度最大范数，单位 m/s^2。 */
    double max_actual_accel;
    /** @brief 至少一类预期传感器测量无效的发送帧数。 */
    size_t sensor_dropout_count;
    /** @brief 等待控制指令超时次数。 */
    size_t command_timeout_count;
    /** @brief FREE_RUNNING 下未收到新命令而保持上一指令的步数。 */
    size_t command_hold_count;
    /** @brief 接收并兼容处理的协议次版本不一致报文数。 */
    size_t protocol_minor_mismatch_count;
    /** @brief 仿真主循环总墙钟耗时。 */
    double run_wall_time_s;
    /** @brief 单步计算耗时，不含实时节拍等待。 */
    TimingAccumulator step_compute;
    /** @brief 传感器 UDP 发送耗时。 */
    TimingAccumulator sensor_send;
    /** @brief 控制命令 UDP 接收等待耗时。 */
    TimingAccumulator command_receive;
    /** @brief 发送传感器到收到控制命令的往返耗时。 */
    TimingAccumulator control_roundtrip;
    /** @brief 二进制/CSV 日志写入耗时。 */
    TimingAccumulator log_write;
    /** @brief 实时节拍累计睡眠时间。 */
    double realtime_sleep_time_s;
    /** @brief 未能在步截止时间前完成的次数。 */
    size_t realtime_overrun_count;
    /** @brief 最大实时超限时间。 */
    double max_realtime_overrun_s;
} OperationalRunStats;

/** @brief 启动配置中允许但需要告警的未识别字段集合。 */
typedef struct ConfigWarningCollector {
    char source[32];
    char paths[ENV_MAX_CONFIG_WARNINGS][256];
    size_t stored_count;
    size_t total_count;
} ConfigWarningCollector;

/** @brief 收集未知字段路径并立即输出启动 warning。 */
static void collect_unknown_config_key(
    const char *object_path,
    const char *key,
    void *user_data)
{
    ConfigWarningCollector *collector = (ConfigWarningCollector *)user_data;
    char full_path[256];

    if (collector == 0 || object_path == 0 || key == 0) {
        return;
    }
    (void)snprintf(
        full_path,
        sizeof(full_path),
        "%s.%s%s%s",
        collector->source,
        object_path[0] == '\0' ? "" : object_path,
        object_path[0] == '\0' ? "" : ".",
        key);
    (void)fprintf(stderr, "environment_sim: config warning: unrecognized field %s\n", full_path);
    if (collector->stored_count < ENV_MAX_CONFIG_WARNINGS) {
        (void)snprintf(
            collector->paths[collector->stored_count],
            sizeof(collector->paths[collector->stored_count]),
            "%s",
            full_path);
        ++collector->stored_count;
    }
    ++collector->total_count;
}

/** @brief 校验一个必需或可选对象的直接字段白名单。 */
static SimStatus check_config_object_keys(
    const ConfigTree *tree,
    const char *source,
    const char *path,
    const char *const *allowed_keys,
    size_t allowed_key_count,
    int optional,
    ConfigWarningCollector *collector)
{
    int exists = 1;
    size_t unknown_count = 0u;
    SimStatus status;

    if (tree == 0 || source == 0 || path == 0 || collector == 0) {
        return SIM_ERR_INVALID_ARG;
    }
    if (path[0] != '\0') {
        status = config_path_exists(tree, path, &exists);
        if (status != SIM_OK) {
            return status;
        }
        if (exists == 0) {
            return optional != 0 ? SIM_OK : SIM_ERR_CONFIG;
        }
    }
    (void)snprintf(collector->source, sizeof(collector->source), "%s", source);
    return config_visit_unknown_keys(
        tree,
        path,
        allowed_keys,
        allowed_key_count,
        collect_unknown_config_key,
        collector,
        &unknown_count);
}

#define ENV_ARRAY_COUNT(values) (sizeof(values) / sizeof((values)[0]))

/** @brief 对环境、运行时和故障配置执行已知字段审计。 */
static SimStatus audit_environment_config_fields(
    const ConfigTree *scenario,
    const ConfigTree *runtime,
    const ConfigTree *faults,
    ConfigWarningCollector *collector)
{
    static const char *const scenario_root[] = {
        "schema_version", "simulation", "earth", "map", "plant", "gravity",
        "atmosphere", "propulsion", "aerodynamics", "missile", "target", "sensors"
    };
    static const char *const simulation_keys[] = { "mode", "dt", "max_time", "hit_radius_m" };
    static const char *const earth_keys[] = { "model", "enable_rotation_terms", "origin" };
    static const char *const origin_keys[] = { "lat_deg", "lon_deg", "height_m", "local_frame" };
    static const char *const map_keys[] = {
        "database_path", "tile_index", "tile_path", "tile_paths", "tile_index_path",
        "resource_manifest_path", "enable_terrain", "enable_los_occlusion",
        "missing_tile_policy", "terrain"
    };
    static const char *const terrain_keys[] = {
        "interpolation", "height_reference", "cache_tile_count", "flat_fill_height_m"
    };
    static const char *const plant_keys[] = {
        "model", "integrator", "mass_kg", "propellant_mass_kg", "inertia_diag",
        "mass_properties", "command_tau_s", "acceleration_limit_mps2",
        "acceleration_rate_limit_mps3"
    };
    static const char *const mass_property_keys[] = {
        "enabled", "dry_center_of_mass_b_m", "propellant_center_of_mass_full_b_m",
        "propellant_center_of_mass_empty_b_m", "dry_inertia_centroid_b_kgm2",
        "propellant_inertia_full_centroid_b_kgm2"
    };
    static const char *const gravity_keys[] = { "enabled" };
    static const char *const atmosphere_keys[] = {
        "enabled", "maximum_model_height_m", "wind_velocity_ecef_mps", "wind_model"
    };
    static const char *const wind_keys[] = {
        "enabled", "reference_height_m", "shear_ecef_per_m", "gust_amplitude_ecef_mps",
        "gust_frequency_hz", "turbulence_sigma_ecef_mps", "turbulence_time_constant_s"
    };
    static const char *const propulsion_keys[] = {
        "enabled", "thrust_n", "mass_flow_kgps", "burn_time_s", "thrust_direction_b"
    };
    static const char *const aero_keys[] = {
        "enabled", "reference_area_m2", "reference_length_m", "drag_coefficient",
        "control_force_coefficient", "control_moment_coefficient", "table_path",
        "table_v2_path", "table_extrapolation_policy", "table_height_min_m",
        "table_height_max_m", "table_actuator_min_rad", "table_actuator_max_rad",
        "surrogate_model_path", "surrogate_model_version", "surrogate_training_data_version"
    };
    static const char *const missile_keys[] = {
        "initial_lla_deg_m", "initial_velocity_ecef_mps"
    };
    static const char *const target_keys[] = {
        "model", "initial_lla_deg_m", "initial_velocity_ecef_mps", "maneuvers"
    };
    static const char *const target_maneuver_keys[] = {
        "start_time_s", "duration_s", "acceleration_ecef_mps2"
    };
    static const char *const sensors_keys[] = {
        "imu", "accelerometer", "speedometer", "seeker"
    };
    static const char *const vector_sensor_keys[] = {
        "enabled", "sample_period_s", "delay_s", "dropout_probability", "noise"
    };
    static const char *const seeker_keys[] = {
        "enabled", "sample_period_s", "delay_s", "dropout_probability", "range_noise",
        "los_unit_noise", "los_rate_noise", "closing_velocity_noise"
    };
    static const char *const noise_keys[] = {
        "bias", "bias_xyz", "white_noise_std", "random_walk_std", "min_value",
        "max_value", "resolution"
    };
    static const char *const runtime_root[] = {
        "schema_version", "campaign", "network", "logging", "tools", "instances"
    };
    static const char *const campaign_keys[] = {
        "campaign_id", "instance_count", "schedule", "run_mode", "synchronization_mode",
        "max_parallel_instances", "base_random_seed", "failure_strategy"
    };
    static const char *const network_keys[] = {
        "protocol_version_major", "protocol_version_minor", "environment_base_port",
        "flight_control_base_port", "host"
    };
    static const char *const logging_keys[] = {
        "output_dir", "instance_dir_template", "binary_logs", "event_log", "flush_every_steps"
    };
    static const char *const tools_keys[] = { "environment_program", "flight_control_program" };
    static const char *const instance_keys[] = {
        "enabled", "instance_id", "scenario", "flight_control", "faults", "random_seed"
    };
    static const char *const faults_root[] = { "schema_version", "faults" };
    static const char *const fault_keys[] = {
        "id", "enabled", "start_time_s", "time_s", "duration_s", "target", "type",
        "value", "value_xyz", "scale", "ramp_in_s", "recovery_ramp_s",
        "recovery_hold_s", "recovery_timeout_s", "burst_period_s", "burst_active_s",
        "delay_pattern_steps", "min_value", "max_value"
    };
    static const char *const vector_sensor_paths[] = {
        "sensors.imu", "sensors.accelerometer", "sensors.speedometer"
    };
    static const char *const noise_paths[] = {
        "sensors.imu.noise", "sensors.accelerometer.noise", "sensors.speedometer.noise",
        "sensors.seeker.range_noise", "sensors.seeker.los_unit_noise",
        "sensors.seeker.los_rate_noise", "sensors.seeker.closing_velocity_noise"
    };
    size_t index;
    size_t count = 0u;
    SimStatus status;

#define CHECK(tree, source, path, keys, optional) \
    do { \
        status = check_config_object_keys( \
            tree, source, path, keys, ENV_ARRAY_COUNT(keys), optional, collector); \
        if (status != SIM_OK) { \
            return status; \
        } \
    } while (0)
    CHECK(scenario, "scenario", "", scenario_root, 0);
    CHECK(scenario, "scenario", "simulation", simulation_keys, 0);
    CHECK(scenario, "scenario", "earth", earth_keys, 0);
    CHECK(scenario, "scenario", "earth.origin", origin_keys, 1);
    CHECK(scenario, "scenario", "map", map_keys, 0);
    CHECK(scenario, "scenario", "map.terrain", terrain_keys, 0);
    CHECK(scenario, "scenario", "plant", plant_keys, 0);
    CHECK(scenario, "scenario", "plant.mass_properties", mass_property_keys, 1);
    CHECK(scenario, "scenario", "gravity", gravity_keys, 0);
    CHECK(scenario, "scenario", "atmosphere", atmosphere_keys, 0);
    CHECK(scenario, "scenario", "atmosphere.wind_model", wind_keys, 1);
    CHECK(scenario, "scenario", "propulsion", propulsion_keys, 0);
    CHECK(scenario, "scenario", "aerodynamics", aero_keys, 0);
    CHECK(scenario, "scenario", "missile", missile_keys, 0);
    CHECK(scenario, "scenario", "target", target_keys, 0);
    if (config_get_array_count(scenario, "target.maneuvers", &count) == SIM_OK) {
        for (index = 0u; index < count; ++index) {
            char maneuver_path[64];

            (void)snprintf(maneuver_path, sizeof(maneuver_path), "target.maneuvers[%zu]", index);
            status = check_config_object_keys(
                scenario, "scenario", maneuver_path, target_maneuver_keys,
                ENV_ARRAY_COUNT(target_maneuver_keys), 0, collector);
            if (status != SIM_OK) {
                return status;
            }
        }
    }
    CHECK(scenario, "scenario", "sensors", sensors_keys, 0);
    for (index = 0u; index < ENV_ARRAY_COUNT(vector_sensor_paths); ++index) {
        status = check_config_object_keys(
            scenario, "scenario", vector_sensor_paths[index], vector_sensor_keys,
            ENV_ARRAY_COUNT(vector_sensor_keys), 0, collector);
        if (status != SIM_OK) {
            return status;
        }
    }
    CHECK(scenario, "scenario", "sensors.seeker", seeker_keys, 0);
    for (index = 0u; index < ENV_ARRAY_COUNT(noise_paths); ++index) {
        status = check_config_object_keys(
            scenario, "scenario", noise_paths[index], noise_keys,
            ENV_ARRAY_COUNT(noise_keys), 0, collector);
        if (status != SIM_OK) {
            return status;
        }
    }
    CHECK(runtime, "runtime", "", runtime_root, 0);
    CHECK(runtime, "runtime", "campaign", campaign_keys, 0);
    CHECK(runtime, "runtime", "network", network_keys, 0);
    CHECK(runtime, "runtime", "logging", logging_keys, 0);
    CHECK(runtime, "runtime", "tools", tools_keys, 1);
    if (config_get_array_count(runtime, "instances", &count) == SIM_OK) {
        for (index = 0u; index < count; ++index) {
            char instance_path[64];

            (void)snprintf(instance_path, sizeof(instance_path), "instances[%zu]", index);
            status = check_config_object_keys(
                runtime, "runtime", instance_path, instance_keys,
                ENV_ARRAY_COUNT(instance_keys), 0, collector);
            if (status != SIM_OK) {
                return status;
            }
        }
    }
    CHECK(faults, "faults", "", faults_root, 0);
    status = config_get_array_count(faults, "faults", &count);
    if (status != SIM_OK) {
        return status;
    }
    for (index = 0u; index < count; ++index) {
        char fault_path[64];

        (void)snprintf(fault_path, sizeof(fault_path), "faults[%zu]", index);
        status = check_config_object_keys(
            faults, "faults", fault_path, fault_keys,
            ENV_ARRAY_COUNT(fault_keys), 0, collector);
        if (status != SIM_OK) {
            return status;
        }
    }
#undef CHECK
    return SIM_OK;
}

#undef ENV_ARRAY_COUNT

/** @brief 记录一次有限非负墙钟耗时。 */
static void timing_accumulate(TimingAccumulator *stats, double duration_s)
{
    if (stats == 0 || !isfinite(duration_s) || duration_s < 0.0) {
        return;
    }
    ++stats->count;
    stats->total_s += duration_s;
    if (duration_s > stats->max_s) {
        stats->max_s = duration_s;
    }
}

/** @brief 完成一次墙钟计时并更新统计。 */
static SimStatus timing_finish(double start_s, TimingAccumulator *stats)
{
    double end_s;
    SimStatus status = monotonic_time_now(&end_s);

    if (status == SIM_OK) {
        timing_accumulate(stats, end_s - start_s);
    }
    return status;
}

/** @brief 返回运行模式的稳定 manifest 字符串。 */
static const char *env_run_mode_to_string(EnvRunMode mode)
{
    return mode == ENV_RUN_MODE_SIL_REALTIME ? "SIL_REALTIME" : "SIL_FAST";
}

/** @brief 返回同步策略的稳定 manifest 字符串。 */
static const char *env_synchronization_mode_to_string(EnvSynchronizationMode mode)
{
    return mode == ENV_SYNC_FREE_RUNNING ? "FREE_RUNNING" : "LOCKSTEP";
}

/** @brief 返回积分器稳定名称。 */
static const char *integrator_type_to_string(IntegratorType type)
{
    if (type == INTEGRATOR_EULER) {
        return "EULER";
    }
    if (type == INTEGRATOR_RK2) {
        return "RK2";
    }
    return "RK4";
}

/** @brief 从运行时配置读取网络和输出目录。 */
static SimStatus load_runtime_config(const ConfigTree *runtime, EnvRuntimeConfig *out)
{
    SimStatus status;

    if (runtime == 0 || out == 0) {
        return SIM_ERR_INVALID_ARG;
    }
    memset(out, 0, sizeof(*out));

    status = config_get_string(runtime, "campaign.campaign_id", out->campaign_id, sizeof(out->campaign_id));
    if (status != SIM_OK || out->campaign_id[0] == '\0') {
        return SIM_ERR_CONFIG;
    }
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
    {
        char run_mode[32];
        int run_mode_exists = 0;

        status = config_path_exists(runtime, "campaign.run_mode", &run_mode_exists);
        if (status != SIM_OK) {
            return status;
        }
        if (run_mode_exists == 0) {
            out->run_mode = ENV_RUN_MODE_SIL_FAST;
        } else {
            status = config_get_string(runtime, "campaign.run_mode", run_mode, sizeof(run_mode));
            if (status != SIM_OK) {
                return status;
            }
            if (strcmp(run_mode, "SIL_FAST") == 0) {
                out->run_mode = ENV_RUN_MODE_SIL_FAST;
            } else if (strcmp(run_mode, "SIL_REALTIME") == 0) {
                out->run_mode = ENV_RUN_MODE_SIL_REALTIME;
            } else {
                return SIM_ERR_CONFIG;
            }
        }
    }
    {
        char synchronization_mode[32];
        int synchronization_mode_exists = 0;

        status = config_path_exists(
            runtime,
            "campaign.synchronization_mode",
            &synchronization_mode_exists);
        if (status != SIM_OK) {
            return status;
        }
        if (synchronization_mode_exists == 0) {
            out->synchronization_mode = ENV_SYNC_LOCKSTEP;
        } else {
            status = config_get_string(
                runtime,
                "campaign.synchronization_mode",
                synchronization_mode,
                sizeof(synchronization_mode));
            if (status != SIM_OK) {
                return status;
            }
            if (strcmp(synchronization_mode, "LOCKSTEP") == 0) {
                out->synchronization_mode = ENV_SYNC_LOCKSTEP;
            } else if (strcmp(synchronization_mode, "FREE_RUNNING") == 0) {
                out->synchronization_mode = ENV_SYNC_FREE_RUNNING;
            } else {
                return SIM_ERR_CONFIG;
            }
        }
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

/** @brief 追加一个内部地形瓦片路径到场景配置快照。 */
static SimStatus append_terrain_tile_path(EnvScenarioConfig *out, const char *path)
{
    int written;

    if (out == 0 || path == 0 || path[0] == '\0') {
        return SIM_ERR_INVALID_ARG;
    }
    if (out->terrain_tile_path_count >= ENV_MAX_TERRAIN_TILES) {
        return SIM_ERR_OUT_OF_RANGE;
    }
    written = snprintf(
        out->terrain_tile_paths[out->terrain_tile_path_count],
        sizeof(out->terrain_tile_paths[out->terrain_tile_path_count]),
        "%s",
        path);
    if (written < 0 ||
        (size_t)written >= sizeof(out->terrain_tile_paths[out->terrain_tile_path_count])) {
        return SIM_ERR_OUT_OF_RANGE;
    }
    (void)memset(
        &out->terrain_tile_index_entries[out->terrain_tile_path_count],
        0,
        sizeof(out->terrain_tile_index_entries[out->terrain_tile_path_count]));
    written = snprintf(
        out->terrain_tile_index_entries[out->terrain_tile_path_count].path,
        sizeof(out->terrain_tile_index_entries[out->terrain_tile_path_count].path),
        "%s",
        path);
    if (written < 0 ||
        (size_t)written >= sizeof(out->terrain_tile_index_entries[out->terrain_tile_path_count].path)) {
        return SIM_ERR_OUT_OF_RANGE;
    }
    ++out->terrain_tile_path_count;
    return SIM_OK;
}

/** @brief 追加一个带空间边界的内部地形瓦片索引项。 */
static SimStatus append_terrain_tile_index_entry(
    EnvScenarioConfig *out,
    const TerrainTileIndexEntry *entry)
{
    SimStatus status;

    if (out == 0 || entry == 0) {
        return SIM_ERR_INVALID_ARG;
    }
    status = append_terrain_tile_path(out, entry->path);
    if (status != SIM_OK) {
        return status;
    }
    out->terrain_tile_index_entries[out->terrain_tile_path_count - 1u] = *entry;
    return SIM_OK;
}

/** @brief 判断资源路径是否已经是绝对路径。 */
static int path_is_absolute(const char *path)
{
    return path != 0 &&
        (path[0] == '/' ||
            (isalpha((unsigned char)path[0]) != 0 && path[1] == ':' &&
                (path[2] == '/' || path[2] == '\\')));
}

/** @brief 将索引内的相对瓦片路径解析到索引文件所在目录。 */
static SimStatus resolve_index_relative_path(
    const char *index_path,
    const char *entry_path,
    char *out,
    size_t out_size)
{
    const char *last_separator = 0;
    const char *cursor;
    char entry_copy[512];
    size_t directory_size;
    int written;

    if (index_path == 0 || entry_path == 0 || out == 0 || out_size == 0u ||
        entry_path[0] == '\0') {
        return SIM_ERR_INVALID_ARG;
    }
    written = snprintf(entry_copy, sizeof(entry_copy), "%s", entry_path);
    if (written < 0 || (size_t)written >= sizeof(entry_copy)) {
        return SIM_ERR_OUT_OF_RANGE;
    }
    if (path_is_absolute(entry_copy) != 0) {
        written = snprintf(out, out_size, "%s", entry_copy);
        return written >= 0 && (size_t)written < out_size ? SIM_OK : SIM_ERR_OUT_OF_RANGE;
    }
    for (cursor = index_path; *cursor != '\0'; ++cursor) {
        if (*cursor == '/' || *cursor == '\\') {
            last_separator = cursor;
        }
    }
    if (last_separator == 0) {
        written = snprintf(out, out_size, "%s", entry_copy);
        return written >= 0 && (size_t)written < out_size ? SIM_OK : SIM_ERR_OUT_OF_RANGE;
    }
    directory_size = (size_t)(last_separator - index_path);
    if (directory_size == 0u) {
        written = snprintf(out, out_size, "/%s", entry_copy);
    } else {
        if (directory_size >= out_size) {
            return SIM_ERR_OUT_OF_RANGE;
        }
        (void)memcpy(out, index_path, directory_size);
        out[directory_size] = '\0';
        written = snprintf(out + directory_size, out_size - directory_size, "/%s", entry_copy);
    }
    return written >= 0 && directory_size + (size_t)written < out_size ?
        SIM_OK :
        SIM_ERR_OUT_OF_RANGE;
}

/** @brief 从文件读取小端 16 位无符号整数。 */
static SimStatus read_file_u16_le(FILE *file, uint16_t *out)
{
    unsigned char bytes[2];

    if (file == 0 || out == 0) {
        return SIM_ERR_INVALID_ARG;
    }
    if (fread(bytes, 1u, sizeof(bytes), file) != sizeof(bytes)) {
        return SIM_ERR_CONFIG;
    }
    *out = (uint16_t)bytes[0] | (uint16_t)((uint16_t)bytes[1] << 8u);
    return SIM_OK;
}

/** @brief 从文件读取小端 32 位无符号整数。 */
static SimStatus read_file_u32_le(FILE *file, uint32_t *out)
{
    unsigned char bytes[4];
    size_t index;
    uint32_t value = 0u;

    if (file == 0 || out == 0) {
        return SIM_ERR_INVALID_ARG;
    }
    if (fread(bytes, 1u, sizeof(bytes), file) != sizeof(bytes)) {
        return SIM_ERR_CONFIG;
    }
    for (index = 0u; index < sizeof(bytes); ++index) {
        value |= (uint32_t)bytes[index] << (8u * index);
    }
    *out = value;
    return SIM_OK;
}

/** @brief 从文件读取小端 IEEE-754 双精度值。 */
static SimStatus read_file_double_le(FILE *file, double *out)
{
    unsigned char bytes[8];
    uint64_t bits = 0u;
    size_t index;

    if (file == 0 || out == 0) {
        return SIM_ERR_INVALID_ARG;
    }
    if (fread(bytes, 1u, sizeof(bytes), file) != sizeof(bytes)) {
        return SIM_ERR_CONFIG;
    }
    for (index = 0u; index < sizeof(bytes); ++index) {
        bits |= (uint64_t)bytes[index] << (8u * index);
    }
    (void)memcpy(out, &bits, sizeof(bits));
    return SIM_OK;
}

/** @brief 从纯文本索引读取内部地形瓦片路径。 */
static SimStatus load_terrain_tile_text_index(EnvScenarioConfig *out, const char *index_path)
{
    FILE *file;
    char line[512];

    if (out == 0 || index_path == 0) {
        return SIM_ERR_INVALID_ARG;
    }
    file = fopen(index_path, "r");
    if (file == 0) {
        return SIM_ERR_IO;
    }
    while (fgets(line, sizeof(line), file) != 0) {
        char *start = line;
        char *end;
        SimStatus status;

        if (strchr(line, '\n') == 0 && !feof(file)) {
            (void)fclose(file);
            return SIM_ERR_OUT_OF_RANGE;
        }
        while (*start != '\0' && isspace((unsigned char)*start) != 0) {
            ++start;
        }
        if (*start == '\0' || *start == '#') {
            continue;
        }
        end = start + strlen(start);
        while (end > start && isspace((unsigned char)end[-1]) != 0) {
            --end;
        }
        *end = '\0';
        {
            char resolved_path[512];

            status = resolve_index_relative_path(
                index_path,
                start,
                resolved_path,
                sizeof(resolved_path));
            if (status == SIM_OK) {
                status = append_terrain_tile_path(out, resolved_path);
            }
        }
        if (status != SIM_OK) {
            (void)fclose(file);
            return status;
        }
    }
    return fclose(file) == 0 ? SIM_OK : SIM_ERR_IO;
}

/** @brief 从固定小端二进制空间索引读取内部地形瓦片路径和边界。 */
static SimStatus load_terrain_tile_binary_index(EnvScenarioConfig *out, const char *index_path)
{
    FILE *file;
    uint32_t magic = 0u;
    uint16_t version = 0u;
    uint16_t entry_count = 0u;
    uint16_t index;
    SimStatus status;

    if (out == 0 || index_path == 0) {
        return SIM_ERR_INVALID_ARG;
    }
    file = fopen(index_path, "rb");
    if (file == 0) {
        return SIM_ERR_IO;
    }
    status = read_file_u32_le(file, &magic);
    if (status == SIM_OK) {
        status = read_file_u16_le(file, &version);
    }
    if (status == SIM_OK) {
        status = read_file_u16_le(file, &entry_count);
    }
    if (status != SIM_OK) {
        (void)fclose(file);
        return status;
    }
    if (magic != TERRAIN_TILE_INDEX_MAGIC ||
        version != TERRAIN_TILE_INDEX_VERSION ||
        entry_count > ENV_MAX_TERRAIN_TILES) {
        (void)fclose(file);
        return SIM_ERR_CONFIG;
    }
    for (index = 0u; index < entry_count; ++index) {
        TerrainTileIndexEntry entry;
        uint16_t path_size = 0u;

        (void)memset(&entry, 0, sizeof(entry));
        status = read_file_double_le(file, &entry.lat_min);
        if (status == SIM_OK) {
            status = read_file_double_le(file, &entry.lat_max);
        }
        if (status == SIM_OK) {
            status = read_file_double_le(file, &entry.lon_min);
        }
        if (status == SIM_OK) {
            status = read_file_double_le(file, &entry.lon_max);
        }
        if (status == SIM_OK) {
            status = read_file_u16_le(file, &path_size);
        }
        if (status != SIM_OK) {
            (void)fclose(file);
            return status;
        }
        if (path_size == 0u || path_size >= sizeof(entry.path)) {
            (void)fclose(file);
            return SIM_ERR_CONFIG;
        }
        if (fread(entry.path, 1u, path_size, file) != path_size) {
            (void)fclose(file);
            return SIM_ERR_CONFIG;
        }
        entry.path[path_size] = '\0';
        status = resolve_index_relative_path(
            index_path,
            entry.path,
            entry.path,
            sizeof(entry.path));
        if (status != SIM_OK) {
            (void)fclose(file);
            return status;
        }
        entry.has_bounds = 1;
        status = append_terrain_tile_index_entry(out, &entry);
        if (status != SIM_OK) {
            (void)fclose(file);
            return status;
        }
    }
    return fclose(file) == 0 ? SIM_OK : SIM_ERR_IO;
}

/** @brief 自动识别文本或二进制地形瓦片索引。 */
static SimStatus load_terrain_tile_index(EnvScenarioConfig *out, const char *index_path)
{
    FILE *file;
    uint32_t magic = 0u;
    SimStatus status;

    if (out == 0 || index_path == 0) {
        return SIM_ERR_INVALID_ARG;
    }
    file = fopen(index_path, "rb");
    if (file == 0) {
        return SIM_ERR_IO;
    }
    status = read_file_u32_le(file, &magic);
    (void)fclose(file);
    if (status == SIM_OK && magic == TERRAIN_TILE_INDEX_MAGIC) {
        return load_terrain_tile_binary_index(out, index_path);
    }
    return load_terrain_tile_text_index(out, index_path);
}

/** @brief 从场景配置读取动力学和初始条件。 */
static SimStatus load_scenario_config(const ConfigTree *scenario, EnvScenarioConfig *out)
{
    double vector_values[3];
    double matrix_values[9];
    char integrator[16];
    char missing_policy[32];
    char aero_policy[32];
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
    status = config_get_bool(
        scenario,
        "plant.mass_properties.enabled",
        &out->mass_properties_enabled);
    if (status != SIM_OK) {
        out->mass_properties_enabled = 0;
    }
    if (out->mass_properties_enabled != 0) {
        size_t row;
        size_t column;

        status = config_get_double_array(
            scenario,
            "plant.mass_properties.dry_center_of_mass_b_m",
            vector_values,
            3u);
        if (status != SIM_OK) {
            return status;
        }
        out->dry_center_of_mass_b_m = vec3_make(vector_values[0], vector_values[1], vector_values[2]);
        status = config_get_double_array(
            scenario,
            "plant.mass_properties.propellant_center_of_mass_full_b_m",
            vector_values,
            3u);
        if (status != SIM_OK) {
            return status;
        }
        out->propellant_center_of_mass_full_b_m =
            vec3_make(vector_values[0], vector_values[1], vector_values[2]);
        status = config_get_double_array(
            scenario,
            "plant.mass_properties.propellant_center_of_mass_empty_b_m",
            vector_values,
            3u);
        if (status != SIM_OK) {
            return status;
        }
        out->propellant_center_of_mass_empty_b_m =
            vec3_make(vector_values[0], vector_values[1], vector_values[2]);
        status = config_get_double_array(
            scenario,
            "plant.mass_properties.dry_inertia_centroid_b_kgm2",
            matrix_values,
            9u);
        if (status != SIM_OK) {
            return status;
        }
        for (row = 0u; row < 3u; ++row) {
            for (column = 0u; column < 3u; ++column) {
                out->dry_inertia_centroid_b_kgm2.m[row][column] = matrix_values[(3u * row) + column];
            }
        }
        status = config_get_double_array(
            scenario,
            "plant.mass_properties.propellant_inertia_full_centroid_b_kgm2",
            matrix_values,
            9u);
        if (status != SIM_OK) {
            return status;
        }
        for (row = 0u; row < 3u; ++row) {
            for (column = 0u; column < 3u; ++column) {
                out->propellant_inertia_full_centroid_b_kgm2.m[row][column] =
                    matrix_values[(3u * row) + column];
            }
        }
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
    out->wind_model.enabled = 1;
    out->wind_model.base_velocity_ecef_mps = out->force_model.wind_velocity_ecef_mps;
    out->wind_model.turbulence_time_constant_s = 1.0;
    (void)config_get_bool(scenario, "atmosphere.wind_model.enabled", &out->wind_model.enabled);
    status = config_get_double_array(
        scenario,
        "atmosphere.wind_model.shear_ecef_per_m",
        vector_values,
        3u);
    if (status == SIM_OK) {
        out->wind_model.shear_ecef_per_m = vec3_make(vector_values[0], vector_values[1], vector_values[2]);
    }
    (void)config_get_double(
        scenario,
        "atmosphere.wind_model.reference_height_m",
        &out->wind_model.reference_height_m);
    status = config_get_double_array(
        scenario,
        "atmosphere.wind_model.gust_amplitude_ecef_mps",
        vector_values,
        3u);
    if (status == SIM_OK) {
        out->wind_model.gust_amplitude_ecef_mps =
            vec3_make(vector_values[0], vector_values[1], vector_values[2]);
    }
    (void)config_get_double(
        scenario,
        "atmosphere.wind_model.gust_frequency_hz",
        &out->wind_model.gust_frequency_hz);
    status = config_get_double_array(
        scenario,
        "atmosphere.wind_model.turbulence_sigma_ecef_mps",
        vector_values,
        3u);
    if (status == SIM_OK) {
        out->wind_model.turbulence_sigma_ecef_mps =
            vec3_make(vector_values[0], vector_values[1], vector_values[2]);
    }
    (void)config_get_double(
        scenario,
        "atmosphere.wind_model.turbulence_time_constant_s",
        &out->wind_model.turbulence_time_constant_s);
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
    status = config_get_string(
        scenario,
        "aerodynamics.table_path",
        out->aero_table_path,
        sizeof(out->aero_table_path));
    if (status == SIM_OK) {
        out->aero_table_path_enabled = 1;
    } else if (status == SIM_ERR_OUT_OF_RANGE) {
        return status;
    }
    status = config_get_string(
        scenario,
        "aerodynamics.table_v2_path",
        out->aero_table_v2_path,
        sizeof(out->aero_table_v2_path));
    if (status == SIM_OK) {
        out->aero_table_v2_path_enabled = 1;
    } else if (status == SIM_ERR_OUT_OF_RANGE) {
        return status;
    }
    status = config_get_string(
        scenario,
        "aerodynamics.table_extrapolation_policy",
        aero_policy,
        sizeof(aero_policy));
    if (status == SIM_OK) {
        if (strcmp(aero_policy, "ERROR") == 0) {
            out->aero_table_policy_override = AERO_DB_EXTRAPOLATION_ERROR;
        } else if (strcmp(aero_policy, "CLAMP_AND_WARN") == 0) {
            out->aero_table_policy_override = AERO_DB_EXTRAPOLATION_CLAMP_AND_WARN;
        } else if (strcmp(aero_policy, "HOLD_LAST_VALID") == 0) {
            out->aero_table_policy_override = AERO_DB_EXTRAPOLATION_HOLD_LAST_VALID;
        } else {
            return SIM_ERR_CONFIG;
        }
        out->aero_table_policy_override_enabled = 1;
    } else if (status == SIM_ERR_OUT_OF_RANGE) {
        return status;
    }
    {
        double minimum = 0.0;
        double maximum = 0.0;
        int min_found = 0;
        int max_found = 0;

        status = config_get_double(scenario, "aerodynamics.table_height_min_m", &minimum);
        if (status == SIM_OK) {
            min_found = 1;
        } else if (status == SIM_ERR_OUT_OF_RANGE) {
            return status;
        }
        status = config_get_double(scenario, "aerodynamics.table_height_max_m", &maximum);
        if (status == SIM_OK) {
            max_found = 1;
        } else if (status == SIM_ERR_OUT_OF_RANGE) {
            return status;
        }
        if (min_found != max_found) {
            return SIM_ERR_CONFIG;
        }
        if (min_found != 0) {
            if (!isfinite(minimum) || !isfinite(maximum) || maximum < minimum) {
                return SIM_ERR_OUT_OF_RANGE;
            }
            out->aero_table_height_envelope_enabled = 1;
            out->aero_table_height_min_m = minimum;
            out->aero_table_height_max_m = maximum;
        }

        min_found = 0;
        max_found = 0;
        status = config_get_double(scenario, "aerodynamics.table_actuator_min_rad", &minimum);
        if (status == SIM_OK) {
            min_found = 1;
        } else if (status == SIM_ERR_OUT_OF_RANGE) {
            return status;
        }
        status = config_get_double(scenario, "aerodynamics.table_actuator_max_rad", &maximum);
        if (status == SIM_OK) {
            max_found = 1;
        } else if (status == SIM_ERR_OUT_OF_RANGE) {
            return status;
        }
        if (min_found != max_found) {
            return SIM_ERR_CONFIG;
        }
        if (min_found != 0) {
            if (!isfinite(minimum) || !isfinite(maximum) || maximum < minimum) {
                return SIM_ERR_OUT_OF_RANGE;
            }
            out->aero_table_actuator_envelope_enabled = 1;
            out->aero_table_actuator_min_rad = minimum;
            out->aero_table_actuator_max_rad = maximum;
        }
    }
    status = config_get_string(
        scenario,
        "aerodynamics.surrogate_model_path",
        out->aero_surrogate_model_path,
        sizeof(out->aero_surrogate_model_path));
    if (status == SIM_OK) {
        out->aero_surrogate_model_path_enabled = 1;
        (void)snprintf(
            out->aero_surrogate_model_version,
            sizeof(out->aero_surrogate_model_version),
            "UNSPECIFIED");
        (void)snprintf(
            out->aero_surrogate_training_data_version,
            sizeof(out->aero_surrogate_training_data_version),
            "UNSPECIFIED");
        status = config_get_string(
            scenario,
            "aerodynamics.surrogate_model_version",
            out->aero_surrogate_model_version,
            sizeof(out->aero_surrogate_model_version));
        if (status == SIM_ERR_OUT_OF_RANGE) {
            return status;
        }
        status = config_get_string(
            scenario,
            "aerodynamics.surrogate_training_data_version",
            out->aero_surrogate_training_data_version,
            sizeof(out->aero_surrogate_training_data_version));
        if (status == SIM_ERR_OUT_OF_RANGE) {
            return status;
        }
    } else if (status == SIM_ERR_OUT_OF_RANGE) {
        return status;
    }
    if (out->aero_table_v2_path_enabled != 0 &&
        (out->aero_table_path_enabled != 0 ||
            out->aero_surrogate_model_path_enabled != 0)) {
        return SIM_ERR_CONFIG;
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
    {
        unsigned int cache_tile_count = ENV_MAX_TERRAIN_TILES;

        status = config_get_uint32(
            scenario,
            "map.terrain.cache_tile_count",
            &cache_tile_count);
        if (status != SIM_OK) {
            cache_tile_count = ENV_MAX_TERRAIN_TILES;
        }
        if (cache_tile_count == 0u || cache_tile_count > ENV_MAX_TERRAIN_TILES) {
            return SIM_ERR_OUT_OF_RANGE;
        }
        out->terrain_cache_tile_count = (size_t)cache_tile_count;
    }
    status = config_get_string(
        scenario,
        "map.resource_manifest_path",
        out->terrain_resource_manifest_path,
        sizeof(out->terrain_resource_manifest_path));
    if (status == SIM_OK) {
        FILE *manifest = fopen(out->terrain_resource_manifest_path, "rb");

        if (manifest == 0) {
            return SIM_ERR_IO;
        }
        (void)fclose(manifest);
        out->terrain_resource_manifest_path_enabled = 1;
    } else if (status == SIM_ERR_OUT_OF_RANGE) {
        return status;
    }
    status = config_get_string(
        scenario,
        "map.tile_path",
        out->terrain_tile_paths[out->terrain_tile_path_count],
        sizeof(out->terrain_tile_paths[out->terrain_tile_path_count]));
    if (status == SIM_OK) {
        ++out->terrain_tile_path_count;
    } else if (status == SIM_ERR_OUT_OF_RANGE) {
        return status;
    }
    {
        size_t tile_path_count = 0u;

        status = config_get_array_count(scenario, "map.tile_paths", &tile_path_count);
        if (status == SIM_OK) {
            size_t index;

            if (tile_path_count > ENV_MAX_TERRAIN_TILES ||
                out->terrain_tile_path_count + tile_path_count > ENV_MAX_TERRAIN_TILES) {
                return SIM_ERR_OUT_OF_RANGE;
            }
            for (index = 0u; index < tile_path_count; ++index) {
                char tile_path_key[64];
                char tile_path[512];

                (void)snprintf(
                    tile_path_key,
                    sizeof(tile_path_key),
                    "map.tile_paths[%zu]",
                    index);
                status = config_get_string(
                    scenario,
                    tile_path_key,
                    tile_path,
                    sizeof(tile_path));
                if (status != SIM_OK) {
                    return status;
                }
                status = append_terrain_tile_path(out, tile_path);
                if (status != SIM_OK) {
                    return status;
                }
            }
        }
    }
    status = config_get_string(
        scenario,
        "map.tile_index_path",
        out->terrain_tile_index_path,
        sizeof(out->terrain_tile_index_path));
    if (status == SIM_OK) {
        out->terrain_tile_index_path_enabled = 1;
        status = load_terrain_tile_index(out, out->terrain_tile_index_path);
        if (status != SIM_OK) {
            return status;
        }
    } else if (status == SIM_ERR_OUT_OF_RANGE) {
        return status;
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
    if (status == SIM_OK) {
        status = target_model_load_config(scenario, &out->target_model);
    }
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
    const SensorFrame *sensor,
    const FaultStepEffects *effects)
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
    if (effects != 0 && effects->communication_drop_enabled != 0) {
        return SIM_OK;
    }
    if (effects != 0 && effects->communication_corrupt_enabled != 0) {
        buffer[packet_size - 1u] ^= UINT8_C(0x01);
    }
    sent = sendto(sock, buffer, packet_size, 0, (const struct sockaddr *)peer, sizeof(*peer));
    if (sent != (ssize_t)packet_size) {
        return SIM_ERR_IO;
    }
    if (effects != 0 && effects->communication_duplicate_enabled != 0) {
        sent = sendto(sock, buffer, packet_size, 0, (const struct sockaddr *)peer, sizeof(*peer));
        if (sent != (ssize_t)packet_size) {
            return SIM_ERR_IO;
        }
    }
    return SIM_OK;
}

/** @brief 向飞控发送带 CRC 的本实例正常停止控制帧。 */
static SimStatus send_sim_stop(
    int sock,
    const struct sockaddr_in *peer,
    uint32_t instance_id,
    uint32_t seq,
    double sim_time)
{
    unsigned char buffer[SIM_SIM_CONTROL_PACKET_WIRE_SIZE];
    size_t packet_size = 0u;
    ssize_t sent;
    SimStatus status;

    if (peer == 0) {
        return SIM_ERR_INVALID_ARG;
    }
    status = packet_encode_sim_control(
        instance_id,
        seq,
        sim_time,
        SIM_CONTROL_STOP,
        buffer,
        sizeof(buffer),
        &packet_size);
    if (status != SIM_OK) {
        return status;
    }
    sent = sendto(sock, buffer, packet_size, 0, (const struct sockaddr *)peer, sizeof(*peer));
    return sent == (ssize_t)packet_size ? SIM_OK : SIM_ERR_IO;
}

/** @brief 清空通信层传感器帧延迟线。 */
static void communication_delay_reset(CommunicationDelayLine *delay)
{
    if (delay != 0) {
        (void)memset(delay, 0, sizeof(*delay));
    }
}

/** @brief 向固定容量延迟线追加一帧。 */
static SimStatus communication_delay_push(CommunicationDelayLine *delay, const SensorFrame *sensor)
{
    size_t tail;

    if (delay == 0 || sensor == 0) {
        return SIM_ERR_INVALID_ARG;
    }
    if (delay->count >= ENV_COMM_DELAY_BUFFER_CAPACITY) {
        return SIM_ERR_OUT_OF_RANGE;
    }
    tail = (delay->head + delay->count) % ENV_COMM_DELAY_BUFFER_CAPACITY;
    delay->frames[tail] = *sensor;
    ++delay->count;
    return SIM_OK;
}

/** @brief 从固定容量延迟线弹出最旧一帧。 */
static SimStatus communication_delay_pop(CommunicationDelayLine *delay, SensorFrame *sensor)
{
    if (delay == 0 || sensor == 0 || delay->count == 0u) {
        return SIM_ERR_INVALID_ARG;
    }
    *sensor = delay->frames[delay->head];
    delay->head = (delay->head + 1u) % ENV_COMM_DELAY_BUFFER_CAPACITY;
    --delay->count;
    return SIM_OK;
}

/** @brief 应用锁步安全的通信层帧延迟故障。 */
static SimStatus apply_communication_delay(
    CommunicationDelayLine *delay,
    const FaultStepEffects *effects,
    SensorFrame *sensor)
{
    SensorFrame current;

    if (delay == 0 || effects == 0 || sensor == 0) {
        return SIM_ERR_INVALID_ARG;
    }
    if (effects->communication_delay_enabled == 0) {
        communication_delay_reset(delay);
        return SIM_OK;
    }
    if (effects->communication_delay_steps == 0u ||
        effects->communication_delay_steps > ENV_MAX_COMMUNICATION_DELAY_STEPS) {
        return SIM_ERR_OUT_OF_RANGE;
    }
    current = *sensor;
    if (communication_delay_push(delay, &current) != SIM_OK) {
        return SIM_ERR_OUT_OF_RANGE;
    }
    if (delay->count > (size_t)effects->communication_delay_steps) {
        return communication_delay_pop(delay, sensor);
    }
    sensor->sensor_valid_flags &= ~(
        SIM_SENSOR_VALID_SEEKER |
        SIM_SENSOR_VALID_IMU_GYRO |
        SIM_SENSOR_VALID_ACCEL |
        SIM_SENSOR_VALID_SPEED);
    sensor->sensor_fault_flags |= SIM_SENSOR_FAULT_DELAY_WARMUP;
    return SIM_OK;
}

/** @brief 重置通信乱序/上一帧重放状态。 */
static void communication_reorder_reset(CommunicationReorderState *state)
{
    if (state != 0) {
        (void)memset(state, 0, sizeof(*state));
    }
}

/** @brief 应用锁步安全的通信乱序故障。 */
static SimStatus apply_communication_reorder(
    CommunicationReorderState *state,
    const FaultStepEffects *effects,
    SensorFrame *sensor)
{
    SensorFrame current;

    if (state == 0 || effects == 0 || sensor == 0) {
        return SIM_ERR_INVALID_ARG;
    }
    if (effects->communication_reorder_enabled == 0) {
        communication_reorder_reset(state);
        return SIM_OK;
    }
    current = *sensor;
    if (state->has_previous != 0) {
        *sensor = state->previous;
        state->previous = current;
        return SIM_OK;
    }
    state->previous = current;
    state->has_previous = 1;
    sensor->sensor_valid_flags &= ~(
        SIM_SENSOR_VALID_SEEKER |
        SIM_SENSOR_VALID_IMU_GYRO |
        SIM_SENSOR_VALID_ACCEL |
        SIM_SENSOR_VALID_SPEED);
    sensor->sensor_fault_flags |= SIM_SENSOR_FAULT_DELAY_WARMUP;
    return SIM_OK;
}

/** @brief 重置单轴执行机构命令延迟线。 */
static void actuator_command_delay_reset(ActuatorCommandDelayLine *delay)
{
    if (delay != 0) {
        (void)memset(delay, 0, sizeof(*delay));
    }
}

/** @brief 应用固定步数执行机构命令延迟，激活初期以中立命令填充。 */
static SimStatus apply_actuator_command_delay(
    ActuatorCommandDelayLine *delay,
    unsigned int delay_steps,
    double *command)
{
    size_t tail;
    double current;

    if (delay == 0 || command == 0 || !isfinite(*command)) {
        return SIM_ERR_INVALID_ARG;
    }
    if (delay_steps == 0u) {
        actuator_command_delay_reset(delay);
        return SIM_OK;
    }
    if (delay_steps > ENV_MAX_ACTUATOR_DELAY_STEPS ||
        delay->count >= ENV_ACTUATOR_DELAY_BUFFER_CAPACITY) {
        return SIM_ERR_OUT_OF_RANGE;
    }
    current = *command;
    tail = (delay->head + delay->count) % ENV_ACTUATOR_DELAY_BUFFER_CAPACITY;
    delay->commands[tail] = current;
    ++delay->count;
    if (delay->count > (size_t)delay_steps) {
        *command = delay->commands[delay->head];
        delay->head = (delay->head + 1u) % ENV_ACTUATOR_DELAY_BUFFER_CAPACITY;
        --delay->count;
    } else {
        *command = 0.0;
    }
    return SIM_OK;
}

/** @brief 返回故障测量通道对应的 SensorFrame 有效位。 */
static uint32_t sensor_fault_channel_valid_mask(size_t channel)
{
    if (channel <= 3u) {
        return SIM_SENSOR_VALID_SEEKER;
    }
    if (channel == 4u) {
        return SIM_SENSOR_VALID_IMU_GYRO;
    }
    if (channel == 5u) {
        return SIM_SENSOR_VALID_ACCEL;
    }
    return channel == 6u ? SIM_SENSOR_VALID_SPEED : 0u;
}

/** @brief 仅复制一个故障测量通道，不改变当前帧序号和时间戳。 */
static SimStatus copy_sensor_fault_channel(
    SensorFrame *destination,
    const SensorFrame *source,
    size_t channel)
{
    if (destination == 0 || source == 0 ||
        channel >= ENV_SENSOR_FAULT_CHANNEL_COUNT) {
        return SIM_ERR_INVALID_ARG;
    }
    if (channel == 0u) {
        destination->target_range_meas = source->target_range_meas;
    } else if (channel == 1u) {
        destination->target_los_unit_ecef_meas = source->target_los_unit_ecef_meas;
    } else if (channel == 2u) {
        destination->target_los_rate_ecef_meas = source->target_los_rate_ecef_meas;
    } else if (channel == 3u) {
        destination->target_closing_velocity_meas = source->target_closing_velocity_meas;
    } else if (channel == 4u) {
        destination->missile_gyro_b_meas = source->missile_gyro_b_meas;
    } else if (channel == 5u) {
        destination->missile_accel_ecef_meas = source->missile_accel_ecef_meas;
    } else {
        destination->missile_vel_ecef_meas = source->missile_vel_ecef_meas;
    }
    return SIM_OK;
}

/** @brief 应用传感器故障起点捕获卡滞。 */
static SimStatus apply_sensor_fault_stuck(
    SensorFaultStuckState states[ENV_SENSOR_FAULT_CHANNEL_COUNT],
    const FaultStepEffects *effects,
    SensorFrame *sensor)
{
    size_t channel;

    if (states == 0 || effects == 0 || sensor == 0) {
        return SIM_ERR_INVALID_ARG;
    }
    for (channel = 0u; channel < ENV_SENSOR_FAULT_CHANNEL_COUNT; ++channel) {
        if (effects->sensor_stuck_enabled[channel] == 0) {
            states[channel].active = 0;
            continue;
        }
        if (states[channel].active == 0) {
            states[channel].held = *sensor;
            states[channel].active = 1;
        } else if (copy_sensor_fault_channel(sensor, &states[channel].held, channel) != SIM_OK) {
            return SIM_ERR_INTERNAL;
        }
    }
    return SIM_OK;
}

/** @brief 应用逐测量通道固定步数延迟。 */
static SimStatus apply_sensor_fault_delay(
    SensorFaultDelayLine lines[ENV_SENSOR_FAULT_CHANNEL_COUNT],
    const FaultStepEffects *effects,
    SensorFrame *sensor)
{
    size_t channel;

    if (lines == 0 || effects == 0 || sensor == 0) {
        return SIM_ERR_INVALID_ARG;
    }
    for (channel = 0u; channel < ENV_SENSOR_FAULT_CHANNEL_COUNT; ++channel) {
        SensorFaultDelayLine *line = &lines[channel];
        const unsigned int delay_steps = effects->sensor_delay_steps[channel];
        size_t tail;

        if (delay_steps == 0u) {
            (void)memset(line, 0, sizeof(*line));
            continue;
        }
        if (delay_steps > ENV_MAX_SENSOR_DELAY_STEPS ||
            line->count >= ENV_SENSOR_FAULT_DELAY_BUFFER_CAPACITY) {
            return SIM_ERR_OUT_OF_RANGE;
        }
        tail = (line->head + line->count) % ENV_SENSOR_FAULT_DELAY_BUFFER_CAPACITY;
        line->frames[tail] = *sensor;
        ++line->count;
        if (line->count > (size_t)delay_steps) {
            if (copy_sensor_fault_channel(sensor, &line->frames[line->head], channel) != SIM_OK) {
                return SIM_ERR_INTERNAL;
            }
            line->head = (line->head + 1u) % ENV_SENSOR_FAULT_DELAY_BUFFER_CAPACITY;
            --line->count;
        } else {
            sensor->sensor_valid_flags &= ~sensor_fault_channel_valid_mask(channel);
            sensor->sensor_fault_flags |= SIM_SENSOR_FAULT_DELAY_WARMUP;
        }
    }
    return SIM_OK;
}

/** @brief 接收并校验飞控控制指令。 */
static SimStatus receive_control_command(
    int sock,
    uint32_t instance_id,
    const struct sockaddr_in *expected_peer,
    size_t *minor_mismatch_count,
    ControlCommand *command)
{
    unsigned char buffer[ENV_PACKET_BUFFER_SIZE];
    struct sockaddr_in from;
    socklen_t from_len = sizeof(from);
    ssize_t got = recvfrom(sock, buffer, sizeof(buffer), 0, (struct sockaddr *)&from, &from_len);

    PacketHeader header;
    SimStatus status;

    if (expected_peer == 0 || minor_mismatch_count == 0 || command == 0) {
        return SIM_ERR_INVALID_ARG;
    }
    if (got < 0) {
        return errno == EAGAIN || errno == EWOULDBLOCK ? SIM_ERR_TIMEOUT : SIM_ERR_IO;
    }
    if (from.sin_family != expected_peer->sin_family ||
        from.sin_port != expected_peer->sin_port ||
        from.sin_addr.s_addr != expected_peer->sin_addr.s_addr) {
        return SIM_ERR_BAD_PACKET;
    }
    status = packet_peek_header(buffer, (size_t)got, &header);
    if (status != SIM_OK) {
        return status;
    }
    if (header.version_minor != MISSILE_SIM_PROTOCOL_VERSION_MINOR) {
        ++*minor_mismatch_count;
    }
    return packet_decode_control_command(buffer, (size_t)got, instance_id, command);
}

/** @brief 非阻塞排空有限数量控制帧，并保留不晚于当前传感器序号的最新命令。 */
static SimStatus receive_latest_control_command(
    int sock,
    uint32_t instance_id,
    uint32_t current_sensor_seq,
    const struct sockaddr_in *expected_peer,
    size_t *minor_mismatch_count,
    ControlCommand *latest_command,
    int *received_new_command)
{
    size_t drained_count;

    if (expected_peer == 0 || minor_mismatch_count == 0 ||
        latest_command == 0 || received_new_command == 0) {
        return SIM_ERR_INVALID_ARG;
    }
    *received_new_command = 0;
    for (drained_count = 0u;
         drained_count < ENV_MAX_DRAINED_COMMANDS_PER_STEP;
         ++drained_count) {
        unsigned char buffer[ENV_PACKET_BUFFER_SIZE];
        struct sockaddr_in from;
        socklen_t from_len = sizeof(from);
        ControlCommand candidate;
        PacketHeader header;
        ssize_t got = recvfrom(
            sock,
            buffer,
            sizeof(buffer),
            MSG_DONTWAIT,
            (struct sockaddr *)&from,
            &from_len);
        SimStatus status;

        if (got < 0) {
            return errno == EAGAIN || errno == EWOULDBLOCK ? SIM_OK : SIM_ERR_IO;
        }
        if (from.sin_family != expected_peer->sin_family ||
            from.sin_port != expected_peer->sin_port ||
            from.sin_addr.s_addr != expected_peer->sin_addr.s_addr) {
            return SIM_ERR_BAD_PACKET;
        }
        status = packet_peek_header(buffer, (size_t)got, &header);
        if (status == SIM_OK && header.version_minor != MISSILE_SIM_PROTOCOL_VERSION_MINOR) {
            ++*minor_mismatch_count;
        }
        if (status == SIM_OK) {
            status = packet_decode_control_command(
            buffer,
            (size_t)got,
            instance_id,
            &candidate);
        }
        if (status != SIM_OK || candidate.seq > current_sensor_seq) {
            return status == SIM_OK ? SIM_ERR_BAD_PACKET : status;
        }
        if (candidate.seq >= latest_command->seq) {
            *latest_command = candidate;
            *received_new_command = 1;
        }
    }
    return SIM_OK;
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
    const FaultStepEffects *fault_effects,
    ActuatorCommandDelayLine actuator_delays[3])
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

    if (state == 0 || command == 0 || cfg == 0 || force_model == 0 ||
        actuator_delays == 0) {
        return SIM_ERR_INVALID_ARG;
    }
    if (fault_effects != 0) {
        fault_injection_apply_actuators(
            fault_effects,
            state->acceleration_actuators,
            commands);
    }
    for (index = 0u; index < 3u && status == SIM_OK; ++index) {
        ActuatorState *actuator = &state->acceleration_actuators[index];
        const double original_rate_limit = actuator->rate_limit;
        const double original_pos_min = actuator->pos_min;
        const double original_pos_max = actuator->pos_max;

        if (fault_effects != 0) {
            status = apply_actuator_command_delay(
                &actuator_delays[index],
                fault_effects->actuator_delay_steps[index],
                &commands[index]);
            actuator->rate_limit = original_rate_limit *
                fault_effects->actuator_rate_limit_scale[index];
            actuator->pos_min = original_pos_min *
                fault_effects->actuator_position_limit_scale[index];
            actuator->pos_max = original_pos_max *
                fault_effects->actuator_position_limit_scale[index];
        }
        if (status == SIM_OK) {
            status = actuator_model_step(actuator, commands[index], cfg->dt);
        }
        actuator->rate_limit = original_rate_limit;
        actuator->pos_min = original_pos_min;
        actuator->pos_max = original_pos_max;
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
    force_input.pitch_actuator_rad = command->actuator_cmd[0];
    force_input.yaw_actuator_rad = command->actuator_cmd[1];
    force_input.propellant_mass_kg = state->missile_mass.propellant_mass_kg;
    force_input.dt_s = cfg->dt;
    status = wind_model_step(
        &state->wind_model,
        state->time,
        state->missile_lla.height_m,
        cfg->dt,
        &state->wind_velocity_ecef_mps);
    if (status != SIM_OK) {
        return status;
    }
    force_input.wind_velocity_ecef_mps = state->wind_velocity_ecef_mps;
    status = environment_force_model_evaluate(
        force_model,
        &force_input,
        &force_output);
    if (status != SIM_OK) {
        return status;
    }
    state->aero_model_flags = force_output.aerodynamic_model_flags;
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
    if (state->missile_mass.properties_enabled != 0) {
        Vec3 center_of_mass_b_m;

        status = mass_model_get_properties(
            &state->missile_mass,
            &center_of_mass_b_m,
            &state->missile_plant.inertia_b);
        if (status != SIM_OK) {
            return status;
        }
    } else if (state->initial_mass_kg > 0.0) {
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
    status = target_model_step(
        &cfg->target_model,
        state->time,
        cfg->dt,
        &state->target_pos,
        &state->target_vel);
    if (status != SIM_OK) {
        return status;
    }
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

/** @brief 将地形缺瓦片策略转为清单中的稳定字符串。 */
static const char *terrain_missing_policy_name(TerrainMissingPolicy policy)
{
    switch (policy) {
    case MAP_MISSING_ERROR:
        return "ERROR";
    case MAP_MISSING_FLAT_FILL:
        return "FLAT_FILL";
    case MAP_MISSING_NEAREST:
        return "NEAREST";
    default:
        return "UNKNOWN";
    }
}

/** @brief 将气动表包络外策略转为清单中的稳定字符串。 */
static const char *aero_extrapolation_policy_name(AeroDatabaseExtrapolationPolicy policy)
{
    switch (policy) {
    case AERO_DB_EXTRAPOLATION_ERROR:
        return "ERROR";
    case AERO_DB_EXTRAPOLATION_CLAMP_AND_WARN:
        return "CLAMP_AND_WARN";
    case AERO_DB_EXTRAPOLATION_HOLD_LAST_VALID:
        return "HOLD_LAST_VALID";
    default:
        return "UNKNOWN";
    }
}

/** @brief 计算原始配置文件字节的 CRC32。 */
static SimStatus config_file_crc32(const char *path, uint32_t *crc_out)
{
    ConfigTree config;
    SimStatus status;

    if (path == 0 || crc_out == 0) {
        return SIM_ERR_INVALID_ARG;
    }
    (void)memset(&config, 0, sizeof(config));
    status = config_load_file(path, &config);
    if (status != SIM_OK) {
        return status;
    }
    *crc_out = crc32_compute(config.data, config.size);
    config_free(&config);
    return SIM_OK;
}

/** @brief 将配置原始字节复制到实例证据目录。 */
static SimStatus copy_file_bytes(const char *source_path, const char *destination_path)
{
    FILE *source;
    FILE *destination;
    unsigned char buffer[4096];
    size_t size;
    SimStatus status = SIM_OK;

    if (source_path == 0 || destination_path == 0) {
        return SIM_ERR_INVALID_ARG;
    }
    source = fopen(source_path, "rb");
    if (source == 0) {
        return SIM_ERR_IO;
    }
    destination = fopen(destination_path, "wb");
    if (destination == 0) {
        (void)fclose(source);
        return SIM_ERR_IO;
    }
    while ((size = fread(buffer, 1u, sizeof(buffer), source)) > 0u) {
        if (fwrite(buffer, 1u, size, destination) != size) {
            status = SIM_ERR_IO;
            break;
        }
    }
    if (ferror(source) != 0) {
        status = SIM_ERR_IO;
    }
    if (fclose(source) != 0) {
        status = SIM_ERR_IO;
    }
    if (fclose(destination) != 0) {
        status = SIM_ERR_IO;
    }
    return status;
}

/** @brief 保存三份运行配置的逐字节快照。 */
static SimStatus write_config_snapshots(const char *instance_dir, const EnvContext *ctx)
{
    char path[1024];
    SimStatus status;

    if (instance_dir == 0 || ctx == 0) {
        return SIM_ERR_INVALID_ARG;
    }
    (void)snprintf(path, sizeof(path), "%s/config_snapshot_scenario.json", instance_dir);
    status = copy_file_bytes(ctx->scenario_path, path);
    if (status != SIM_OK) {
        return status;
    }
    (void)snprintf(path, sizeof(path), "%s/config_snapshot_runtime.json", instance_dir);
    status = copy_file_bytes(ctx->runtime_path, path);
    if (status != SIM_OK) {
        return status;
    }
    (void)snprintf(path, sizeof(path), "%s/config_snapshot_faults.json", instance_dir);
    return copy_file_bytes(ctx->faults_path, path);
}

/** @brief 生成 UTC 墙钟启动时间。 */
static void format_wall_clock_utc(char *out, size_t out_size)
{
    const time_t now = time(0);
    const struct tm *utc = now == (time_t)-1 ? 0 : gmtime(&now);

    if (out == 0 || out_size == 0u) {
        return;
    }
    if (utc == 0 || strftime(out, out_size, "%Y-%m-%dT%H:%M:%SZ", utc) == 0u) {
        (void)snprintf(out, out_size, "unknown");
    }
}

/** @brief 写出可复现实例所需的软件、配置、端口和时间参数。 */
static SimStatus write_run_manifest(
    const char *instance_dir,
    const EnvContext *ctx,
    const EnvScenarioConfig *scenario,
    const EnvRuntimeConfig *runtime,
    const char *start_time_wall_clock,
    uint64_t instance_random_seed,
    unsigned int env_port,
    unsigned int fc_port,
    const ConfigWarningCollector *config_warnings)
{
    char path[1024];
    FILE *file;
    size_t tile_index;
    uint32_t scenario_crc;
    uint32_t runtime_crc;
    uint32_t faults_crc;

    if (config_file_crc32(ctx->scenario_path, &scenario_crc) != SIM_OK ||
        config_file_crc32(ctx->runtime_path, &runtime_crc) != SIM_OK ||
        config_file_crc32(ctx->faults_path, &faults_crc) != SIM_OK) {
        return SIM_ERR_IO;
    }

    (void)snprintf(path, sizeof(path), "%s/run_manifest.json", instance_dir);
    file = fopen(path, "wb");
    if (file == 0) {
        return SIM_ERR_IO;
    }
    (void)fprintf(file, "{\n");
    (void)fprintf(file, "  \"schema_version\": 1,\n");
    (void)fprintf(file, "  \"instance_id\": %u,\n", ctx->instance_id);
    (void)fprintf(file, "  \"campaign_id\": \"%s\",\n", runtime->campaign_id);
    (void)fprintf(file, "  \"software_version\": \"%d.%d.%d\",\n",
        MISSILE_SIM_VERSION_MAJOR,
        MISSILE_SIM_VERSION_MINOR,
        MISSILE_SIM_VERSION_PATCH);
    (void)fprintf(
        file,
        "  \"program_version\": \"environment_sim %d.%d.%d\",\n",
        MISSILE_SIM_VERSION_MAJOR,
        MISSILE_SIM_VERSION_MINOR,
        MISSILE_SIM_VERSION_PATCH);
    (void)fprintf(file, "  \"git_commit\": \"%s\",\n", MISSILE_SIM_GIT_COMMIT);
    (void)fprintf(
        file,
        "  \"git_worktree_dirty\": %s,\n",
        MISSILE_SIM_GIT_DIRTY != 0 ? "true" : "false");
    (void)fprintf(file, "  \"build_time\": \"%s\",\n", MISSILE_SIM_BUILD_TIME);
    (void)fprintf(file, "  \"compiler\": \"%s\",\n", MISSILE_SIM_COMPILER);
    (void)fprintf(file, "  \"start_time_wall_clock\": \"%s\",\n", start_time_wall_clock);
    (void)fprintf(file, "  \"protocol_version\": \"%d.%d\",\n",
        MISSILE_SIM_PROTOCOL_VERSION_MAJOR,
        MISSILE_SIM_PROTOCOL_VERSION_MINOR);
    (void)fprintf(file, "  \"scenario_path\": \"%s\",\n", ctx->scenario_path);
    (void)fprintf(file, "  \"runtime_path\": \"%s\",\n", ctx->runtime_path);
    (void)fprintf(file, "  \"faults_path\": \"%s\",\n", ctx->faults_path);
    (void)fprintf(
        file,
        "  \"config_file_list\": [\"%s\", \"%s\", \"%s\"],\n",
        ctx->scenario_path,
        ctx->runtime_path,
        ctx->faults_path);
    (void)fprintf(file, "  \"config_crc32\": {\n");
    (void)fprintf(file, "    \"scenario\": \"0x%08x\",\n", scenario_crc);
    (void)fprintf(file, "    \"runtime\": \"0x%08x\",\n", runtime_crc);
    (void)fprintf(file, "    \"faults\": \"0x%08x\"\n", faults_crc);
    (void)fprintf(file, "  },\n");
    (void)fprintf(file, "  \"config_snapshots\": {\n");
    (void)fprintf(file, "    \"scenario\": \"%s/config_snapshot_scenario.json\",\n", instance_dir);
    (void)fprintf(file, "    \"runtime\": \"%s/config_snapshot_runtime.json\",\n", instance_dir);
    (void)fprintf(file, "    \"faults\": \"%s/config_snapshot_faults.json\"\n", instance_dir);
    (void)fprintf(file, "  },\n");
    (void)fprintf(
        file,
        "  \"unrecognized_config_field_count\": %lu,\n",
        config_warnings != 0 ? (unsigned long)config_warnings->total_count : 0ul);
    (void)fprintf(file, "  \"unrecognized_config_fields\": [");
    if (config_warnings != 0) {
        size_t warning_index;

        for (warning_index = 0u;
             warning_index < config_warnings->stored_count;
             ++warning_index) {
            (void)fprintf(
                file,
                "%s\"%s\"",
                warning_index == 0u ? "" : ", ",
                config_warnings->paths[warning_index]);
        }
    }
    (void)fprintf(file, "],\n");
    (void)fprintf(
        file,
        "  \"run_mode\": \"%s\",\n",
        env_run_mode_to_string(runtime->run_mode));
    (void)fprintf(
        file,
        "  \"synchronization_mode\": \"%s\",\n",
        env_synchronization_mode_to_string(runtime->synchronization_mode));
    (void)fprintf(file, "  \"output_dir\": \"%s\",\n", runtime->output_dir);
    (void)fprintf(file, "  \"instance_dir\": \"%s\",\n", instance_dir);
    (void)fprintf(
        file,
        "  \"integrator_type\": \"%s\",\n",
        integrator_type_to_string(scenario->integrator));
    (void)fprintf(
        file,
        "  \"target_model\": \"%s\",\n",
        target_model_type_to_string(scenario->target_model.type));
    (void)fprintf(
        file,
        "  \"target_maneuver_count\": %zu,\n",
        scenario->target_model.maneuver_count);
    (void)fprintf(file, "  \"log_files\": {\n");
    (void)fprintf(file, "    \"sensor\": \"%s/sensor_log.bin\",\n", instance_dir);
    (void)fprintf(file, "    \"command\": \"%s/command_log.bin\",\n", instance_dir);
    (void)fprintf(file, "    \"flight_control_internal\": \"%s/fc_internal_log.bin\",\n", instance_dir);
    (void)fprintf(file, "    \"flight_control_manifest\": \"%s/fc_run_manifest.json\",\n", instance_dir);
    (void)fprintf(file, "    \"truth_trajectory\": \"%s/trajectory.csv\",\n", instance_dir);
    (void)fprintf(file, "    \"diagnostics\": \"%s/trajectory_diagnostics.csv\",\n", instance_dir);
    (void)fprintf(file, "    \"events\": \"%s/event_log.txt\",\n", instance_dir);
    (void)fprintf(file, "    \"summary\": \"%s/summary.json\",\n", instance_dir);
    (void)fprintf(file, "    \"environment_performance\": \"%s/performance.json\",\n", instance_dir);
    (void)fprintf(file, "    \"flight_control_performance\": \"%s/fc_performance.json\"\n", instance_dir);
    (void)fprintf(file, "  },\n");
    (void)fprintf(file, "  \"dt_s\": %.17g,\n", scenario->dt);
    (void)fprintf(file, "  \"max_time_s\": %.17g,\n", scenario->max_time);
    (void)fprintf(file, "  \"environment_port\": %u,\n", env_port);
    (void)fprintf(file, "  \"flight_control_port\": %u,\n", fc_port);
    (void)fprintf(
        file,
        "  \"random_seed\": %llu,\n",
        (unsigned long long)instance_random_seed);
    (void)fprintf(file, "  \"initial_mass_kg\": %.17g,\n", scenario->mass_kg);
    (void)fprintf(
        file,
        "  \"initial_propellant_mass_kg\": %.17g,\n",
        scenario->propellant_mass_kg);
    (void)fprintf(
        file,
        "  \"mass_properties_enabled\": %s,\n",
        scenario->mass_properties_enabled != 0 ? "true" : "false");
    if (scenario->mass_properties_enabled != 0) {
        (void)fprintf(
            file,
            "  \"dry_center_of_mass_b_m\": [%.17g, %.17g, %.17g],\n",
            scenario->dry_center_of_mass_b_m.x,
            scenario->dry_center_of_mass_b_m.y,
            scenario->dry_center_of_mass_b_m.z);
        (void)fprintf(
            file,
            "  \"propellant_center_of_mass_full_b_m\": [%.17g, %.17g, %.17g],\n",
            scenario->propellant_center_of_mass_full_b_m.x,
            scenario->propellant_center_of_mass_full_b_m.y,
            scenario->propellant_center_of_mass_full_b_m.z);
        (void)fprintf(
            file,
            "  \"propellant_center_of_mass_empty_b_m\": [%.17g, %.17g, %.17g],\n",
            scenario->propellant_center_of_mass_empty_b_m.x,
            scenario->propellant_center_of_mass_empty_b_m.y,
            scenario->propellant_center_of_mass_empty_b_m.z);
    }
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
        "  \"wind_model_enabled\": %s,\n",
        scenario->wind_model.enabled != 0 ? "true" : "false");
    (void)fprintf(
        file,
        "  \"wind_turbulence_sigma_ecef_mps\": [%.17g, %.17g, %.17g],\n",
        scenario->wind_model.turbulence_sigma_ecef_mps.x,
        scenario->wind_model.turbulence_sigma_ecef_mps.y,
        scenario->wind_model.turbulence_sigma_ecef_mps.z);
    (void)fprintf(
        file,
        "  \"wind_shear_ecef_per_m\": [%.17g, %.17g, %.17g],\n",
        scenario->wind_model.shear_ecef_per_m.x,
        scenario->wind_model.shear_ecef_per_m.y,
        scenario->wind_model.shear_ecef_per_m.z);
    (void)fprintf(
        file,
        "  \"wind_gust_amplitude_ecef_mps\": [%.17g, %.17g, %.17g],\n",
        scenario->wind_model.gust_amplitude_ecef_mps.x,
        scenario->wind_model.gust_amplitude_ecef_mps.y,
        scenario->wind_model.gust_amplitude_ecef_mps.z);
    (void)fprintf(
        file,
        "  \"aerodynamics_enabled\": %s,\n",
        scenario->force_model.aerodynamics.enabled != 0 ? "true" : "false");
    (void)fprintf(
        file,
        "  \"aero_table_enabled\": %s,\n",
        scenario->aero_table_path_enabled != 0 ? "true" : "false");
    if (scenario->aero_table_path_enabled != 0) {
        (void)fprintf(file, "  \"aero_table_path\": \"%s\",\n", scenario->aero_table_path);
        (void)fprintf(file, "  \"aero_table_file_version\": %u,\n", AERO_DATABASE_VERSION);
        (void)fprintf(
            file,
            "  \"aero_table_extrapolation_policy_source\": \"%s\",\n",
            scenario->aero_table_policy_override_enabled != 0 ? "CONFIG_OVERRIDE" : "FILE_HEADER");
        if (scenario->aero_table_policy_override_enabled != 0) {
            (void)fprintf(
                file,
                "  \"aero_table_extrapolation_policy\": \"%s\",\n",
                aero_extrapolation_policy_name(scenario->aero_table_policy_override));
        }
        (void)fprintf(
            file,
            "  \"aero_table_height_envelope_enabled\": %s,\n",
            scenario->aero_table_height_envelope_enabled != 0 ? "true" : "false");
        if (scenario->aero_table_height_envelope_enabled != 0) {
            (void)fprintf(
                file,
                "  \"aero_table_height_min_m\": %.17g,\n",
                scenario->aero_table_height_min_m);
            (void)fprintf(
                file,
                "  \"aero_table_height_max_m\": %.17g,\n",
                scenario->aero_table_height_max_m);
        }
        (void)fprintf(
            file,
            "  \"aero_table_actuator_envelope_enabled\": %s,\n",
            scenario->aero_table_actuator_envelope_enabled != 0 ? "true" : "false");
        if (scenario->aero_table_actuator_envelope_enabled != 0) {
            (void)fprintf(
                file,
                "  \"aero_table_actuator_min_rad\": %.17g,\n",
                scenario->aero_table_actuator_min_rad);
            (void)fprintf(
                file,
                "  \"aero_table_actuator_max_rad\": %.17g,\n",
                scenario->aero_table_actuator_max_rad);
        }
    }
    (void)fprintf(
        file,
        "  \"aero_table_v2_enabled\": %s,\n",
        scenario->aero_table_v2_path_enabled != 0 ? "true" : "false");
    if (scenario->aero_table_v2_path_enabled != 0) {
        (void)fprintf(
            file,
            "  \"aero_table_v2_path\": \"%s\",\n",
            scenario->aero_table_v2_path);
        (void)fprintf(
            file,
            "  \"aero_table_v2_file_version\": %u,\n",
            AERO_DATABASE_V2_VERSION);
        (void)fprintf(
            file,
            "  \"aero_table_v2_dimensions\": [%zu, %zu, %zu, %zu, %zu, %zu],\n",
            scenario->aero_table_v2_dimensions[0],
            scenario->aero_table_v2_dimensions[1],
            scenario->aero_table_v2_dimensions[2],
            scenario->aero_table_v2_dimensions[3],
            scenario->aero_table_v2_dimensions[4],
            scenario->aero_table_v2_dimensions[5]);
        (void)fprintf(
            file,
            "  \"aero_table_v2_extrapolation_policy_source\": \"%s\",\n",
            scenario->aero_table_policy_override_enabled != 0 ?
                "CONFIG_OVERRIDE" : "FILE_HEADER");
        if (scenario->aero_table_policy_override_enabled != 0) {
            (void)fprintf(
                file,
                "  \"aero_table_v2_extrapolation_policy\": \"%s\",\n",
                aero_extrapolation_policy_name(scenario->aero_table_policy_override));
        }
    }
    (void)fprintf(
        file,
        "  \"aero_surrogate_enabled\": %s,\n",
        scenario->aero_surrogate_model_path_enabled != 0 ? "true" : "false");
    if (scenario->aero_surrogate_model_path_enabled != 0) {
        const AeroSurrogateModel *surrogate =
            scenario->force_model.aerodynamics.surrogate;

        (void)fprintf(file, "  \"aero_surrogate_model_path\": \"%s\",\n", scenario->aero_surrogate_model_path);
        (void)fprintf(
            file,
            "  \"aero_surrogate_model_version\": \"%s\",\n",
            scenario->aero_surrogate_model_version);
        (void)fprintf(
            file,
            "  \"aero_surrogate_training_data_version\": \"%s\",\n",
            scenario->aero_surrogate_training_data_version);
        (void)fprintf(
            file,
            "  \"aero_surrogate_envelope_available\": %s,\n",
            surrogate != 0 ? "true" : "false");
        if (surrogate != 0) {
            (void)fprintf(
                file,
                "  \"aero_surrogate_envelope\": { \"mach\": [%.17g, %.17g], "
                "\"alpha_rad\": [%.17g, %.17g], \"beta_rad\": [%.17g, %.17g] },\n",
                surrogate->mach_min,
                surrogate->mach_max,
                surrogate->alpha_min_rad,
                surrogate->alpha_max_rad,
                surrogate->beta_min_rad,
                surrogate->beta_max_rad);
        }
    }
    (void)fprintf(
        file,
        "  \"propulsion_enabled\": %s,\n",
        scenario->force_model.propulsion.enabled != 0 ? "true" : "false");
    (void)fprintf(
        file,
        "  \"earth_rotation_enabled\": %s,\n",
        scenario->force_model.enable_earth_rotation != 0 ? "true" : "false");
    (void)fprintf(
        file,
        "  \"terrain_enabled\": %s,\n",
        scenario->terrain_enabled != 0 ? "true" : "false");
    (void)fprintf(
        file,
        "  \"terrain_los_occlusion_enabled\": %s,\n",
        scenario->los_occlusion_enabled != 0 ? "true" : "false");
    (void)fprintf(
        file,
        "  \"terrain_missing_policy\": \"%s\",\n",
        terrain_missing_policy_name(scenario->terrain_missing_policy));
    (void)fprintf(file, "  \"terrain_flat_fill_height_m\": %.17g,\n", scenario->terrain_flat_fill_height_m);
    (void)fprintf(file, "  \"terrain_cache_tile_count\": %zu,\n", scenario->terrain_cache_tile_count);
    (void)fprintf(
        file,
        "  \"terrain_resource_manifest_path_enabled\": %s,\n",
        scenario->terrain_resource_manifest_path_enabled != 0 ? "true" : "false");
    if (scenario->terrain_resource_manifest_path_enabled != 0) {
        (void)fprintf(
            file,
            "  \"terrain_resource_manifest_path\": \"%s\",\n",
            scenario->terrain_resource_manifest_path);
    }
    (void)fprintf(
        file,
        "  \"terrain_tile_index_path_enabled\": %s,\n",
        scenario->terrain_tile_index_path_enabled != 0 ? "true" : "false");
    if (scenario->terrain_tile_index_path_enabled != 0) {
        (void)fprintf(file, "  \"terrain_tile_index_path\": \"%s\",\n", scenario->terrain_tile_index_path);
    }
    (void)fprintf(file, "  \"terrain_tile_path_count\": %zu,\n", scenario->terrain_tile_path_count);
    (void)fprintf(file, "  \"terrain_tile_paths\": [");
    for (tile_index = 0u; tile_index < scenario->terrain_tile_path_count; ++tile_index) {
        (void)fprintf(
            file,
            "%s\"%s\"",
            tile_index == 0u ? "" : ", ",
            scenario->terrain_tile_paths[tile_index]);
    }
    (void)fprintf(file, "],\n");
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
        vec3_norm(effects->speed_bias_ecef_mps) > 0.0 ||
        effects->communication_delay_enabled != 0 ||
        effects->communication_reorder_enabled != 0 ||
        effects->communication_drop_enabled != 0 ||
        effects->communication_duplicate_enabled != 0 ||
        effects->communication_corrupt_enabled != 0;
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
            effects->actuator_command_scale[index] != 1.0 ||
            effects->actuator_command_bias[index] != 0.0 ||
            effects->actuator_rate_limit_scale[index] != 1.0 ||
            effects->actuator_position_limit_scale[index] != 1.0 ||
            effects->actuator_delay_steps[index] != 0u ||
            effects->actuator_disabled[index] != 0) {
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
    const OperationalRunStats *operational_stats,
    const FaultRunStats *fault_stats,
    const TerrainTileCache *terrain_cache,
    const DiagnosticRunStats *diagnostic_stats)
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
    (void)fprintf(
        file,
        "  \"max_command_norm\": %.9f,\n",
        operational_stats != 0 ? operational_stats->max_command_norm : 0.0);
    (void)fprintf(
        file,
        "  \"max_actual_accel\": %.9f,\n",
        operational_stats != 0 ? operational_stats->max_actual_accel : 0.0);
    (void)fprintf(
        file,
        "  \"sensor_dropout_count\": %lu,\n",
        operational_stats != 0 ? (unsigned long)operational_stats->sensor_dropout_count : 0ul);
    (void)fprintf(
        file,
        "  \"command_timeout_count\": %lu,\n",
        operational_stats != 0 ? (unsigned long)operational_stats->command_timeout_count : 0ul);
    (void)fprintf(
        file,
        "  \"command_hold_count\": %lu,\n",
        operational_stats != 0 ? (unsigned long)operational_stats->command_hold_count : 0ul);
    (void)fprintf(
        file,
        "  \"protocol_minor_mismatch_count\": %lu,\n",
        operational_stats != 0 ?
            (unsigned long)operational_stats->protocol_minor_mismatch_count : 0ul);
    (void)fprintf(
        file,
        "  \"fault_count\": %lu,\n",
        fault_stats != 0 ? (unsigned long)fault_stats->fault_start_count : 0ul);
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
            "  \"fault_max_concurrent_active\": %lu,\n",
            (unsigned long)fault_stats->max_concurrent_active);
    } else {
        (void)fprintf(file, "  \"fault_configured_count\": 0,\n");
        (void)fprintf(file, "  \"fault_start_count\": 0,\n");
        (void)fprintf(file, "  \"fault_end_count\": 0,\n");
        (void)fprintf(file, "  \"fault_active_step_count\": 0,\n");
        (void)fprintf(file, "  \"fault_sensor_affected_step_count\": 0,\n");
        (void)fprintf(file, "  \"fault_actuator_affected_step_count\": 0,\n");
        (void)fprintf(file, "  \"fault_max_concurrent_active\": 0,\n");
    }
    if (terrain_cache != 0) {
        (void)fprintf(
            file,
            "  \"terrain_cache_path_count\": %lu,\n",
            (unsigned long)terrain_cache->path_count);
        (void)fprintf(
            file,
            "  \"terrain_cache_capacity\": %lu,\n",
            (unsigned long)terrain_cache->capacity);
        (void)fprintf(
            file,
            "  \"terrain_cache_loaded_count\": %lu,\n",
            (unsigned long)terrain_cache->loaded_count);
        (void)fprintf(
            file,
            "  \"terrain_cache_load_count\": %llu,\n",
            (unsigned long long)terrain_cache->load_count);
        (void)fprintf(
            file,
            "  \"terrain_cache_eviction_count\": %llu,\n",
            (unsigned long long)terrain_cache->eviction_count);
    } else {
        (void)fprintf(file, "  \"terrain_cache_path_count\": 0,\n");
        (void)fprintf(file, "  \"terrain_cache_capacity\": 0,\n");
        (void)fprintf(file, "  \"terrain_cache_loaded_count\": 0,\n");
        (void)fprintf(file, "  \"terrain_cache_load_count\": 0,\n");
        (void)fprintf(file, "  \"terrain_cache_eviction_count\": 0,\n");
    }
    if (diagnostic_stats != 0 && diagnostic_stats->sample_count > 0u) {
        (void)fprintf(
            file,
            "  \"diagnostic_sample_count\": %lu,\n",
            (unsigned long)diagnostic_stats->sample_count);
        (void)fprintf(
            file,
            "  \"max_quat_norm_error\": %.12e,\n",
            diagnostic_stats->max_quat_norm_error);
        (void)fprintf(
            file,
            "  \"max_dcm_orthogonality_error\": %.12e,\n",
            diagnostic_stats->max_dcm_orthogonality_error);
        (void)fprintf(
            file,
            "  \"min_mass_kg\": %.9f,\n",
            diagnostic_stats->min_mass_kg);
        (void)fprintf(
            file,
            "  \"min_inertia_diag_kgm2\": %.9f,\n",
            diagnostic_stats->min_inertia_diag_kgm2);
        (void)fprintf(
            file,
            "  \"aero_model_flags_or\": %u,\n",
            diagnostic_stats->aero_model_flags_or);
        (void)fprintf(
            file,
            "  \"model_degradation_flags_or\": %u,\n",
            diagnostic_stats->model_degradation_flags_or);
        (void)fprintf(
            file,
            "  \"aero_extrapolated_sample_count\": %lu\n",
            (unsigned long)diagnostic_stats->aero_extrapolated_sample_count);
    } else {
        (void)fprintf(file, "  \"diagnostic_sample_count\": 0,\n");
        (void)fprintf(file, "  \"max_quat_norm_error\": 0.000000000000e+00,\n");
        (void)fprintf(file, "  \"max_dcm_orthogonality_error\": 0.000000000000e+00,\n");
        (void)fprintf(file, "  \"min_mass_kg\": 0.000000000,\n");
        (void)fprintf(file, "  \"min_inertia_diag_kgm2\": 0.000000000,\n");
        (void)fprintf(file, "  \"aero_model_flags_or\": 0,\n");
        (void)fprintf(file, "  \"model_degradation_flags_or\": 0,\n");
        (void)fprintf(file, "  \"aero_extrapolated_sample_count\": 0\n");
    }
    (void)fprintf(file, "}\n");
    (void)fclose(file);
}

/** @brief 写出不参与确定性物理回归的环境墙钟性能指标。 */
static void write_performance_report(
    const char *instance_dir,
    const EnvRuntimeConfig *runtime,
    double dt_s,
    const OperationalRunStats *stats)
{
    char path[1024];
    FILE *file;

    if (instance_dir == 0 || runtime == 0 || stats == 0) {
        return;
    }
    (void)snprintf(path, sizeof(path), "%s/performance.json", instance_dir);
    file = fopen(path, "wb");
    if (file == 0) {
        return;
    }
    (void)fprintf(file, "{\n");
    (void)fprintf(file, "  \"schema_version\": 1,\n");
    (void)fprintf(file, "  \"scope\": \"wall_clock_diagnostics_not_physics_input\",\n");
    (void)fprintf(file, "  \"run_mode\": \"%s\",\n", env_run_mode_to_string(runtime->run_mode));
    (void)fprintf(
        file,
        "  \"synchronization_mode\": \"%s\",\n",
        env_synchronization_mode_to_string(runtime->synchronization_mode));
    (void)fprintf(file, "  \"target_step_period_s\": %.9f,\n", dt_s);
    (void)fprintf(file, "  \"run_wall_time_s\": %.9f,\n", stats->run_wall_time_s);
    (void)fprintf(file, "  \"step_sample_count\": %lu,\n", (unsigned long)stats->step_compute.count);
    (void)fprintf(
        file,
        "  \"mean_step_compute_time_s\": %.12f,\n",
        stats->step_compute.count > 0u ?
            stats->step_compute.total_s / (double)stats->step_compute.count : 0.0);
    (void)fprintf(file, "  \"max_step_compute_time_s\": %.12f,\n", stats->step_compute.max_s);
    (void)fprintf(
        file,
        "  \"mean_sensor_send_time_s\": %.12f,\n",
        stats->sensor_send.count > 0u ?
            stats->sensor_send.total_s / (double)stats->sensor_send.count : 0.0);
    (void)fprintf(file, "  \"max_sensor_send_time_s\": %.12f,\n", stats->sensor_send.max_s);
    (void)fprintf(
        file,
        "  \"mean_command_receive_time_s\": %.12f,\n",
        stats->command_receive.count > 0u ?
            stats->command_receive.total_s / (double)stats->command_receive.count : 0.0);
    (void)fprintf(file, "  \"max_command_receive_time_s\": %.12f,\n", stats->command_receive.max_s);
    (void)fprintf(
        file,
        "  \"mean_control_roundtrip_time_s\": %.12f,\n",
        stats->control_roundtrip.count > 0u ?
            stats->control_roundtrip.total_s / (double)stats->control_roundtrip.count : 0.0);
    (void)fprintf(file, "  \"max_control_roundtrip_time_s\": %.12f,\n", stats->control_roundtrip.max_s);
    (void)fprintf(
        file,
        "  \"mean_log_write_time_s\": %.12f,\n",
        stats->log_write.count > 0u ?
            stats->log_write.total_s / (double)stats->log_write.count : 0.0);
    (void)fprintf(file, "  \"max_log_write_time_s\": %.12f,\n", stats->log_write.max_s);
    (void)fprintf(file, "  \"realtime_sleep_time_s\": %.9f,\n", stats->realtime_sleep_time_s);
    (void)fprintf(
        file,
        "  \"realtime_overrun_count\": %lu,\n",
        (unsigned long)stats->realtime_overrun_count);
    (void)fprintf(file, "  \"max_realtime_overrun_s\": %.12f,\n", stats->max_realtime_overrun_s);
    (void)fprintf(
        file,
        "  \"realtime_margin_s\": %.12f,\n",
        dt_s - stats->step_compute.max_s);
    (void)fprintf(
        file,
        "  \"command_timeout_count\": %lu,\n",
        (unsigned long)stats->command_timeout_count);
    (void)fprintf(
        file,
        "  \"command_hold_count\": %lu,\n",
        (unsigned long)stats->command_hold_count);
    (void)fprintf(
        file,
        "  \"protocol_minor_mismatch_count\": %lu\n",
        (unsigned long)stats->protocol_minor_mismatch_count);
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

/** @brief 写入数值诊断 CSV 表头。 */
static void write_diagnostics_header(FILE *file)
{
    (void)fprintf(
        file,
        "time_s,quat_norm_error,dcm_orthogonality_error,mass_kg,"
        "propellant_mass_kg,center_of_mass_b_x_m,center_of_mass_b_y_m,center_of_mass_b_z_m,"
        "inertia_xx_kgm2,inertia_xy_kgm2,inertia_xz_kgm2,"
        "inertia_yy_kgm2,inertia_yz_kgm2,inertia_zz_kgm2,"
        "wind_x_ecef_mps,wind_y_ecef_mps,wind_z_ecef_mps,"
        "inertia_min_diag_kgm2,aero_model_flags,"
        "model_degradation_flags,aero_uncertainty_scale,integrator_type,dt_s,"
        "force_norm_n,moment_norm_nm\n");
}

/** @brief 计算姿态 DCM 正交性 Frobenius 误差。 */
static double dcm_orthogonality_error(Quat attitude)
{
    Matrix3 dcm;
    Matrix3 product;
    double sum = 0.0;
    size_t row;
    size_t column;

    if (quat_to_dcm(attitude, &dcm) != SIM_OK) {
        return HUGE_VAL;
    }
    product = matrix3_multiply(dcm, matrix3_transpose(dcm));
    for (row = 0u; row < 3u; ++row) {
        for (column = 0u; column < 3u; ++column) {
            const double expected = row == column ? 1.0 : 0.0;
            const double error = product.m[row][column] - expected;

            sum += error * error;
        }
    }
    return sqrt(sum);
}

/** @brief 计算当前状态的诊断标量。 */
static void compute_diagnostics(
    const EnvTruthState *state,
    double *quat_norm_error,
    double *dcm_error,
    double *inertia_min)
{
    const Quat q = state->missile_plant.q_bi;
    const double q_norm = sqrt((q.w * q.w) + (q.x * q.x) + (q.y * q.y) + (q.z * q.z));
    double min_diag = state->missile_plant.inertia_b.m[0][0];

    if (state->missile_plant.inertia_b.m[1][1] < min_diag) {
        min_diag = state->missile_plant.inertia_b.m[1][1];
    }
    if (state->missile_plant.inertia_b.m[2][2] < min_diag) {
        min_diag = state->missile_plant.inertia_b.m[2][2];
    }
    if (quat_norm_error != 0) {
        *quat_norm_error = fabs(q_norm - 1.0);
    }
    if (dcm_error != 0) {
        *dcm_error = dcm_orthogonality_error(q);
    }
    if (inertia_min != 0) {
        *inertia_min = min_diag;
    }
}

/** @brief 将当前诊断标量分类为模型降级 flags。 */
static uint32_t classify_model_degradation(
    const EnvTruthState *state,
    const TerrainModel *terrain,
    double quat_error,
    double dcm_error,
    double inertia_min)
{
    uint32_t flags = 0u;

    if (state == 0) {
        return ENV_MODEL_DEGRADATION_MASS_INVALID |
            ENV_MODEL_DEGRADATION_INERTIA_INVALID;
    }
    if (state->aero_model_flags != 0u) {
        flags |= ENV_MODEL_DEGRADATION_AERO_FLAGS;
    }
    if (terrain != 0 && terrain->warning_flags != 0u) {
        flags |= ENV_MODEL_DEGRADATION_TERRAIN_WARNING;
    }
    if (!isfinite(state->missile_mass.mass_kg) || state->missile_mass.mass_kg <= 0.0) {
        flags |= ENV_MODEL_DEGRADATION_MASS_INVALID;
    }
    if (!isfinite(inertia_min) || inertia_min <= 0.0) {
        flags |= ENV_MODEL_DEGRADATION_INERTIA_INVALID;
    }
    if (!isfinite(quat_error) || quat_error > 1.0e-6) {
        flags |= ENV_MODEL_DEGRADATION_ATTITUDE_ERROR;
    }
    if (!isfinite(dcm_error) || dcm_error > 1.0e-6) {
        flags |= ENV_MODEL_DEGRADATION_DCM_ERROR;
    }
    return flags;
}

/** @brief 将当前状态累加进本次运行的数值诊断统计。 */
static void update_diagnostic_stats(
    DiagnosticRunStats *stats,
    const EnvTruthState *state,
    const TerrainModel *terrain)
{
    double quat_error;
    double dcm_error;
    double inertia_min;
    uint32_t degradation_flags;

    if (stats == 0 || state == 0) {
        return;
    }
    compute_diagnostics(state, &quat_error, &dcm_error, &inertia_min);
    degradation_flags = classify_model_degradation(
        state,
        terrain,
        quat_error,
        dcm_error,
        inertia_min);
    if (stats->sample_count == 0u || quat_error > stats->max_quat_norm_error) {
        stats->max_quat_norm_error = quat_error;
    }
    if (stats->sample_count == 0u || dcm_error > stats->max_dcm_orthogonality_error) {
        stats->max_dcm_orthogonality_error = dcm_error;
    }
    if (stats->sample_count == 0u || state->missile_mass.mass_kg < stats->min_mass_kg) {
        stats->min_mass_kg = state->missile_mass.mass_kg;
    }
    if (stats->sample_count == 0u || inertia_min < stats->min_inertia_diag_kgm2) {
        stats->min_inertia_diag_kgm2 = inertia_min;
    }
    stats->aero_model_flags_or |= state->aero_model_flags;
    if ((state->aero_model_flags & AERO_DB_FLAG_EXTRAPOLATED) != 0u) {
        ++stats->aero_extrapolated_sample_count;
    }
    stats->model_degradation_flags_or |= degradation_flags;
    ++stats->sample_count;
}

/** @brief 写入一行数值诊断。 */
static void write_diagnostics_row(
    FILE *file,
    const EnvTruthState *state,
    const EnvScenarioConfig *scenario,
    const TerrainModel *terrain)
{
    double quat_error;
    double dcm_error;
    double inertia_min;
    uint32_t degradation_flags;

    compute_diagnostics(state, &quat_error, &dcm_error, &inertia_min);
    degradation_flags = classify_model_degradation(
        state,
        terrain,
        quat_error,
        dcm_error,
        inertia_min);
    (void)fprintf(
        file,
        "%.6f,%.12e,%.12e,%.9f,%.9f,"
        "%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,"
        "%.9f,%u,%u,%.9f,%d,%.9f,%.9f,%.9f\n",
        state->time,
        quat_error,
        dcm_error,
        state->missile_mass.mass_kg,
        state->missile_mass.propellant_mass_kg,
        state->missile_mass.center_of_mass_b_m.x,
        state->missile_mass.center_of_mass_b_m.y,
        state->missile_mass.center_of_mass_b_m.z,
        state->missile_plant.inertia_b.m[0][0],
        state->missile_plant.inertia_b.m[0][1],
        state->missile_plant.inertia_b.m[0][2],
        state->missile_plant.inertia_b.m[1][1],
        state->missile_plant.inertia_b.m[1][2],
        state->missile_plant.inertia_b.m[2][2],
        state->wind_velocity_ecef_mps.x,
        state->wind_velocity_ecef_mps.y,
        state->wind_velocity_ecef_mps.z,
        inertia_min,
        state->aero_model_flags,
        degradation_flags,
        0.0,
        (int)scenario->integrator,
        scenario->dt,
        vec3_norm(state->missile_plant.force_b),
        vec3_norm(state->missile_plant.moment_b));
}

/** @brief 释放已加载的地形瓦片数组。 */
static void unload_terrain_tiles(TerrainTile *tiles, size_t tile_count)
{
    size_t index;

    if (tiles == 0) {
        return;
    }
    for (index = 0u; index < tile_count; ++index) {
        map_tile_unload(&tiles[index]);
    }
}

/** @brief 释放预加载瓦片和懒加载缓存中的瓦片。 */
static void unload_terrain_resources(
    TerrainTile *tiles,
    size_t tile_count,
    TerrainTileCache *cache)
{
    unload_terrain_tiles(tiles, tile_count);
    terrain_tile_cache_unload(cache);
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
    CommunicationDelayLine communication_delay;
    CommunicationReorderState communication_reorder;
    ActuatorCommandDelayLine actuator_command_delays[3];
    SensorFaultDelayLine sensor_fault_delay_lines[ENV_SENSOR_FAULT_CHANNEL_COUNT];
    SensorFaultStuckState sensor_fault_stuck_states[ENV_SENSOR_FAULT_CHANNEL_COUNT];
    FaultRunStats fault_stats;
    OperationalRunStats operational_stats;
    DiagnosticRunStats diagnostic_stats;
    RealtimePacer realtime_pacer;
    ConfigWarningCollector config_warnings;
    EarthModel earth;
    TerrainModel terrain;
    TerrainTile terrain_tiles[ENV_MAX_TERRAIN_TILES];
    TerrainTileCache terrain_cache;
    size_t terrain_tile_count = 0u;
    AeroDatabase aero_database;
    AeroDatabaseV2 aero_database_v2;
    AeroSurrogateModel aero_surrogate;
    int aero_database_loaded = 0;
    int aero_database_v2_loaded = 0;
    Logger logger;
    SimStatus status;
    int sock = -1;
    unsigned int env_port;
    unsigned int fc_port;
    struct sockaddr_in fc_addr;
    char instance_dir[512];
    char path[1024];
    char start_time_wall_clock[32];
    FILE *sensor_log = 0;
    FILE *command_log = 0;
    FILE *trajectory_log = 0;
    FILE *diagnostics_log = 0;
    FILE *event_log = 0;
    uint32_t seq = 0u;
    int hit = 0;
    const char *exit_reason = "max_time_reached";
    uint64_t instance_random_seed;
    double run_wall_start_s = 0.0;
    int realtime_overrun_event_written = 0;
    ControlCommand held_command;
    int has_held_command = 0;
    int initial_command_hold_event_written = 0;
    int protocol_minor_warning_written = 0;

    if (ctx == 0 || ctx->scenario_path == 0 ||
        ctx->runtime_path == 0 || ctx->faults_path == 0) {
        return SIM_ERR_INVALID_ARG;
    }

    memset(&scenario_tree, 0, sizeof(scenario_tree));
    memset(&runtime_tree, 0, sizeof(runtime_tree));
    memset(&fault_stats, 0, sizeof(fault_stats));
    memset(&operational_stats, 0, sizeof(operational_stats));
    memset(&diagnostic_stats, 0, sizeof(diagnostic_stats));
    memset(&realtime_pacer, 0, sizeof(realtime_pacer));
    memset(&config_warnings, 0, sizeof(config_warnings));
    memset(&held_command, 0, sizeof(held_command));
    memset(terrain_tiles, 0, sizeof(terrain_tiles));
    memset(&terrain_cache, 0, sizeof(terrain_cache));
    memset(&aero_database, 0, sizeof(aero_database));
    memset(&aero_database_v2, 0, sizeof(aero_database_v2));
    memset(&aero_surrogate, 0, sizeof(aero_surrogate));
    format_wall_clock_utc(start_time_wall_clock, sizeof(start_time_wall_clock));
    communication_delay_reset(&communication_delay);
    communication_reorder_reset(&communication_reorder);
    actuator_command_delay_reset(&actuator_command_delays[0]);
    actuator_command_delay_reset(&actuator_command_delays[1]);
    actuator_command_delay_reset(&actuator_command_delays[2]);
    (void)memset(sensor_fault_delay_lines, 0, sizeof(sensor_fault_delay_lines));
    (void)memset(sensor_fault_stuck_states, 0, sizeof(sensor_fault_stuck_states));
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
        status = audit_environment_config_fields(
            &scenario_tree,
            &runtime_tree,
            &faults_tree,
            &config_warnings);
        if (status != SIM_OK) {
            (void)fprintf(
                stderr,
                "environment_sim: config field audit failed: %s\n",
                sim_status_to_string(status));
            config_free(&faults_tree);
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
    instance_random_seed = ctx->has_random_seed_override != 0 ?
        ctx->random_seed_override :
        runtime.base_random_seed + ctx->instance_id;
    fault_injection_set_seed(
        &faults,
        instance_random_seed ^ UINT64_C(0x4641554C54));
    status = init_sensor_state(
        &sensors,
        &scenario,
        instance_random_seed);
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
    status = write_config_snapshots(instance_dir, ctx);
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
        start_time_wall_clock,
        instance_random_seed,
        env_port,
        fc_port,
        &config_warnings);
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
        {
            size_t warning_index;

            for (warning_index = 0u;
                 warning_index < config_warnings.stored_count;
                 ++warning_index) {
                write_event(
                    event_log,
                    0.0,
                    "CONFIG_UNKNOWN_FIELD_WARNING",
                    config_warnings.paths[warning_index]);
            }
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
    status = terrain_tile_cache_init_from_index(
        &terrain_cache,
        scenario.terrain_tile_index_entries,
        scenario.terrain_tile_path_count,
        scenario.terrain_cache_tile_count);
    if (status == SIM_OK) {
        status = terrain_model_init_with_cache(
            &terrain,
            &terrain_cache,
            scenario.terrain_missing_policy,
            scenario.terrain_flat_fill_height_m);
    }
    if (status != SIM_OK) {
        unload_terrain_resources(terrain_tiles, terrain_tile_count, &terrain_cache);
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
            unload_terrain_resources(terrain_tiles, terrain_tile_count, &terrain_cache);
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
    status = wind_model_init(
        &state.wind_model,
        &scenario.wind_model,
        instance_random_seed ^ UINT64_C(0x57494E44));
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
        unload_terrain_resources(terrain_tiles, terrain_tile_count, &terrain_cache);
        return status;
    }
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
        unload_terrain_resources(terrain_tiles, terrain_tile_count, &terrain_cache);
        return status;
    }
    state.initial_mass_kg = state.missile_mass.mass_kg;
    state.missile_plant.mass = state.missile_mass.mass_kg;
    state.missile_plant.inertia_b = matrix3_zero();
    state.missile_plant.inertia_b.m[0][0] = scenario.inertia_diag[0];
    state.missile_plant.inertia_b.m[1][1] = scenario.inertia_diag[1];
    state.missile_plant.inertia_b.m[2][2] = scenario.inertia_diag[2];
    if (scenario.mass_properties_enabled != 0) {
        Vec3 center_of_mass_b_m;

        status = mass_model_configure_properties(
            &state.missile_mass,
            scenario.dry_center_of_mass_b_m,
            scenario.propellant_center_of_mass_full_b_m,
            scenario.propellant_center_of_mass_empty_b_m,
            scenario.dry_inertia_centroid_b_kgm2,
            scenario.propellant_inertia_full_centroid_b_kgm2);
        if (status == SIM_OK) {
            status = mass_model_get_properties(
                &state.missile_mass,
                &center_of_mass_b_m,
                &state.missile_plant.inertia_b);
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
            unload_terrain_resources(terrain_tiles, terrain_tile_count, &terrain_cache);
            return status;
        }
    }
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
    if (scenario.aero_table_path_enabled != 0) {
        status = aero_database_load_file(scenario.aero_table_path, &aero_database);
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
            unload_terrain_resources(terrain_tiles, terrain_tile_count, &terrain_cache);
            (void)close(sock);
            config_free(&scenario_tree);
            config_free(&runtime_tree);
            return status;
        }
        if (scenario.aero_table_policy_override_enabled != 0) {
            aero_database.extrapolation_policy = scenario.aero_table_policy_override;
        }
        scenario.force_model.aerodynamics.database = &aero_database;
        aero_database_loaded = 1;
    }
    if (scenario.aero_table_v2_path_enabled != 0) {
        size_t axis;

        status = aero_database_v2_load_file(
            scenario.aero_table_v2_path,
            &aero_database_v2);
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
            if (aero_database_loaded != 0) {
                aero_database_unload(&aero_database);
            }
            unload_terrain_resources(terrain_tiles, terrain_tile_count, &terrain_cache);
            (void)close(sock);
            config_free(&scenario_tree);
            config_free(&runtime_tree);
            return status;
        }
        if (scenario.aero_table_policy_override_enabled != 0) {
            aero_database_v2.extrapolation_policy = scenario.aero_table_policy_override;
        }
        for (axis = 0u; axis < AERO_DATABASE_V2_AXIS_COUNT; ++axis) {
            scenario.aero_table_v2_dimensions[axis] = aero_database_v2.dimensions[axis];
        }
        scenario.force_model.aerodynamics.database_v2 = &aero_database_v2;
        aero_database_v2_loaded = 1;
    }
    if (scenario.aero_surrogate_model_path_enabled != 0) {
        status = aero_surrogate_load_file(scenario.aero_surrogate_model_path, &aero_surrogate);
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
            if (aero_database_loaded != 0) {
                aero_database_unload(&aero_database);
            }
            if (aero_database_v2_loaded != 0) {
                aero_database_v2_unload(&aero_database_v2);
            }
            unload_terrain_resources(terrain_tiles, terrain_tile_count, &terrain_cache);
            (void)close(sock);
            config_free(&scenario_tree);
            config_free(&runtime_tree);
            return status;
        }
        scenario.force_model.aerodynamics.surrogate = &aero_surrogate;
    }
    status = write_run_manifest(
        instance_dir,
        ctx,
        &scenario,
        &runtime,
        start_time_wall_clock,
        instance_random_seed,
        env_port,
        fc_port,
        &config_warnings);
    if (status != SIM_OK) {
        if (aero_database_loaded != 0) {
            aero_database_unload(&aero_database);
        }
        if (aero_database_v2_loaded != 0) {
            aero_database_v2_unload(&aero_database_v2);
        }
        unload_terrain_resources(terrain_tiles, terrain_tile_count, &terrain_cache);
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
    status = update_geodetic_state(&state, &earth, &terrain);
    if (status != SIM_OK) {
        exit_reason = "terrain_initialization_failed";
        write_event(event_log, state.time, "TERRAIN_ERROR", sim_status_to_string(status));
    }
    state.min_range = vec3_norm(vec3_sub(state.target_pos, state.missile_pos));
    if (trajectory_log != 0) {
        (void)snprintf(path, sizeof(path), "%s/trajectory_diagnostics.csv", instance_dir);
        diagnostics_log = fopen(path, "wb");
        if (diagnostics_log == 0) {
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
            if (aero_database_loaded != 0) {
                aero_database_unload(&aero_database);
            }
            if (aero_database_v2_loaded != 0) {
                aero_database_v2_unload(&aero_database_v2);
            }
            unload_terrain_resources(terrain_tiles, terrain_tile_count, &terrain_cache);
            (void)close(sock);
            config_free(&scenario_tree);
            config_free(&runtime_tree);
            return SIM_ERR_IO;
        }
        write_trajectory_header(trajectory_log);
        write_trajectory_row(trajectory_log, &state);
        write_diagnostics_header(diagnostics_log);
        update_diagnostic_stats(&diagnostic_stats, &state, &terrain);
        write_diagnostics_row(diagnostics_log, &state, &scenario, &terrain);
    }

    status = realtime_pacer_init(
        &realtime_pacer,
        scenario.dt,
        runtime.run_mode == ENV_RUN_MODE_SIL_REALTIME);
    if (status == SIM_OK) {
        status = monotonic_time_now(&run_wall_start_s);
    }
    if (status != SIM_OK) {
        exit_reason = "wall_clock_initialization_failed";
        write_event(event_log, state.time, "REALTIME_ERROR", sim_status_to_string(status));
    }
    (void)logger_info(&logger, "environment_sim UDP loop started");
    write_event(
        event_log,
        state.time,
        "SIMULATION_START",
        env_run_mode_to_string(runtime.run_mode));
    (void)printf("instance_id=%u env_port=%u fc_port=%u\n", ctx->instance_id, env_port, fc_port);

    while (status == SIM_OK && state.time <= scenario.max_time) {
        SensorFrame sensor;
        ControlCommand command;
        FaultStepEffects fault_effects;
        FaultTransition fault_transitions[ENV_MAX_FAULT_TRANSITIONS];
        size_t fault_transition_count = 0u;
        double range = vec3_norm(vec3_sub(state.target_pos, state.missile_pos));
        double step_start_s = 0.0;
        double control_roundtrip_start_s = 0.0;
        int surface_collision = 0;

        status = monotonic_time_now(&step_start_s);
        if (status != SIM_OK) {
            exit_reason = "wall_clock_read_failed";
            write_event(event_log, state.time, "REALTIME_ERROR", sim_status_to_string(status));
            break;
        }

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
        status = apply_sensor_fault_stuck(
            sensor_fault_stuck_states,
            &fault_effects,
            &sensor);
        if (status == SIM_OK) {
            status = apply_sensor_fault_delay(
                sensor_fault_delay_lines,
                &fault_effects,
                &sensor);
        }
        if (status != SIM_OK) {
            exit_reason = "sensor_fault_state_failed";
            write_event(event_log, state.time, "FAULT_ERROR", sim_status_to_string(status));
            break;
        }
        status = apply_communication_reorder(&communication_reorder, &fault_effects, &sensor);
        if (status != SIM_OK) {
            exit_reason = "communication_reorder_failed";
            write_event(event_log, state.time, "FAULT_ERROR", sim_status_to_string(status));
            break;
        }
        status = apply_communication_delay(&communication_delay, &fault_effects, &sensor);
        if (status != SIM_OK) {
            exit_reason = "communication_delay_failed";
            write_event(event_log, state.time, "FAULT_ERROR", sim_status_to_string(status));
            break;
        }
        {
            const uint32_t expected_sensor_flags =
                SIM_SENSOR_VALID_SEEKER |
                SIM_SENSOR_VALID_IMU_GYRO |
                SIM_SENSOR_VALID_ACCEL |
                SIM_SENSOR_VALID_SPEED |
                SIM_SENSOR_VALID_GEODETIC;

            if ((sensor.sensor_valid_flags & expected_sensor_flags) != expected_sensor_flags) {
                ++operational_stats.sensor_dropout_count;
            }
        }

        status = monotonic_time_now(&control_roundtrip_start_s);
        if (status == SIM_OK) {
            status = send_sensor_frame(
                sock,
                &fc_addr,
                ctx->instance_id,
                &sensor,
                &fault_effects);
        }
        if (status == SIM_OK) {
            status = timing_finish(control_roundtrip_start_s, &operational_stats.sensor_send);
        }
        if (status != SIM_OK) {
            exit_reason = "send_failed";
            break;
        }
        if (seq == 0u) {
            (void)printf("sent first SensorFrame range=%.3f closing=%.3f\n",
                sensor.target_range_meas,
                sensor.target_closing_velocity_meas);
        }
        {
            double log_start_s = 0.0;

            status = monotonic_time_now(&log_start_s);
            if (status == SIM_OK) {
                status = write_sensor_log(sensor_log, ctx->instance_id, &sensor);
            }
            if (status == SIM_OK) {
                status = timing_finish(log_start_s, &operational_stats.log_write);
            }
        }
        if (status != SIM_OK) {
            exit_reason = "sensor_log_failed";
            write_event(event_log, state.time, "IO_ERROR", exit_reason);
            break;
        }

        {
            double receive_start_s = 0.0;
            SimStatus timing_status;

            status = monotonic_time_now(&receive_start_s);
            if (status == SIM_OK && runtime.synchronization_mode == ENV_SYNC_LOCKSTEP) {
                status = receive_control_command(
                    sock,
                    ctx->instance_id,
                    &fc_addr,
                    &operational_stats.protocol_minor_mismatch_count,
                    &command);
                if (status == SIM_OK) {
                    held_command = command;
                    has_held_command = 1;
                }
            } else if (status == SIM_OK) {
                int received_new_command = 0;

                status = receive_latest_control_command(
                    sock,
                    ctx->instance_id,
                    seq,
                    &fc_addr,
                    &operational_stats.protocol_minor_mismatch_count,
                    &held_command,
                    &received_new_command);
                if (status == SIM_OK && received_new_command != 0) {
                    has_held_command = 1;
                } else if (status == SIM_OK) {
                    ++operational_stats.command_hold_count;
                }
                command = held_command;
            }
            if (status == SIM_OK || status == SIM_ERR_TIMEOUT) {
                timing_status = timing_finish(
                    receive_start_s,
                    &operational_stats.command_receive);
                if (timing_status != SIM_OK && status == SIM_OK) {
                    status = timing_status;
                }
                if (runtime.synchronization_mode == ENV_SYNC_LOCKSTEP) {
                    timing_status = timing_finish(
                        control_roundtrip_start_s,
                        &operational_stats.control_roundtrip);
                    if (timing_status != SIM_OK && status == SIM_OK) {
                        status = timing_status;
                    }
                }
            }
        }
        if (status != SIM_OK) {
            if (status == SIM_ERR_TIMEOUT) {
                ++operational_stats.command_timeout_count;
                exit_reason = "control_timeout";
            } else {
                exit_reason = "control_receive_failed";
            }
            write_event(event_log, state.time, "CONTROL_RECEIVE_FAILED", sim_status_to_string(status));
            break;
        }
        if (operational_stats.protocol_minor_mismatch_count > 0u &&
            protocol_minor_warning_written == 0) {
            write_event(
                event_log,
                state.time,
                "PROTOCOL_MINOR_VERSION_WARNING",
                "compatible_minor_version_received");
            protocol_minor_warning_written = 1;
        }
        if (runtime.synchronization_mode == ENV_SYNC_FREE_RUNNING &&
            has_held_command == 0 && initial_command_hold_event_written == 0) {
            write_event(event_log, state.time, "COMMAND_HOLD", "no_command_received_yet");
            initial_command_hold_event_written = 1;
        }
        {
            const double command_norm = vec3_norm(command.accel_cmd_ecef);

            if (command_norm > operational_stats.max_command_norm) {
                operational_stats.max_command_norm = command_norm;
            }
        }
        {
            double log_start_s = 0.0;

            status = monotonic_time_now(&log_start_s);
            if (status == SIM_OK) {
                status = write_command_log(command_log, ctx->instance_id, &command);
            }
            if (status == SIM_OK) {
                status = timing_finish(log_start_s, &operational_stats.log_write);
            }
        }
        if (status != SIM_OK) {
            exit_reason = "command_log_failed";
            write_event(event_log, state.time, "IO_ERROR", exit_reason);
            break;
        }
        {
            const Vec3 missile_start = state.missile_pos;
            const Vec3 target_start = state.target_pos;
            const double step_start_time = state.time;
            HitDetectResult hit_result;

            status = update_truth(
                &state,
                &command,
                &scenario,
                &scenario.force_model,
                &fault_effects,
                actuator_command_delays);
            if (status == SIM_OK) {
                status = hit_detect_segment(
                    missile_start,
                    state.missile_pos,
                    target_start,
                    state.target_pos,
                    scenario.hit_radius_m,
                    &hit_result);
            }
            if (status != SIM_OK) {
                exit_reason = "plant_update_failed";
                write_event(event_log, state.time, "PLANT_ERROR", sim_status_to_string(status));
                break;
            }
            if (hit_result.minimum_range_m < state.min_range) {
                state.min_range = hit_result.minimum_range_m;
                state.time_of_closest = step_start_time +
                    (hit_result.closest_fraction * scenario.dt);
            }
            if (hit_result.hit != 0) {
                hit = 1;
                exit_reason = "hit";
                write_event(
                    event_log,
                    state.time_of_closest,
                    "HIT",
                    "continuous_relative_segment");
                break;
            }
        }
        {
            const double actual_accel = vec3_norm(state.missile_actual_accel);

            if (actual_accel > operational_stats.max_actual_accel) {
                operational_stats.max_actual_accel = actual_accel;
            }
        }
        status = update_geodetic_state(&state, &earth, &terrain);
        if (status != SIM_OK) {
            exit_reason = "geodetic_update_failed";
            write_event(event_log, state.time, "NUMERIC_ERROR", sim_status_to_string(status));
            break;
        }
        if (trajectory_log != 0) {
            double log_start_s = 0.0;

            status = monotonic_time_now(&log_start_s);
            if (status != SIM_OK) {
                exit_reason = "wall_clock_read_failed";
                break;
            }
            write_trajectory_row(trajectory_log, &state);
            update_diagnostic_stats(&diagnostic_stats, &state, &terrain);
            write_diagnostics_row(diagnostics_log, &state, &scenario, &terrain);
            status = timing_finish(log_start_s, &operational_stats.log_write);
            if (status != SIM_OK) {
                exit_reason = "wall_clock_read_failed";
                break;
            }
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
            if (diagnostics_log != 0) {
                (void)fflush(diagnostics_log);
            }
            if (event_log != 0) {
                (void)fflush(event_log);
            }
        }
        status = timing_finish(step_start_s, &operational_stats.step_compute);
        if (status == SIM_OK) {
            double sleep_s = 0.0;
            double overrun_s = 0.0;

            status = realtime_pacer_wait_next(&realtime_pacer, &sleep_s, &overrun_s);
            operational_stats.realtime_sleep_time_s += sleep_s;
            if (overrun_s > 0.0) {
                char detail[96];

                ++operational_stats.realtime_overrun_count;
                if (overrun_s > operational_stats.max_realtime_overrun_s) {
                    operational_stats.max_realtime_overrun_s = overrun_s;
                }
                if (realtime_overrun_event_written == 0) {
                    (void)snprintf(detail, sizeof(detail), "overrun_s=%.9f", overrun_s);
                    write_event(event_log, state.time, "REALTIME_OVERRUN", detail);
                    realtime_overrun_event_written = 1;
                }
            }
        }
        if (status != SIM_OK) {
            exit_reason = "wall_clock_pacing_failed";
            write_event(event_log, state.time, "REALTIME_ERROR", sim_status_to_string(status));
            break;
        }
    }

    {
        SimStatus shutdown_status = send_sim_stop(
            sock,
            &fc_addr,
            ctx->instance_id,
            seq,
            state.time);

        if (shutdown_status == SIM_OK) {
            write_event(event_log, state.time, "SIM_CONTROL_STOP_SENT", exit_reason);
        } else {
            write_event(
                event_log,
                state.time,
                "SIM_CONTROL_STOP_FAILED",
                sim_status_to_string(shutdown_status));
            if (status == SIM_OK) {
                status = shutdown_status;
                exit_reason = "sim_control_stop_failed";
            }
        }
    }

    {
        double run_wall_end_s = 0.0;

        if (monotonic_time_now(&run_wall_end_s) == SIM_OK && run_wall_end_s >= run_wall_start_s) {
            operational_stats.run_wall_time_s = run_wall_end_s - run_wall_start_s;
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
    if (diagnostics_log != 0) {
        (void)fclose(diagnostics_log);
    }
    write_event(event_log, state.time, "SIMULATION_STOP", exit_reason);
    if (event_log != 0) {
        (void)fclose(event_log);
    }
    write_summary(
        instance_dir,
        hit,
        &state,
        seq,
        exit_reason,
        &operational_stats,
        &fault_stats,
        &terrain_cache,
        &diagnostic_stats);
    write_performance_report(instance_dir, &runtime, scenario.dt, &operational_stats);

    if (sock >= 0) {
        (void)close(sock);
    }
    unload_terrain_resources(terrain_tiles, terrain_tile_count, &terrain_cache);
    if (aero_database_loaded != 0) {
        aero_database_unload(&aero_database);
    }
    if (aero_database_v2_loaded != 0) {
        aero_database_v2_unload(&aero_database_v2);
    }
    config_free(&scenario_tree);
    config_free(&runtime_tree);
    (void)printf("summary: hit=%d min_range=%.3f exit=%s steps=%u\n", hit, state.min_range, exit_reason, seq);
    return status == SIM_ERR_TIMEOUT && hit != 0 ? SIM_OK : status;
}
