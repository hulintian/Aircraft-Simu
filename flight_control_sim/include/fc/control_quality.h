/** @file control_quality.h
 *  @brief 标准机动时序的控制品质指标计算。
 */
#ifndef FC_CONTROL_QUALITY_H
#define FC_CONTROL_QUALITY_H

#include "common/status.h"

#include <stddef.h>

/** @brief 一条标量控制响应采样。 */
typedef struct ControlQualitySample {
    /** @brief 仿真时间，单位 s，必须严格递增。 */
    double time_s;
    /** @brief 被评估的响应量。 */
    double response;
    /** @brief 非零表示该采样发生限幅或饱和。 */
    int saturated;
} ControlQualitySample;

/** @brief 阶跃响应控制品质指标。 */
typedef struct ControlQualityMetrics {
    /** @brief 10% 到 90% 上升时间，单位 s。 */
    double rise_time_s;
    /** @brief 进入并持续保持在容差带内的时间，单位 s。 */
    double settling_time_s;
    /** @brief 相对目标方向的百分比超调。 */
    double overshoot_percent;
    /** @brief 最后一帧响应与目标之差。 */
    double steady_state_error;
    /** @brief 相邻采样间最大绝对变化率。 */
    double max_abs_rate;
    /** @brief 阶跃后限幅/饱和采样所占比例。 */
    double saturation_fraction;
    /** @brief 阶跃后参与评估的采样数。 */
    size_t evaluated_sample_count;
} ControlQualityMetrics;

/**
 * @brief 计算一个标量阶跃工况的品质指标。
 *
 * `target` 必须非零。上升时间按目标方向归一化后计算；稳定时间要求从该
 * 时刻到记录结尾始终满足 `abs(response-target) <= settling_band_fraction * abs(target)`。
 */
SimStatus control_quality_analyze_step(
    const ControlQualitySample *samples,
    size_t sample_count,
    double step_time_s,
    double target,
    double settling_band_fraction,
    ControlQualityMetrics *out);

#endif
