/** @file control_quality.c
 *  @brief 标准阶跃响应控制品质指标实现。
 */
#include "fc/control_quality.h"

#include <math.h>
#include <string.h>

SimStatus control_quality_analyze_step(
    const ControlQualitySample *samples,
    size_t sample_count,
    double step_time_s,
    double target,
    double settling_band_fraction,
    ControlQualityMetrics *out)
{
    size_t start = sample_count;
    size_t index;
    size_t saturated_count = 0u;
    size_t last_outside = sample_count;
    double ten_percent_time = NAN;
    double ninety_percent_time = NAN;
    double maximum_normalized = -INFINITY;
    double maximum_rate = 0.0;
    const double tolerance = fabs(target) * settling_band_fraction;

    if (samples == 0 || out == 0 || sample_count < 2u ||
        !isfinite(step_time_s) || !isfinite(target) || target == 0.0 ||
        !isfinite(settling_band_fraction) || settling_band_fraction <= 0.0) {
        return SIM_ERR_INVALID_ARG;
    }
    (void)memset(out, 0, sizeof(*out));
    for (index = 0u; index < sample_count; ++index) {
        if (!isfinite(samples[index].time_s) || !isfinite(samples[index].response) ||
            (index > 0u && samples[index].time_s <= samples[index - 1u].time_s)) {
            return SIM_ERR_NUMERIC;
        }
        if (start == sample_count && samples[index].time_s >= step_time_s) {
            start = index;
        }
    }
    if (start == sample_count || sample_count - start < 2u) {
        return SIM_ERR_OUT_OF_RANGE;
    }
    for (index = start; index < sample_count; ++index) {
        const double normalized = samples[index].response / target;

        if (normalized >= 0.1 && !isfinite(ten_percent_time)) {
            ten_percent_time = samples[index].time_s;
        }
        if (normalized >= 0.9 && !isfinite(ninety_percent_time)) {
            ninety_percent_time = samples[index].time_s;
        }
        if (normalized > maximum_normalized) {
            maximum_normalized = normalized;
        }
        if (fabs(samples[index].response - target) > tolerance) {
            last_outside = index;
        }
        if (samples[index].saturated != 0) {
            ++saturated_count;
        }
        if (index > start) {
            const double dt = samples[index].time_s - samples[index - 1u].time_s;
            const double rate = fabs(samples[index].response - samples[index - 1u].response) / dt;

            if (rate > maximum_rate) {
                maximum_rate = rate;
            }
        }
    }
    if (!isfinite(ten_percent_time) || !isfinite(ninety_percent_time)) {
        return SIM_ERR_OUT_OF_RANGE;
    }
    out->rise_time_s = ninety_percent_time - ten_percent_time;
    if (last_outside == sample_count) {
        out->settling_time_s = samples[start].time_s - step_time_s;
    } else if (last_outside + 1u < sample_count) {
        out->settling_time_s = samples[last_outside + 1u].time_s - step_time_s;
    } else {
        return SIM_ERR_OUT_OF_RANGE;
    }
    out->overshoot_percent = maximum_normalized > 1.0 ? (maximum_normalized - 1.0) * 100.0 : 0.0;
    out->steady_state_error = samples[sample_count - 1u].response - target;
    out->max_abs_rate = maximum_rate;
    out->evaluated_sample_count = sample_count - start;
    out->saturation_fraction = (double)saturated_count / (double)out->evaluated_sample_count;
    return SIM_OK;
}
