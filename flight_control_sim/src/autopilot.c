/** @file autopilot.c
 *  @brief 自动驾驶仪命令转换实现。
 */
#include "fc/autopilot.h"

#include <math.h>
#include <string.h>

SimStatus autopilot_init(Autopilot *autopilot)
{
    if (autopilot == 0) {
        return SIM_ERR_INVALID_ARG;
    }
    (void)memset(autopilot, 0, sizeof(*autopilot));
    return SIM_OK;
}

void autopilot_zero_command(AutopilotCommand *out)
{
    if (out != 0) {
        (void)memset(out, 0, sizeof(*out));
    }
}

SimStatus autopilot_update(
    Autopilot *autopilot,
    const GuidancePngOutput *guidance,
    AutopilotCommand *out)
{
    if (autopilot == 0 || guidance == 0 || out == 0) {
        return SIM_ERR_INVALID_ARG;
    }
    if (!vec3_isfinite(guidance->accel_cmd_ecef)) {
        autopilot->rejected_count += 1u;
        return SIM_ERR_NUMERIC;
    }
    autopilot_zero_command(out);
    out->accel_cmd_ecef = guidance->accel_cmd_ecef;
    autopilot->accepted_count += 1u;
    return SIM_OK;
}
