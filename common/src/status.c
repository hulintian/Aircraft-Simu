/** @file status.c
 *  @brief 仿真状态码转字符串实现。
 */
#include "common/status.h"

/** @brief 将状态码映射为稳定的诊断字符串。 */
const char *sim_status_to_string(SimStatus status)
{
    switch (status) {
    case SIM_OK:
        return "SIM_OK";
    case SIM_ERR_INVALID_ARG:
        return "SIM_ERR_INVALID_ARG";
    case SIM_ERR_OUT_OF_RANGE:
        return "SIM_ERR_OUT_OF_RANGE";
    case SIM_ERR_BAD_PACKET:
        return "SIM_ERR_BAD_PACKET";
    case SIM_ERR_TIMEOUT:
        return "SIM_ERR_TIMEOUT";
    case SIM_ERR_CONFIG:
        return "SIM_ERR_CONFIG";
    case SIM_ERR_NUMERIC:
        return "SIM_ERR_NUMERIC";
    case SIM_ERR_IO:
        return "SIM_ERR_IO";
    case SIM_ERR_INTERNAL:
        return "SIM_ERR_INTERNAL";
    default:
        return "SIM_ERR_UNKNOWN";
    }
}
