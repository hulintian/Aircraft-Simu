/** @file logger.c
 *  @brief 最小日志实现。
 */
#include "common/logger.h"

/** @brief 将日志器绑定到标准输出。 */
SimStatus logger_open_stdout(Logger *logger)
{
    if (logger == 0) {
        return SIM_ERR_INVALID_ARG;
    }
    logger->stream = stdout;
    return SIM_OK;
}

/** @brief 输出信息级日志。 */
SimStatus logger_info(Logger *logger, const char *message)
{
    if (logger == 0 || logger->stream == 0 || message == 0) {
        return SIM_ERR_INVALID_ARG;
    }
    (void)fprintf(logger->stream, "%s\n", message);
    return SIM_OK;
}
