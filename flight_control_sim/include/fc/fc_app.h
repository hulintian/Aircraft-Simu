/** @file fc_app.h
 *  @brief 飞控仿真程序入口。
 */
#ifndef FC_FC_APP_H
#define FC_FC_APP_H

#include "fc/fc_context.h"
#include "common/status.h"

/** @brief 运行指定实例的飞控主循环。 */
SimStatus fc_app_run(const FcContext *ctx);

#endif
