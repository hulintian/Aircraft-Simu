/** @file fc_interface.h
 *  @brief 飞控外部接口占位。
 */
#ifndef FC_FC_INTERFACE_H
#define FC_FC_INTERFACE_H

#include "common/status.h"

/** @brief 飞控接口对象。 */
typedef struct FcInterface {
    /** @brief 预留扩展字段。 */
    int reserved;
} FcInterface;

/** @brief 初始化飞控接口。 */
SimStatus fc_interface_init(FcInterface *iface);

#endif
