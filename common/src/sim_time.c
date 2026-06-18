/** @file sim_time.c
 *  @brief 仿真时间工具实现。
 */
#include "common/sim_time.h"

/** @brief 将 tick 映射为秒。 */
double sim_time_seconds(SimTime time)
{
    return (double)time.tick * time.dt;
}

/** @brief 推进一个离散时间步。 */
void sim_time_step(SimTime *time)
{
    if (time != 0) {
        time->tick += 1u;
    }
}
