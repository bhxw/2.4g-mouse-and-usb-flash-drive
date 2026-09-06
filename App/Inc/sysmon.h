#ifndef __SYSMON_H
#define __SYSMON_H

#include <stdint.h>

/**
 * @file sysmon.h
 * @brief M4 系统监控：IWDG 看门狗、运行统计/栈水位周期输出、RF 链路计数。
 */
void sysmon_init_hw(void);   /* 启动 IWDG（在进 RTOS 前调用一次） */
void sysmon_start(void);     /* 启动监控任务 */
void sysmon_rx_ok(void);     /* rf 收到一包 */
void sysmon_rx_idle(void);   /* rf 轮询无包 */

#endif /* __SYSMON_H */
