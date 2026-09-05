#ifndef __RTOS_API_H
#define __RTOS_API_H

/**
 * @file rtos_api.h
 * @brief RTOS 抽象层（D2 决策）：业务代码只允许通过本接口使用内核。
 *        当前实现基于 FreeRTOS；M5 替换为自研微内核时仅需改 rtos_api.c。
 */

#include <stdint.h>

typedef void (*rtos_task_entry_t)(void *param);
typedef void *rtos_task_handle_t;

/** 创建一个任务（stack_words 单位为字/word，与 FreeRTOS 一致）。成功返回 0，失败 -1。 */
int rtos_task_create(const char *name, rtos_task_entry_t entry, void *param,
                     uint16_t stack_words, uint8_t priority,
                     rtos_task_handle_t *handle);

/** 启动调度器（通常不返回）。 */
void rtos_scheduler_start(void);

/** 阻塞延时（毫秒）。 */
void rtos_delay(uint32_t ms);

#endif /* __RTOS_API_H */
