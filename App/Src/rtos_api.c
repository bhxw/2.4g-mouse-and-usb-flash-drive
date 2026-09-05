/**
 * @file rtos_api.c
 * @brief RTOS 抽象层实现：当前映射到 FreeRTOS（M5 自研内核时整体替换本文件）。
 */
#include "rtos_api.h"

#include "FreeRTOS.h"
#include "task.h"

int rtos_task_create(const char *name, rtos_task_entry_t entry, void *param,
                     uint16_t stack_words, uint8_t priority,
                     rtos_task_handle_t *handle)
{
    BaseType_t rc = xTaskCreate((TaskFunction_t)entry, name, stack_words,
                                param, priority, (TaskHandle_t *)handle);
    return (rc == pdPASS) ? 0 : -1;
}

void rtos_scheduler_start(void)
{
    vTaskStartScheduler();
    /* 调度器启动失败才可能返回（例如堆不足），调用方需兜底 */
}

void rtos_delay(uint32_t ms)
{
    vTaskDelay(pdMS_TO_TICKS(ms));
}
