/**
 * @file rtos_api.c
 * @brief RTOS 抽象层实现：当前映射到 FreeRTOS（M5 自研内核时整体替换本文件）。
 */
#include "rtos_api.h"

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

/* 统一超时换算：RTOS_WAIT_FOREVER -> portMAX_DELAY */
static TickType_t rtos_ms_to_ticks(uint32_t ms)
{
    return (ms == RTOS_WAIT_FOREVER) ? portMAX_DELAY : pdMS_TO_TICKS(ms);
}

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
    vTaskDelay(rtos_ms_to_ticks(ms));
}

int rtos_queue_create(uint16_t depth, uint16_t item_bytes, rtos_queue_handle_t *handle)
{
    QueueHandle_t q = xQueueCreate(depth, item_bytes);
    if (q == NULL)
    {
        return -1;
    }
    *handle = (rtos_queue_handle_t)q;
    return 0;
}

int rtos_queue_send(rtos_queue_handle_t q, const void *item, uint32_t timeout_ms)
{
    return (xQueueSend((QueueHandle_t)q, item, rtos_ms_to_ticks(timeout_ms)) == pdTRUE) ? 0 : -1;
}

int rtos_queue_send_from_isr(rtos_queue_handle_t q, const void *item)
{
    BaseType_t woken = pdFALSE;
    BaseType_t rc = xQueueSendFromISR((QueueHandle_t)q, item, &woken);
    if (woken)
    {
        portYIELD_FROM_ISR(woken);
    }
    return (rc == pdTRUE) ? 0 : -1;
}

int rtos_queue_recv(rtos_queue_handle_t q, void *item, uint32_t timeout_ms)
{
    return (xQueueReceive((QueueHandle_t)q, item, rtos_ms_to_ticks(timeout_ms)) == pdTRUE) ? 0 : -1;
}
