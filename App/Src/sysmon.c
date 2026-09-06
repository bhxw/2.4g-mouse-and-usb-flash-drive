/**
 * @file sysmon.c
 * @brief M4 系统监控实现。
 *   - IWDG：~6s 溢出；空闲钩子 + 监控任务喂狗（配置 configUSE_IDLE_HOOK=1）
 *   - 监控任务：每秒计数；每 10s 打印 FreeRTOS 运行时间统计(CPU%)，每 20s 打印任务栈水位
 *   - RF 链路计数：收到/空闲 包数（供丢包评估）
 */
#include "sysmon.h"

#include "rtos_api.h"
#include "console.h"

#include "main.h"
#include "FreeRTOS.h"
#include "task.h"

#include <string.h>

#define MON_STACK_WORDS   300
#define MON_PRIORITY      1

static volatile uint32_t s_rx_ok = 0;
static volatile uint32_t s_rx_idle = 0;
static volatile uint8_t s_iwdg_on = 0;

void sysmon_rx_ok(void)   { s_rx_ok++; }
void sysmon_rx_idle(void) { s_rx_idle++; }

void sysmon_init_hw(void)
{
    uint32_t t0;
    uint16_t guard;

    /* 启动 LSI，限时等待；异常则放弃看门狗，绝不阻塞启动 */
    __HAL_RCC_LSI_ENABLE();
    t0 = HAL_GetTick();
    while (__HAL_RCC_GET_FLAG(RCC_FLAG_LSIRDY) == RESET)
    {
        if ((HAL_GetTick() - t0) > 200U)
        {
            return;
        }
    }
    /* IWDG 配置：LSI≈40kHz / 128 ≈312Hz，RLR=3000 -> 溢出约 9.6s（余量足） */
    IWDG->KR = 0x5555u;
    IWDG->PR = 0x05u;              /* 预分频 128 */
    IWDG->RLR = 3000u;
    guard = 1000u;
    while ((IWDG->SR != 0u) && (--guard != 0u)) { }   /* 有界等待 SR 清零 */
    if (guard == 0u)
    {
        return;                    /* IWDG 寄存器异常：放弃启用 */
    }
    IWDG->KR = 0xCCCCu;            /* 启动计数 */
    IWDG->KR = 0xAAAAu;            /* 立即喂一次 */
    s_iwdg_on = 1U;
}

static void iwdg_feed(void)
{
    if (s_iwdg_on)
    {
        IWDG->KR = 0xAAAAu;  /* 重新装载 */
    }
}

/* 空闲钩子：最可靠的喂狗点（configUSE_IDLE_HOOK=1） */
void vApplicationIdleHook(void)
{
    iwdg_feed();
}

static void mon_task(void *param)
{
    char buf[384];
    uint32_t last_cpu = 0;
    uint32_t last_stack = 0;

    (void)param;

    for (;;)
    {
        uint32_t sec = HAL_GetTick() / 1000U;

        dbg_printf("[MON] up=%lus rx_ok=%lu rx_idle=%lu link_loss_ratio=%lu%%\r\n",
                   (unsigned long)sec, (unsigned long)s_rx_ok,
                   (unsigned long)s_rx_idle,
                   (unsigned long)((s_rx_ok + s_rx_idle) ? (s_rx_idle * 100U) / (s_rx_ok + s_rx_idle) : 0U));

        if (sec - last_cpu >= 10U)
        {
            last_cpu = sec;
            dbg_printf("[MON] --- CPU run-time stats ---\r\n");
            vTaskGetRunTimeStats(buf);
            dbg_puts(buf);
        }
        if (sec - last_stack >= 20U)
        {
            last_stack = sec;
            dbg_printf("[MON] --- task list (stack high-water) ---\r\n");
            vTaskList(buf);
            dbg_puts(buf);
        }

        iwdg_feed();
        rtos_delay(1000);
    }
}

void sysmon_start(void)
{
    (void)rtos_task_create("sysmon", mon_task, NULL,
                           MON_STACK_WORDS, MON_PRIORITY, NULL);
}
