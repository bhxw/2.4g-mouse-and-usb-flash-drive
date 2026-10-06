/**
 * @file sysmon.c
 * @brief M4 系统监控实现。
 *   - IWDG：≈9.6s 溢出；空闲钩子 + 监控任务喂狗（配置 configUSE_IDLE_HOOK=1）
 *   - 监控任务：每秒计数；每 10s 打印 FreeRTOS 运行时间统计(CPU%)，每 20s 打印任务栈水位
 *   - RF 链路计数：收到/空闲 包数（供丢包评估）
 */
#include "sysmon.h"

#include "rtos_api.h"
#include "console.h"
#include "usb_storage.h"   /* msc_traffic_t：[MS] 入口流量行 */

#include "main.h"
#include "FreeRTOS.h"
#include "task.h"

#include <stddef.h>
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

/* configCHECK_FOR_STACK_OVERFLOW 原本是 0，栈溢出完全静默——不 assert、不复位、
 * 只把下方内存踩坏后继续跑，正好能表现成"某个任务的输出莫名消失而系统还活着"。
 * 改成 2（每次切换检查栈底 16 字节的 0xa5 填充）并在这里把它变成可见信号：
 * 打印任务名后死循环，喂狗停止，IWDG 约 9.6s 后复位。
 * 代价：每次上下文切换多一次 16 字节比较，约 1us，会让 gap 探针轻微偏大。 */
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    (void)xTask;

    dbg_printf("[FATAL] stack overflow: %s\r\n", pcTaskName);
    taskDISABLE_INTERRUPTS();
    for (;;)
    {
    }
}

/* FreeRTOS 11 里 vTaskList / vTaskGetRunTimeStats 不是函数，只是 task.h:2317
 * 和 :2440 的兼容宏，展开成 xxxTasks(buf, configSTATS_BUFFER_MAX_LENGTH)。两个后果：
 *   1) 宏名后面不带括号就不展开——当函数指针传会报 identifier undefined；
 *   2) 那个长度宏在说谎：configSTATS_BUFFER_MAX_LENGTH = 0xFFFF
 *      （FreeRTOSConfig.h:205），而 buf 只有 384 字节。内核的边界判断
 *      （tasks.c:7394/7406）全部基于这个传入长度，所以保护形同虚设——任务数
 *      一多就会静默写穿 sysmon 的栈。当前 5 个任务只占 135 字节（实测 len=135），
 *      还没越界，但那是靠任务少，不是靠设计。
 * 所以直接调真函数 vTaskListTasks / vTaskGetRunTimeStatistics，长度传 sizeof(buf)。 */
typedef void (*mon_dump_fn)(char *, size_t);

static void mon_dump(const char *tag, char *buf, size_t buflen, mon_dump_fn fn)
{
    dbg_printf("[MON] --- %s ---\r\n", tag);
    /* 先清零：pvPortMalloc 失败时这两个函数一个字节都不写，
     * 不清零就会把未初始化的栈内存当字符串发出去。 */
    buf[0] = '\0';
    fn(buf, buflen);
    if (buf[0] != '\0')
    {
        dbg_puts(buf);
    }
}

static void mon_task(void *param)
{
    char buf[384];
    uint32_t last_cpu = 0;
    uint32_t last_stack = 0;
    uint32_t last_mem = 0;
    uint32_t last_tr = 0;
    msc_traffic_t tr_prev = {0};

    (void)param;

    for (;;)
    {
        uint32_t sec = HAL_GetTick() / 1000U;

        /* 内存审计：每 60s 重打一次。高水位和 minEverFreeHeap 都是**累积最小值**，
         * 只记录到采样那一刻为止的最深用量——所以必须在跑完拷贝之后再读一次，
         * 开机 10s 的一次性快照完全不含 MSC 负载。实测对比：scsi_msc 空闲时只用
         * 40 字，USB 挂载后是 152 字（余量 216 → 104），差 3.8 倍。
         *
         * 周期打印不会拖慢拷贝：本任务优先级 1 < scsi_msc 的 4，抢不动它，
         * UART 忙等只发生在 scsi_msc 阻塞等 USB 的窗口里。撞车导致的丢行
         * 由 dbg_console_drops() 计数，直接打在 drops= 字段里。
         * 判据：heap free 若 >= 512B，MSC 双缓冲的第二块 shadow 可以直接
         * pvPortMalloc，不必动只剩 208B 的链接器静态预算。 */
        if (sec - last_mem >= 60U)
        {
            last_mem = sec;
            dbg_printf("[MEM] heap free=%u minEver=%u total=%u drops=%lu\r\n",
                       (unsigned)xPortGetFreeHeapSize(),
                       (unsigned)xPortGetMinimumEverFreeHeapSize(),
                       (unsigned)configTOTAL_HEAP_SIZE,
                       (unsigned long)dbg_console_drops());
        }

        /* MSC 入口流量：每 2s 一行（诊断，语义见 usb_storage.h 的 msc_traffic_t）。
         * 现有 [SD-RD]/[SD-WR] 要累计成功满 256 块才打一行，主机在准备相或重试退避期间
         * 一条都不出；这一行按"调用"计数、2 秒粒度，是唯一能把
         * 「主机没发 / 主机发了但设备回错 / 设备真慢」分开的东西，同时充当 2s 心跳。
         * 本任务优先级 1 < scsi_msc 的 4，抢不动它，不影响被测数据率。 */
        if (sec - last_tr >= 2U)
        {
            msc_traffic_t tr;

            last_tr = sec;
            usb_storage_traffic(&tr);
            dbg_printf("[MS] t=%lus rd c=%lu b=%lu f=%lu j=%lu | wr c=%lu b=%lu f=%lu j=%lu\r\n",
                       (unsigned long)sec,
                       (unsigned long)(tr.rd_calls  - tr_prev.rd_calls),
                       (unsigned long)(tr.rd_blocks - tr_prev.rd_blocks),
                       (unsigned long)(tr.rd_fails  - tr_prev.rd_fails),
                       (unsigned long)(tr.rd_jumps  - tr_prev.rd_jumps),
                       (unsigned long)(tr.wr_calls  - tr_prev.wr_calls),
                       (unsigned long)(tr.wr_blocks - tr_prev.wr_blocks),
                       (unsigned long)(tr.wr_fails  - tr_prev.wr_fails),
                       (unsigned long)(tr.wr_jumps  - tr_prev.wr_jumps));
            tr_prev = tr;
        }

        /*dbg_printf("[MON] up=%lus rx_ok=%lu rx_idle=%lu link_loss_ratio=%lu%%\r\n",
                   (unsigned long)sec, (unsigned long)s_rx_ok,
                   (unsigned long)s_rx_idle,
                   (unsigned long)((s_rx_ok + s_rx_idle) ? (s_rx_idle * 100U) / (s_rx_ok + s_rx_idle) : 0U));*/

        if (sec - last_cpu >= 10U)
        {
            last_cpu = sec;
            mon_dump("CPU run-time stats", buf, sizeof(buf), vTaskGetRunTimeStatistics);
        }
        if (sec - last_stack >= 20U)
        {
            last_stack = sec;
            mon_dump("task list (stack high-water)", buf, sizeof(buf), vTaskListTasks);
        }

        iwdg_feed();
        rtos_delay(1000);
    }
}

void sysmon_start(void)
{
    /* 原来这里 (void) 丢掉了返回值：任务创建失败（堆不够）是完全静默的，
     * 表现就是"[MON] 一条都不打，但系统其他部分正常"。必须报出来。 */
    int rc = rtos_task_create("sysmon", mon_task, NULL,
                              MON_STACK_WORDS, MON_PRIORITY, NULL);

    if (rc != 0)
    {
        dbg_printf("[BOOT] sysmon create FAIL rc=%d heap=%u\r\n", rc,
                   (unsigned)xPortGetFreeHeapSize());
    }
}
