/**
 * @file app_main.c
 * @brief 应用层任务（M2c/M3）
 *        任务集：rf_rx（射频收包→HID 上报 + 投递日志）、sd_log（批量写 DATA.LOG）。
 *        所有任务一律通过 rtos_api 访问内核。
 */
#include "app_main.h"
#include "rtos_api.h"

#include "nrf_demo.h"   /* MousePacket_t */
#include "nrf24l01.h"
#include "sd_log.h"
#include "usb_device.h"
#include "usbd_hid.h"   /* USBD_HID_SendReport */
#include "console.h"

extern USBD_HandleTypeDef hUsbDeviceFS;

#define RF_TASK_STACK_WORDS   160
#define RF_TASK_PRIORITY      5
#define RF_POLL_MS            10

static rtos_task_handle_t s_rf_task;

/** 射频接收任务：10ms 轮询收包，收到即按 HID 鼠标报告上送并写入日志 */
static void rf_rx_task(void *param)
{
    MousePacket_t pack = {0};
    int8_t mouseout[4] = {0, 0, 0, 0};
    uint32_t last_busy_print = 0;

    (void)param;

    for (;;)
    {
        if (NRF24L01_RxPacket((uint8_t *)&pack) == 0)
        {
            mouseout[0] = pack.buttons;
            mouseout[1] = pack.x;
            mouseout[2] = pack.y;
            if (USBD_HID_SendReport(&hUsbDeviceFS, (uint8_t *)mouseout, sizeof(mouseout)) != USBD_OK)
            {
                if (HAL_GetTick() - last_busy_print > 2000U)
                {
                    dbg_printf("[RF] HID report busy\r\n");
                    last_busy_print = HAL_GetTick();
                }
            }

            sd_log_write_packet(&pack);
        }
        rtos_delay(RF_POLL_MS);
    }
}

void app_start(void)
{
    /* 创建任务失败不阻塞后续：调度器启动时低优先级任务仍可运行 */
    (void)rtos_task_create("rf_rx", rf_rx_task, NULL,
                           RF_TASK_STACK_WORDS, RF_TASK_PRIORITY, &s_rf_task);

    /* M2c：SD 日志任务（无卡时周期打印 mount fail 重试，不阻塞鼠标） */
    sd_log_start();

    rtos_scheduler_start();

    /* 只有调度器启动失败才会执行到此处：死循环兜底 */
    for (;;)
    {
    }
}
