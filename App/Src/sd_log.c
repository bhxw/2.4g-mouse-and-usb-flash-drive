/**
 * @file sd_log.c
 * @brief SD 日志任务：挂载 FAT32 → 消费队列中的 LogRecord → 批量写 DATA.LOG（追加）
 *        写满/超时 1s 落盘一次并 f_sync；写失败自动卸载重试。
 */
#include "sd_log.h"

#include "rtos_api.h"
#include "console.h"
#include "ff.h"
#include "log_format.h"

#include "main.h"       /* HAL_GetTick */

#include <string.h>

#define LOG_Q_DEPTH         64
#define LOG_TASK_STACK      220
#define LOG_TASK_PRIO       3
#define LOG_FILE_NAME       "DATA.LOG"
#define LOG_FLUSH_MS        1000
#define LOG_MOUNT_RETRY_MS  2000

static rtos_queue_handle_t s_log_q;

void sd_log_write_packet(const MousePacket_t *p)
{
    LogRecord r;

    if (s_log_q == NULL)
    {
        return;
    }
    r.magic = LOG_MAGIC;
    r.ts_ms = HAL_GetTick();
    r.seq = p->seq;
    r.buttons = p->buttons;
    r.x = p->x;
    r.y = p->y;
    r.gx = p->gx;
    r.gy = p->gy;
    r.gz = p->gz;
    (void)rtos_queue_send(s_log_q, &r, 0);   /* 满则丢弃最旧策略的简化版：直接丢该条 */
}

static void log_task(void *param)
{
    static FATFS fs;
    static FIL file;
    static uint8_t fbuf[512];
    static uint8_t qbuf[sizeof(LogRecord)];

    unsigned int bcnt = 0;
    unsigned long rec_total = 0;
    uint32_t last_flush = 0;
    FRESULT fr;
    UINT bw;

    (void)param;

    for (;;)
    {
        /* --- 未挂载：尝试挂载并打开文件（追加模式） --- */
        fr = f_mount(&fs, "", 1);
        if (fr != FR_OK)
        {
            dbg_printf("[LOG] mount fail fr=%d\r\n", (int)fr);
            rtos_delay(LOG_MOUNT_RETRY_MS);
            continue;
        }
        fr = f_open(&file, LOG_FILE_NAME, FA_OPEN_ALWAYS | FA_WRITE);
        if (fr != FR_OK)
        {
            dbg_printf("[LOG] open fail fr=%d\r\n", (int)fr);
            (void)f_mount(NULL, "", 0);
            rtos_delay(LOG_MOUNT_RETRY_MS);
            continue;
        }
        if (f_lseek(&file, f_size(&file)) != FR_OK)
        {
            (void)f_close(&file);
            (void)f_mount(NULL, "", 0);
            continue;
        }
        bcnt = 0;
        rec_total = 0;
        last_flush = HAL_GetTick();
        dbg_printf("[LOG] DATA.LOG open tick=%lu\r\n", (unsigned long)HAL_GetTick());

        /* --- 挂载成功：消费队列并批量写盘 --- */
        while (1)
        {
            /* 非阻塞清空队列 */
            while (rtos_queue_recv(s_log_q, qbuf, 0) == 0)
            {
                memcpy(&fbuf[bcnt], qbuf, sizeof(LogRecord));
                bcnt += (unsigned int)sizeof(LogRecord);
                rec_total++;
                if (bcnt + sizeof(LogRecord) > sizeof(fbuf))
                {
                    break;   /* 先去落盘再继续收 */
                }
            }

            if (bcnt >= sizeof(fbuf) ||
                (bcnt > 0 && (HAL_GetTick() - last_flush) >= LOG_FLUSH_MS))
            {
                bw = 0;
                fr = f_write(&file, fbuf, bcnt, &bw);
                if (fr != FR_OK || bw != bcnt)
                {
                    dbg_printf("[LOG] write fail fr=%d\r\n", (int)fr);
                    break;
                }
                (void)f_sync(&file);
                bcnt = 0;
                last_flush = HAL_GetTick();
                dbg_printf("[LOG] flushed rec=%lu tick=%lu\r\n", rec_total,
                           (unsigned long)HAL_GetTick());
            }

            /* 等待新数据（500ms 超时，兼顾心跳打印） */
            if (rtos_queue_recv(s_log_q, qbuf, 500) == 0)
            {
                memcpy(&fbuf[bcnt], qbuf, sizeof(LogRecord));
                bcnt += (unsigned int)sizeof(LogRecord);
                rec_total++;
                continue;
            }
        }

        /* --- 出错或卸载：关闭并重来 --- */
        (void)f_close(&file);
        (void)f_mount(NULL, "", 0);
        dbg_printf("[LOG] closed, retry...\r\n");
    }
}

void sd_log_start(void)
{
    if (rtos_queue_create(LOG_Q_DEPTH, sizeof(LogRecord), &s_log_q) != 0)
    {
        dbg_printf("[LOG] queue create fail\r\n");
        return;
    }
    (void)rtos_task_create("sd_log", log_task, NULL,
                           LOG_TASK_STACK, LOG_TASK_PRIO, NULL);
}
