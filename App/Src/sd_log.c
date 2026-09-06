/**
 * @file sd_log.c
 * @brief SD 日志任务：挂载 FAT32 → 消费队列中的 LogRecord → 批量写 DATA.LOG（追加）
 *
 * 单写者互斥：主机（U盘/MSC）正在读写 SD 时暂停本任务落盘（usb_storage_last_active_ms
 * 距今 < USB_GATE_MS 视为占用），空闲后自动恢复；缓冲写满且被占用时丢弃新记录并计数。
 */
#include "sd_log.h"

#include "rtos_api.h"
#include "console.h"
#include "ff.h"
#include "log_format.h"
#include "usb_storage.h"
#include "usbd_def.h"

#include "main.h"       /* HAL_GetTick */

#include <string.h>

#define LOG_Q_DEPTH         64
#define LOG_TASK_STACK      220
#define LOG_TASK_PRIO       3
#define LOG_FILE_NAME       "DATA.LOG"
#define LOG_FLUSH_MS        1000
#define LOG_MOUNT_RETRY_MS  2000
#define USB_GATE_MS         1500

static rtos_queue_handle_t s_log_q;

extern USBD_HandleTypeDef hUsbDeviceFS;

static int usb_busy(void)
{
    return (HAL_GetTick() - usb_storage_last_active_ms()) < USB_GATE_MS;
}

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
    (void)rtos_queue_send(s_log_q, &r, 0);   /* 队列满则丢弃 */
}

/* 落盘：成功返回 1；被 USB 占用返回 0 */
static int log_flush(FIL *file, uint8_t *fbuf, unsigned int *bcnt,
                     unsigned long *rec_total, unsigned long *drops)
{
    FRESULT fr;
    UINT bw = 0;

    if (*bcnt == 0)
    {
        return 1;
    }
    fr = f_write(file, fbuf, *bcnt, &bw);
    if (fr != FR_OK || bw != *bcnt)
    {
        dbg_printf("[LOG] write fail fr=%d\r\n", (int)fr);
        return -1;
    }
    (void)f_sync(file);
    *bcnt = 0;
    if (*drops > 0)
    {
        dbg_printf("[LOG] flushed rec=%lu dropped=%lu\r\n", *rec_total, *drops);
        *drops = 0;
    }
    else
    {
        dbg_printf("[LOG] flushed rec=%lu\r\n", *rec_total);
    }
    return 1;
}

/* 尝试追加一条：缓冲满先尝试落盘；USB 占用导致无法落盘则丢弃该条 */
static void log_append(FIL *file, uint8_t *fbuf, const LogRecord *r,
                       unsigned int *bcnt, unsigned long *rec_total,
                       unsigned long *drops)
{
    if (*bcnt + (unsigned int)sizeof(LogRecord) > 512U)
    {
        int rc = log_flush(file, fbuf, bcnt, rec_total, drops);
        if (rc <= 0)
        {
            (*drops)++;
            return;   /* 占用或写失败：丢当前条 */
        }
    }
    memcpy(&fbuf[*bcnt], r, sizeof(LogRecord));
    *bcnt += (unsigned int)sizeof(LogRecord);
    (*rec_total)++;
}

static void log_task(void *param)
{
    static FATFS fs;
    static FIL file;
    static uint8_t fbuf[512];
    static uint8_t qbuf[sizeof(LogRecord)];

    unsigned int bcnt = 0;
    unsigned long rec_total = 0;
    unsigned long drops = 0;
    uint32_t last_flush = 0;
    FRESULT fr;

    (void)param;

    /* 上电错峰：先让 USB 枚举稳定，再碰 SD（避免开机即写卡导致电流尖峰/枚举失败） */
    rtos_delay(3000);

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
        drops = 0;
        last_flush = HAL_GetTick();
        dbg_printf("[LOG] DATA.LOG open tick=%lu\r\n", (unsigned long)HAL_GetTick());

        /* --- 挂载成功：消费队列并批量写盘 --- */
        while (1)
        {
            /* 非阻塞清空队列 */
            while (rtos_queue_recv(s_log_q, qbuf, 0) == 0)
            {
                log_append(&file, fbuf, (const LogRecord *)qbuf,
                           &bcnt, &rec_total, &drops);
            }

            /* 可写且（满 或 超时）→ 落盘 */
            if (bcnt > 0 && !usb_busy() &&
                (bcnt >= 512U || (HAL_GetTick() - last_flush) >= LOG_FLUSH_MS))
            {
                int rc = log_flush(&file, fbuf, &bcnt, &rec_total, &drops);
                if (rc < 0)
                {
                    break;   /* 写失败：重挂载 */
                }
                last_flush = HAL_GetTick();
            }

            /* 等待新数据（500ms 超时） */
            if (rtos_queue_recv(s_log_q, qbuf, 500) == 0)
            {
                log_append(&file, fbuf, (const LogRecord *)qbuf,
                           &bcnt, &rec_total, &drops);
                continue;
            }
        }

        /* --- 出错：关闭并重来 --- */
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
