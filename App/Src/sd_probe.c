/**
 * @file sd_probe.c
 * @brief SD/FatFs 自检（M2b 验收用；M2c 将由 sd_log_task 取代）
 */
#include "sd_probe.h"

#include "rtos_api.h"
#include "console.h"
#include "ff.h"

#include <string.h>

#define PROBE_STACK_WORDS   200
#define PROBE_PRIORITY      2
#define PROBE_RETRY_MS      3000

static void sd_probe_entry(void *param)
{
    static FATFS fs;
    static FIL file;
    static uint8_t wbuf[512];
    static uint8_t rbuf[512];
    FRESULT fr;
    UINT bw = 0, br = 0;
    int mismatch;

    (void)param;

    for (;;)
    {
        fr = f_mount(&fs, "", 1);
        if (fr != FR_OK)
        {
            dbg_printf("[SD] mount fail fr=%d (wiring/FAT32?)\r\n", (int)fr);
            rtos_delay(PROBE_RETRY_MS);
            continue;
        }

        /* 写模式扇区（0x5A 填充） */
        memset(wbuf, 0x5A, sizeof(wbuf));
        wbuf[0] = 0xA5;
        fr = f_open(&file, "TEST.LOG", FA_CREATE_ALWAYS | FA_WRITE);
        if (fr != FR_OK)
        {
            dbg_printf("[SD] open(w) fail fr=%d\r\n", (int)fr);
            f_mount(NULL, "", 0);
            rtos_delay(PROBE_RETRY_MS);
            continue;
        }
        fr = f_write(&file, wbuf, sizeof(wbuf), &bw);
        (void)f_close(&file);

        /* 读回校验 */
        mismatch = 1;
        fr = f_open(&file, "TEST.LOG", FA_OPEN_EXISTING | FA_READ);
        if (fr == FR_OK)
        {
            br = 0;
            fr = f_read(&file, rbuf, sizeof(rbuf), &br);
            (void)f_close(&file);
            mismatch = (bw != sizeof(wbuf)) || (br != sizeof(wbuf))
                       || (memcmp(wbuf, rbuf, sizeof(wbuf)) != 0);
        }

        if (fr == FR_OK && !mismatch)
        {
            dbg_printf("[SD] TEST OK : mount + write/read verify OK (512B)\r\n");
        }
        else
        {
            dbg_printf("[SD] TEST FAIL : fr=%d bw=%u br=%u mismatch=%d\r\n",
                       (int)fr, (unsigned)bw, (unsigned)br, mismatch);
        }

        f_mount(NULL, "", 0);   /* 卸载，下轮重试（便于拔卡/换卡） */
        rtos_delay(PROBE_RETRY_MS);
    }
}

void sd_probe_start(void)
{
    (void)rtos_task_create("sd_probe", sd_probe_entry, NULL,
                           PROBE_STACK_WORDS, PROBE_PRIORITY, NULL);
}
