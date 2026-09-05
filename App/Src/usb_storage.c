/**
 * @file usb_storage.c
 * @brief MSC 介质层实现：LUN0 = SD 卡（扇区直通 sd_spi）。
 *        主机枚举/读写期间由上层保证 sd_log 暂停（单写者）。
 */
#include "usb_storage.h"

#include "sd_spi.h"
#include "console.h"
#include "main.h"       /* HAL_GetTick */

#include <string.h>

static volatile uint32_t s_last_active_ms = 0;

void usb_storage_ping(void)
{
    s_last_active_ms = HAL_GetTick();
}

uint32_t usb_storage_last_active_ms(void)
{
    return s_last_active_ms;
}

/* SCSI INQUIRY 数据（36 字节） */
static const int8_t s_inquiry[] =
{
    0x00, 0x00, 0x02, 0x02,                 /* 直接访问 / SCSI-2 */
    (36 - 5),                               /* additional length */
    0x00, 0x00, 0x00,
    'S', 'T', 'M', '3', '2', ' ', ' ', ' ',            /* Vendor: 8 */
    'M', 'o', 'u', 's', 'e', ' ', 'U', 'D', 'i', 's', 'k', ' ', ' ', ' ', ' ', ' ',  /* Product:16 */
    '1', '.', '0', '0',                     /* Rev: 4 */
};

static int8_t sd_storage_init(uint8_t lun)
{
    (void)lun;
    return 0;   /* SD 由 sd_log 或首次访问时初始化 */
}

static int8_t sd_storage_get_capacity(uint8_t lun, uint32_t *block_num, uint16_t *block_size)
{
    (void)lun;
    usb_storage_ping();
    *block_size = SD_BLOCK_SIZE;
    if (SD_GetBlockCount(block_num) != 0)
    {
        return -1;
    }
    return 0;
}

static int8_t sd_storage_is_ready(uint8_t lun)
{
    (void)lun;
    return 0;   /* 0 = 就绪（简化；SD 访问失败会通过 Read/Write 返回） */
}

static int8_t sd_storage_is_write_protected(uint8_t lun)
{
    (void)lun;
    return 0;
}

static int8_t sd_storage_read(uint8_t lun, uint8_t *buf, uint32_t blk_addr, uint16_t blk_len)
{
    (void)lun;
    usb_storage_ping();
    for (uint16_t i = 0; i < blk_len; i++)
    {
        if (SD_ReadBlock(blk_addr + i, buf + (uint32_t)i * SD_BLOCK_SIZE) != 0)
        {
            return -1;
        }
    }
    return 0;
}

static int8_t sd_storage_write(uint8_t lun, uint8_t *buf, uint32_t blk_addr, uint16_t blk_len)
{
    (void)lun;
    usb_storage_ping();
    for (uint16_t i = 0; i < blk_len; i++)
    {
        if (SD_WriteBlock(blk_addr + i, buf + (uint32_t)i * SD_BLOCK_SIZE) != 0)
        {
            return -1;
        }
    }
    return 0;
}

static int8_t sd_storage_get_max_lun(void)
{
    return 0;   /* 单 LUN */
}

USBD_StorageTypeDef USBD_SD_Storage_fops =
{
    sd_storage_init,
    sd_storage_get_capacity,
    sd_storage_is_ready,
    sd_storage_is_write_protected,
    sd_storage_read,
    sd_storage_write,
    sd_storage_get_max_lun,
    (int8_t *)s_inquiry,
};
