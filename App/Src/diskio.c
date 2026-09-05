/**
 * @file diskio.c
 * @brief FatFs 磁盘对接层：卷 0 = SD 卡（App/Src/sd_spi.c）
 */
#include "diskio.h"
#include "sd_spi.h"

DSTATUS disk_initialize(BYTE pdrv)
{
    if (pdrv != 0)
    {
        return STA_NOINIT;
    }
    return (SD_Init() == 0) ? 0 : STA_NOINIT;
}

DSTATUS disk_status(BYTE pdrv)
{
    if (pdrv != 0)
    {
        return STA_NOINIT;
    }
    return 0;   /* 简化：初始化成功后视为就绪 */
}

DRESULT disk_read(BYTE pdrv, BYTE *buff, DWORD sector, UINT count)
{
    if (pdrv != 0)
    {
        return RES_PARERR;
    }
    for (UINT i = 0; i < count; i++)
    {
        if (SD_ReadBlock((uint32_t)(sector + i), buff + (size_t)i * SD_BLOCK_SIZE) != 0)
        {
            return RES_ERROR;
        }
    }
    return RES_OK;
}

DRESULT disk_write(BYTE pdrv, const BYTE *buff, DWORD sector, UINT count)
{
    if (pdrv != 0)
    {
        return RES_PARERR;
    }
    for (UINT i = 0; i < count; i++)
    {
        if (SD_WriteBlock((uint32_t)(sector + i), buff + (size_t)i * SD_BLOCK_SIZE) != 0)
        {
            return RES_ERROR;
        }
    }
    return RES_OK;
}

DRESULT disk_ioctl(BYTE pdrv, BYTE cmd, void *buff)
{
    if (pdrv != 0)
    {
        return RES_PARERR;
    }
    switch (cmd)
    {
    case CTRL_SYNC:
        return RES_OK;
    case GET_BLOCK_SIZE:      /* 擦除块粒度，按 512B 扇区计，取 1 */
        *(DWORD *)buff = 1;
        return RES_OK;
    default:
        return RES_PARERR;
    }
}
