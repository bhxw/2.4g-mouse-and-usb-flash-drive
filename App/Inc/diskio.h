/*---------------------------------------------------------------------------/
/  diskio.h：磁盘底层接口（R0.15 经典结构）
/---------------------------------------------------------------------------*/
#ifndef _DISKIO_DEFINED
#define _DISKIO_DEFINED

#ifdef __cplusplus
extern "C" {
#endif

#include "ff.h"   /* BYTE / UINT / DWORD 等类型 */

/* 状态标志 */
#define STA_NOINIT      0x01
#define STA_NODISK      0x02
#define STA_PROTECT     0x04

typedef BYTE DSTATUS;

/* 结果码 */
typedef enum {
    RES_OK = 0,
    RES_ERROR,
    RES_WRPRT,
    RES_NOTRDY,
    RES_PARERR
} DRESULT;

/* ioctl 命令 */
#define CTRL_SYNC           0
#define GET_SECTOR_COUNT    1
#define GET_SECTOR_SIZE     2
#define GET_BLOCK_SIZE      3
#define CTRL_TRIM           4

DSTATUS disk_initialize(BYTE pdrv);
DSTATUS disk_status(BYTE pdrv);
DRESULT disk_read(BYTE pdrv, BYTE *buff, DWORD sector, UINT count);
DRESULT disk_write(BYTE pdrv, const BYTE *buff, DWORD sector, UINT count);
DRESULT disk_ioctl(BYTE pdrv, BYTE cmd, void *buff);

#ifdef __cplusplus
}
#endif

#endif /* _DISKIO_DEFINED */
