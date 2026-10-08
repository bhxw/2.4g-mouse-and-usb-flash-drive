/**
  ******************************************************************************
  * @file    usbd_msc.h
  * @author  MCD Application Team
  * @brief   Header for the usbd_msc.c file
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2015 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                      www.st.com/SLA0044
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USBD_MSC_H
#define __USBD_MSC_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include  "usbd_msc_bot.h"
#include  "usbd_msc_scsi.h"
#include  "usbd_ioreq.h"

/** @addtogroup USBD_MSC_BOT
  * @{
  */

/** @defgroup USBD_MSC
  * @brief This file is the Header file for usbd_msc.c
  * @{
  */


/** @defgroup USBD_BOT_Exported_Defines
  * @{
  */
/* MSC Class Config */
#ifndef MSC_MEDIA_PACKET
#define MSC_MEDIA_PACKET             512U
#endif /* MSC_MEDIA_PACKET */

/* Media bytes moved per USB transfer: two media packets.  Only the SCSI data
 * pipeline uses this size, and its two ping-pong buffers come from the
 * FreeRTOS heap (scsi_msc_buffers_init), so raising it costs no static RAM.
 * bot_data[] below stays at MSC_MEDIA_PACKET: USBD_static_malloc() holds the
 * whole MSC handle in a 1 KiB static block, and bot_data_length must never
 * exceed sizeof(bot_data) (MSC_BOT_SendData() clamps only against
 * cbw.dDataLength). */
#ifndef MSC_STREAM_PACKET
#define MSC_STREAM_PACKET            (2U * MSC_MEDIA_PACKET)
#endif /* MSC_STREAM_PACKET */

#define MSC_MAX_FS_PACKET            0x40U
#define MSC_MAX_HS_PACKET            0x200U

#define BOT_GET_MAX_LUN              0xFE
#define BOT_RESET                    0xFF
#define USB_MSC_CONFIG_DESC_SIZ      32


/* MSC 的 IN 端点号必须从 0x82 挪到 0x83：F1 一个端点号只有一位 EP_KIND，
 * 而双缓冲把 btable 每端点的 4 个半字全用掉（BUF0=TX_ADDR/+2 槽、BUF1=RX_ADDR/+6 槽），
 * 同一端点号不可能两个方向都开双缓冲。
 * 2026-10-05 三轮对照（开发日志第十二/十七/十八条）：
 *   ① HAL 的 USE_USB_DOUBLE_BUFFER 路径：两个方向都不可用（B 格 IN 读崩、C 格 OUT 写卡死）；
 *   ② A 格（EP3 + 新 PMA 布局、两方向仍单缓冲）全绿 ⇒ 端点号/布局本身无罪；
 *   ③ 现版本改由自研层 usbd_pma_db.c 驱动 EP2 OUT / EP3 IN 的双缓冲。 */
#define MSC_EPIN_ADDR                0x83U
#define MSC_EPOUT_ADDR               0x02U

/**
  * @}
  */

/** @defgroup USB_CORE_Exported_Types
  * @{
  */
typedef struct _USBD_STORAGE
{
  int8_t (* Init)(uint8_t lun);
  int8_t (* GetCapacity)(uint8_t lun, uint32_t *block_num, uint16_t *block_size);
  int8_t (* IsReady)(uint8_t lun);
  int8_t (* IsWriteProtected)(uint8_t lun);
  int8_t (* Read)(uint8_t lun, uint8_t *buf, uint32_t blk_addr, uint16_t blk_len);
  int8_t (* Write)(uint8_t lun, uint8_t *buf, uint32_t blk_addr, uint16_t blk_len);
  int8_t (* GetMaxLun)(void);
  int8_t *pInquiry;

} USBD_StorageTypeDef;


typedef struct
{
  uint32_t                 max_lun;
  uint32_t                 interface;
  uint8_t                  bot_state;
  uint8_t                  bot_status;
  uint16_t                 bot_data_length;
  uint8_t                  bot_data[MSC_MEDIA_PACKET];
  USBD_MSC_BOT_CBWTypeDef  cbw;
  USBD_MSC_BOT_CSWTypeDef  csw;

  USBD_SCSI_SenseTypeDef   scsi_sense [SENSE_LIST_DEEPTH];
  uint8_t                  scsi_sense_head;
  uint8_t                  scsi_sense_tail;

  uint16_t                 scsi_blk_size;
  uint32_t                 scsi_blk_nbr;

  uint32_t                 scsi_blk_addr;
  uint32_t                 scsi_blk_len;
}
USBD_MSC_BOT_HandleTypeDef;

/* Structure for MSC process */
extern USBD_ClassTypeDef  USBD_MSC;
#define USBD_MSC_CLASS    &USBD_MSC

/* [BOT] 写路径事件计数（实现在 usbd_msc_bot.c）：ISR 里只记数，打印由 sysmon 任务每 2s
 * 调 USBD_MSC_BotStatsDump() —— 在 ISR 里 dbg_printf 会阻塞 UART 顶住 USB 端点。 */
void USBD_MSC_BotCbw(uint32_t tag, uint8_t cdb0, uint32_t dlen, uint16_t rx, uint32_t blk_len);
void USBD_MSC_BotCsw(uint8_t status, uint32_t residue);
void USBD_MSC_BotBadCbw(uint16_t rx, uint32_t sig, uint8_t cbllen);
void USBD_MSC_BotBadWrite(uint32_t dlen, uint32_t blk_x512);
void USBD_MSC_BotAbort(void);
void USBD_MSC_BotSig(void);
void USBD_MSC_BotSigDrop(void);
void USBD_MSC_BotChunk(uint32_t addr, uint16_t n, uint8_t arm);
void USBD_MSC_BotStatsDump(void);

/* 失败但数据阶段没结清时的收尾：STALL EP OUT（作废残留数据）后回 CSW，不 re-arm CBW。
 * 调用者保证 hmsc->csw.dDataResidue != 0。详见 usbd_msc_bot.c 的实现注释。 */
void MSC_BOT_SendCSW_StallOut(USBD_HandleTypeDef *pdev);

uint8_t  USBD_MSC_RegisterStorage(USBD_HandleTypeDef   *pdev,
                                  USBD_StorageTypeDef *fops);
USBD_MSC_BOT_HandleTypeDef *usbd_msc_get_hmsc(void);
USBD_StorageTypeDef        *usbd_msc_get_fops(void);
USBD_HandleTypeDef         *usbd_msc_get_pdev(void);
/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif  /* __USBD_MSC_H */
/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
