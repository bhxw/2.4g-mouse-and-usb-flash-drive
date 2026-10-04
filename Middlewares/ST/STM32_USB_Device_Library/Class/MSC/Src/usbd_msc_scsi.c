/**
  ******************************************************************************
  * @file    usbd_msc_scsi.c
  * @author  MCD Application Team
  * @brief   This file provides all the USBD SCSI layer functions.
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

/* BSPDependencies
- "stm32xxxxx_{eval}{discovery}{nucleo_144}.c"
- "stm32xxxxx_{eval}{discovery}_io.c"
- "stm32xxxxx_{eval}{discovery}{adafruit}_sd.c"
EndBSPDependencies */

/* Includes ------------------------------------------------------------------*/
#include "usbd_msc_bot.h"
#include "usbd_msc_scsi.h"
#include "usbd_msc.h"
#include "usbd_msc_data.h"
#include "usbd_core.h"
#include "rtos_api.h"
#include "FreeRTOS.h"
#include "portable.h"

/* ---- Deferred SCSI processing: SD ops run in the task, not the USB ISR ----
 *
 * Buffer ownership is what keeps the pipeline from dropping packets:
 *
 *   - s_buf[] is shared by both directions; a command is either READ or WRITE,
 *     never both at once, so one pair of buffers serves the whole pipeline.
 *   - WRITE: the ISR owns s_buf[s_buf_idx] from the moment EP OUT is armed
 *     until the OUT completion fires, then hands it over by queueing that
 *     index.  READ: s_buf_idx only ever names block 0's buffer; after that the
 *     task tracks its own ping-pong in s_rd_pend_buf and the queued index is
 *     ignored.
 *   - Every completion queues exactly one item, unconditionally.  The queue is
 *     the only record of outstanding work, so there is no "is a slot free"
 *     test anywhere to fail and silently swallow a packet.
 *   - The task owns s_buf[sig.buf] from dequeue until the SD op returns, and
 *     releases the other buffer to the wire before blocking, so the USB
 *     transfer and the SD access overlap.
 *   - Queue occupancy can never exceed 1: the ISR queues once per armed
 *     packet, the task re-arms once per dequeued item.  Depth 2 is margin.
 */
static uint8_t            *s_buf[2];
static volatile uint8_t    s_buf_idx       = 0;
static rtos_queue_handle_t s_scsi_sig_q    = NULL;

/* Read-ahead bookkeeping, touched only by the task. */
static uint8_t             s_rd_pend_valid = 0;  /* s_buf[s_rd_pend_buf] holds a pre-read block */
static uint8_t             s_rd_pend_buf   = 0;
static uint8_t             s_rd_failed     = 0;  /* read-ahead failed; report at next signal */

int scsi_msc_buffers_init(void)
{
    s_buf[0] = (uint8_t *)pvPortMalloc(MSC_MEDIA_PACKET);
    s_buf[1] = (uint8_t *)pvPortMalloc(MSC_MEDIA_PACKET);
    return (s_buf[0] != NULL && s_buf[1] != NULL) ? 0 : -1;
}

void scsi_msc_set_signal_queue(void *q)
{
    s_scsi_sig_q = (rtos_queue_handle_t)q;
}

/* Called from the USB ISR on a Bulk-Only Mass Storage Reset.  Items queued by
   the transfer the host just abandoned must not survive into the next command:
   drained here rather than in the task, because with nothing pending the task
   stays blocked and cannot act on the stale scsi_blk_addr/scsi_blk_len that
   MSC_BOT_Reset leaves behind.  The buffer index and the read-ahead state are
   re-established by the next SCSI_Read10/SCSI_Write10 instead. */
void scsi_msc_reset_pipeline(void)
{
    scsi_msc_sig_t stale;

    if (s_scsi_sig_q != NULL)
    {
        while (rtos_queue_recv_from_isr(s_scsi_sig_q, &stale) == 0)
        {
        }
    }
}

/* One completion interrupt == one queue item.  The queue cannot overflow: the
   ISR only ever signals for a buffer the task armed, and the task arms exactly
   one per item it dequeues. */
static void scsi_msc_signal_from_isr(uint8_t op)
{
    if (s_scsi_sig_q != NULL)
    {
        scsi_msc_sig_t sig;
        sig.op  = op;
        sig.buf = s_buf_idx;
        (void)rtos_queue_send_from_isr(s_scsi_sig_q, &sig);
    }
}

void scsi_msc_task_entry(void *param)
{
    scsi_msc_sig_t sig;
    (void)param;

    for (;;)
    {
        if (rtos_queue_recv(s_scsi_sig_q, &sig, RTOS_WAIT_FOREVER) != 0)
            continue;

        USBD_MSC_BOT_HandleTypeDef *hmsc = usbd_msc_get_hmsc();
        USBD_HandleTypeDef         *pdev = usbd_msc_get_pdev();
        uint32_t len       = MIN(hmsc->scsi_blk_len * hmsc->scsi_blk_size,
                                 MSC_MEDIA_PACKET);
        uint16_t blk_count = (uint16_t)(len / hmsc->scsi_blk_size);

        if (sig.op == SCSI_MSC_OP_WRITE)
        {
            uint8_t cur = sig.buf;                   /* buffer the ISR just filled */
            uint8_t nxt = (uint8_t)(cur ^ 1U);

            /* Arm the next OUT before the blocking SD write -- but only when
               there is one.  On the last block MSC_BOT_SendCSW() below must be
               what re-arms EP OUT, and it re-arms it for the next CBW. */
            if (blk_count < hmsc->scsi_blk_len)
            {
                s_buf_idx = nxt;
                USBD_LL_PrepareReceive(pdev, MSC_EPOUT_ADDR, s_buf[nxt], len);
            }

            if ((usbd_msc_get_fops())->Write(0, s_buf[cur],
                                             hmsc->scsi_blk_addr, blk_count) < 0)
            {
                SCSI_SenseCode(pdev, 0, HARDWARE_ERROR, WRITE_FAULT);
                MSC_BOT_SendCSW(pdev, USBD_CSW_CMD_FAILED);
                continue;
            }

            hmsc->scsi_blk_addr    += blk_count;
            hmsc->scsi_blk_len     -= blk_count;
            hmsc->csw.dDataResidue -= len;

            if (hmsc->scsi_blk_len == 0U)
            {
                MSC_BOT_SendCSW(pdev, USBD_CSW_CMD_PASSED);
            }
        }
        else if (sig.op == SCSI_MSC_OP_READ)
        {
            uint8_t cur;

            if (s_rd_failed)
            {
                /* The read-ahead failed while the previous block was still on
                   the wire.  That IN has now completed, so nothing is in
                   flight and the CSW can safely go out. */
                s_rd_failed = 0;
                SCSI_SenseCode(pdev, 0, HARDWARE_ERROR, UNRECOVERED_READ_ERROR);
                MSC_BOT_SendCSW(pdev, USBD_CSW_CMD_FAILED);
                continue;
            }

            if (s_rd_pend_valid)
            {
                cur = s_rd_pend_buf;                 /* read during the last IN */
                s_rd_pend_valid = 0;
            }
            else
            {
                cur = sig.buf;                       /* first block of the command */
                if ((usbd_msc_get_fops())->Read(0, s_buf[cur],
                                                hmsc->scsi_blk_addr,
                                                blk_count) < 0)
                {
                    SCSI_SenseCode(pdev, 0, HARDWARE_ERROR, UNRECOVERED_READ_ERROR);
                    MSC_BOT_SendCSW(pdev, USBD_CSW_CMD_FAILED);
                    continue;
                }
            }

            USBD_LL_Transmit(pdev, MSC_EPIN_ADDR, s_buf[cur], len);
            hmsc->scsi_blk_addr    += blk_count;
            hmsc->scsi_blk_len     -= blk_count;
            hmsc->csw.dDataResidue -= len;

            if (hmsc->scsi_blk_len == 0U)
            {
                hmsc->bot_state = USBD_BOT_LAST_DATA_IN;
            }
            else
            {
                /* Pull block N+1 off the SD card now, while block N's IN is
                   still on the wire -- this overlap is the whole point of the
                   second buffer.  s_buf[nxt] was transmitted two iterations
                   ago and its IN completed one iteration ago (that completion
                   is the signal that woke us), so it is free.
                   scsi_blk_addr has already been advanced past block N. */
                uint8_t  nxt  = (uint8_t)(cur ^ 1U);
                uint32_t nlen = MIN(hmsc->scsi_blk_len * hmsc->scsi_blk_size,
                                    MSC_MEDIA_PACKET);
                uint16_t nblk = (uint16_t)(nlen / hmsc->scsi_blk_size);

                if ((usbd_msc_get_fops())->Read(0, s_buf[nxt],
                                                hmsc->scsi_blk_addr,
                                                nblk) < 0)
                {
                    s_rd_failed = 1;
                }
                else
                {
                    s_rd_pend_buf   = nxt;
                    s_rd_pend_valid = 1;
                }
            }
        }
    }
}



/** @addtogroup STM32_USB_DEVICE_LIBRARY
  * @{
  */


/** @defgroup MSC_SCSI
  * @brief Mass storage SCSI layer module
  * @{
  */

/** @defgroup MSC_SCSI_Private_TypesDefinitions
  * @{
  */
/**
  * @}
  */


/** @defgroup MSC_SCSI_Private_Defines
  * @{
  */

/**
  * @}
  */


/** @defgroup MSC_SCSI_Private_Macros
  * @{
  */
/**
  * @}
  */


/** @defgroup MSC_SCSI_Private_Variables
  * @{
  */

/**
  * @}
  */


/** @defgroup MSC_SCSI_Private_FunctionPrototypes
  * @{
  */
static int8_t SCSI_TestUnitReady(USBD_HandleTypeDef *pdev, uint8_t lun, uint8_t *params);
static int8_t SCSI_Inquiry(USBD_HandleTypeDef *pdev, uint8_t lun, uint8_t *params);
static int8_t SCSI_ReadFormatCapacity(USBD_HandleTypeDef *pdev, uint8_t lun, uint8_t *params);
static int8_t SCSI_ReadCapacity10(USBD_HandleTypeDef *pdev, uint8_t lun, uint8_t *params);
static int8_t SCSI_RequestSense(USBD_HandleTypeDef *pdev, uint8_t lun, uint8_t *params);
static int8_t SCSI_StartStopUnit(USBD_HandleTypeDef *pdev, uint8_t lun, uint8_t *params);
static int8_t SCSI_ModeSense6(USBD_HandleTypeDef *pdev, uint8_t lun, uint8_t *params);
static int8_t SCSI_ModeSense10(USBD_HandleTypeDef *pdev, uint8_t lun, uint8_t *params);
static int8_t SCSI_Write10(USBD_HandleTypeDef *pdev, uint8_t lun, uint8_t *params);
static int8_t SCSI_Read10(USBD_HandleTypeDef *pdev, uint8_t lun, uint8_t *params);
static int8_t SCSI_Verify10(USBD_HandleTypeDef *pdev, uint8_t lun, uint8_t *params);
static int8_t SCSI_CheckAddressRange(USBD_HandleTypeDef *pdev, uint8_t lun,
                                     uint32_t blk_offset, uint32_t blk_nbr);

static int8_t SCSI_ProcessRead(USBD_HandleTypeDef *pdev, uint8_t lun);
static int8_t SCSI_ProcessWrite(USBD_HandleTypeDef *pdev, uint8_t lun);
/**
  * @}
  */


/** @defgroup MSC_SCSI_Private_Functions
  * @{
  */


/**
* @brief  SCSI_ProcessCmd
*         Process SCSI commands
* @param  pdev: device instance
* @param  lun: Logical unit number
* @param  params: Command parameters
* @retval status
*/
int8_t SCSI_ProcessCmd(USBD_HandleTypeDef *pdev, uint8_t lun, uint8_t *cmd)
{
  switch (cmd[0])
  {
    case SCSI_TEST_UNIT_READY:
      SCSI_TestUnitReady(pdev, lun, cmd);
      break;

    case SCSI_REQUEST_SENSE:
      SCSI_RequestSense(pdev, lun, cmd);
      break;
    case SCSI_INQUIRY:
      SCSI_Inquiry(pdev, lun, cmd);
      break;

    case SCSI_START_STOP_UNIT:
      SCSI_StartStopUnit(pdev, lun, cmd);
      break;

    case SCSI_ALLOW_MEDIUM_REMOVAL:
      SCSI_StartStopUnit(pdev, lun, cmd);
      break;

    case SCSI_MODE_SENSE6:
      SCSI_ModeSense6(pdev, lun, cmd);
      break;

    case SCSI_MODE_SENSE10:
      SCSI_ModeSense10(pdev, lun, cmd);
      break;

    case SCSI_READ_FORMAT_CAPACITIES:
      SCSI_ReadFormatCapacity(pdev, lun, cmd);
      break;

    case SCSI_READ_CAPACITY10:
      SCSI_ReadCapacity10(pdev, lun, cmd);
      break;

    case SCSI_READ10:
      SCSI_Read10(pdev, lun, cmd);
      break;

    case SCSI_WRITE10:
      SCSI_Write10(pdev, lun, cmd);
      break;

    case SCSI_VERIFY10:
      SCSI_Verify10(pdev, lun, cmd);
      break;

    default:
      SCSI_SenseCode(pdev, lun, ILLEGAL_REQUEST, INVALID_CDB);
      return -1;
  }

  return 0;
}


/**
* @brief  SCSI_TestUnitReady
*         Process SCSI Test Unit Ready Command
* @param  lun: Logical unit number
* @param  params: Command parameters
* @retval status
*/
static int8_t SCSI_TestUnitReady(USBD_HandleTypeDef  *pdev, uint8_t lun, uint8_t *params)
{
  USBD_MSC_BOT_HandleTypeDef  *hmsc = usbd_msc_get_hmsc();

  (void)params;

  if ((usbd_msc_get_fops())->IsReady(lun) != 0)
  {
    SCSI_SenseCode(pdev, lun, NOT_READY, MEDIUM_NOT_PRESENT);
    hmsc->bot_state = USBD_BOT_NO_DATA;

    return -1;
  }
  hmsc->bot_data_length = 0U;

  return 0;
}

/**
* @brief  SCSI_Inquiry
*         Process Inquiry command
* @param  lun: Logical unit number
* @param  params: Command parameters
* @retval status
*/
static int8_t  SCSI_Inquiry(USBD_HandleTypeDef  *pdev, uint8_t lun, uint8_t *params)
{
  uint8_t *pPage;
  uint16_t len;
  USBD_MSC_BOT_HandleTypeDef  *hmsc = usbd_msc_get_hmsc();

  if (params[1] & 0x01U)/*Evpd is set*/
  {
    len = LENGTH_INQUIRY_PAGE00;
    hmsc->bot_data_length = len;

    while (len)
    {
      len--;
      hmsc->bot_data[len] = MSC_Page00_Inquiry_Data[len];
    }
  }
  else
  {
    pPage = (uint8_t *)(void *) & (usbd_msc_get_fops())->pInquiry[lun * STANDARD_INQUIRY_DATA_LEN];
    len = (uint16_t)pPage[4] + 5U;

    if (params[4] <= len)
    {
      len = params[4];
    }
    hmsc->bot_data_length = len;

    while (len)
    {
      len--;
      hmsc->bot_data[len] = pPage[len];
    }
  }

  return 0;
}

/**
* @brief  SCSI_ReadCapacity10
*         Process Read Capacity 10 command
* @param  lun: Logical unit number
* @param  params: Command parameters
* @retval status
*/
static int8_t SCSI_ReadCapacity10(USBD_HandleTypeDef  *pdev, uint8_t lun, uint8_t *params)
{
  USBD_MSC_BOT_HandleTypeDef  *hmsc = usbd_msc_get_hmsc();

  if ((usbd_msc_get_fops())->GetCapacity(lun, &hmsc->scsi_blk_nbr, &hmsc->scsi_blk_size) != 0)
  {
    SCSI_SenseCode(pdev, lun, NOT_READY, MEDIUM_NOT_PRESENT);
    return -1;
  }
  else
  {

    hmsc->bot_data[0] = (uint8_t)((hmsc->scsi_blk_nbr - 1U) >> 24);
    hmsc->bot_data[1] = (uint8_t)((hmsc->scsi_blk_nbr - 1U) >> 16);
    hmsc->bot_data[2] = (uint8_t)((hmsc->scsi_blk_nbr - 1U) >>  8);
    hmsc->bot_data[3] = (uint8_t)(hmsc->scsi_blk_nbr - 1U);

    hmsc->bot_data[4] = (uint8_t)(hmsc->scsi_blk_size >>  24);
    hmsc->bot_data[5] = (uint8_t)(hmsc->scsi_blk_size >>  16);
    hmsc->bot_data[6] = (uint8_t)(hmsc->scsi_blk_size >>  8);
    hmsc->bot_data[7] = (uint8_t)(hmsc->scsi_blk_size);

    hmsc->bot_data_length = 8U;
    return 0;
  }
}
/**
* @brief  SCSI_ReadFormatCapacity
*         Process Read Format Capacity command
* @param  lun: Logical unit number
* @param  params: Command parameters
* @retval status
*/
static int8_t SCSI_ReadFormatCapacity(USBD_HandleTypeDef  *pdev, uint8_t lun, uint8_t *params)
{
  USBD_MSC_BOT_HandleTypeDef  *hmsc = usbd_msc_get_hmsc();

  uint16_t blk_size;
  uint32_t blk_nbr;
  uint16_t i;

  for (i = 0U; i < 12U ; i++)
  {
    hmsc->bot_data[i] = 0U;
  }

  if ((usbd_msc_get_fops())->GetCapacity(lun, &blk_nbr, &blk_size) != 0U)
  {
    SCSI_SenseCode(pdev, lun, NOT_READY, MEDIUM_NOT_PRESENT);
    return -1;
  }
  else
  {
    hmsc->bot_data[3] = 0x08U;
    hmsc->bot_data[4] = (uint8_t)((blk_nbr - 1U) >> 24);
    hmsc->bot_data[5] = (uint8_t)((blk_nbr - 1U) >> 16);
    hmsc->bot_data[6] = (uint8_t)((blk_nbr - 1U) >>  8);
    hmsc->bot_data[7] = (uint8_t)(blk_nbr - 1U);

    hmsc->bot_data[8] = 0x02U;
    hmsc->bot_data[9] = (uint8_t)(blk_size >>  16);
    hmsc->bot_data[10] = (uint8_t)(blk_size >>  8);
    hmsc->bot_data[11] = (uint8_t)(blk_size);

    hmsc->bot_data_length = 12U;
    return 0;
  }
}
/**
* @brief  SCSI_ModeSense6
*         Process Mode Sense6 command
* @param  lun: Logical unit number
* @param  params: Command parameters
* @retval status
*/
static int8_t SCSI_ModeSense6(USBD_HandleTypeDef  *pdev, uint8_t lun, uint8_t *params)
{
  USBD_MSC_BOT_HandleTypeDef  *hmsc = usbd_msc_get_hmsc();
  uint16_t len = 8U;
  hmsc->bot_data_length = len;

  while (len)
  {
    len--;
    hmsc->bot_data[len] = MSC_Mode_Sense6_data[len];
  }
  return 0;
}

/**
* @brief  SCSI_ModeSense10
*         Process Mode Sense10 command
* @param  lun: Logical unit number
* @param  params: Command parameters
* @retval status
*/
static int8_t SCSI_ModeSense10(USBD_HandleTypeDef  *pdev, uint8_t lun, uint8_t *params)
{
  uint16_t len = 8U;
  USBD_MSC_BOT_HandleTypeDef  *hmsc = usbd_msc_get_hmsc();

  hmsc->bot_data_length = len;

  while (len)
  {
    len--;
    hmsc->bot_data[len] = MSC_Mode_Sense10_data[len];
  }

  return 0;
}

/**
* @brief  SCSI_RequestSense
*         Process Request Sense command
* @param  lun: Logical unit number
* @param  params: Command parameters
* @retval status
*/

static int8_t SCSI_RequestSense(USBD_HandleTypeDef  *pdev, uint8_t lun, uint8_t *params)
{
  uint8_t i;
  USBD_MSC_BOT_HandleTypeDef  *hmsc = usbd_msc_get_hmsc();

  for (i = 0U ; i < REQUEST_SENSE_DATA_LEN; i++)
  {
    hmsc->bot_data[i] = 0U;
  }

  hmsc->bot_data[0] = 0x70U;
  hmsc->bot_data[7] = REQUEST_SENSE_DATA_LEN - 6U;

  if ((hmsc->scsi_sense_head != hmsc->scsi_sense_tail))
  {

    hmsc->bot_data[2]     = hmsc->scsi_sense[hmsc->scsi_sense_head].Skey;
    hmsc->bot_data[12]    = hmsc->scsi_sense[hmsc->scsi_sense_head].w.b.ASCQ;
    hmsc->bot_data[13]    = hmsc->scsi_sense[hmsc->scsi_sense_head].w.b.ASC;
    hmsc->scsi_sense_head++;

    if (hmsc->scsi_sense_head == SENSE_LIST_DEEPTH)
    {
      hmsc->scsi_sense_head = 0U;
    }
  }
  hmsc->bot_data_length = REQUEST_SENSE_DATA_LEN;

  if (params[4] <= REQUEST_SENSE_DATA_LEN)
  {
    hmsc->bot_data_length = params[4];
  }
  return 0;
}

/**
* @brief  SCSI_SenseCode
*         Load the last error code in the error list
* @param  lun: Logical unit number
* @param  sKey: Sense Key
* @param  ASC: Additional Sense Key
* @retval none

*/
void SCSI_SenseCode(USBD_HandleTypeDef  *pdev, uint8_t lun, uint8_t sKey, uint8_t ASC)
{
  USBD_MSC_BOT_HandleTypeDef  *hmsc = usbd_msc_get_hmsc();

  hmsc->scsi_sense[hmsc->scsi_sense_tail].Skey  = sKey;
  hmsc->scsi_sense[hmsc->scsi_sense_tail].w.ASC = ASC << 8;
  hmsc->scsi_sense_tail++;
  if (hmsc->scsi_sense_tail == SENSE_LIST_DEEPTH)
  {
    hmsc->scsi_sense_tail = 0U;
  }
}
/**
* @brief  SCSI_StartStopUnit
*         Process Start Stop Unit command
* @param  lun: Logical unit number
* @param  params: Command parameters
* @retval status
*/
static int8_t SCSI_StartStopUnit(USBD_HandleTypeDef  *pdev, uint8_t lun, uint8_t *params)
{
  USBD_MSC_BOT_HandleTypeDef  *hmsc = usbd_msc_get_hmsc();
  hmsc->bot_data_length = 0U;
  return 0;
}

/**
* @brief  SCSI_Read10
*         Process Read10 command
* @param  lun: Logical unit number
* @param  params: Command parameters
* @retval status
*/
static int8_t SCSI_Read10(USBD_HandleTypeDef *pdev, uint8_t lun, uint8_t *params)
{
  USBD_MSC_BOT_HandleTypeDef  *hmsc = usbd_msc_get_hmsc();

  if (hmsc->bot_state == USBD_BOT_IDLE) /* Idle */
  {
    /* case 10 : Ho <> Di */
    if ((hmsc->cbw.bmFlags & 0x80U) != 0x80U)
    {
      SCSI_SenseCode(pdev, hmsc->cbw.bLUN, ILLEGAL_REQUEST, INVALID_CDB);
      return -1;
    }

    if ((usbd_msc_get_fops())->IsReady(lun) != 0)
    {
      SCSI_SenseCode(pdev, lun, NOT_READY, MEDIUM_NOT_PRESENT);
      return -1;
    }

    hmsc->scsi_blk_addr = ((uint32_t)params[2] << 24) |
                          ((uint32_t)params[3] << 16) |
                          ((uint32_t)params[4] <<  8) |
                          (uint32_t)params[5];

    hmsc->scsi_blk_len = ((uint32_t)params[7] <<  8) | (uint32_t)params[8];

    if (SCSI_CheckAddressRange(pdev, lun, hmsc->scsi_blk_addr,
                               hmsc->scsi_blk_len) < 0)
    {
      return -1; /* error */
    }

    hmsc->bot_state = USBD_BOT_DATA_IN;

    /* cases 4,5 : Hi <> Dn */
    if (hmsc->cbw.dDataLength != (hmsc->scsi_blk_len * hmsc->scsi_blk_size))
    {
      SCSI_SenseCode(pdev, hmsc->cbw.bLUN, ILLEGAL_REQUEST, INVALID_CDB);
      return -1;
    }

    hmsc->bot_data_length = MSC_MEDIA_PACKET;

    /* New command: drop any read-ahead state carried over from a previous one,
       and point the ISR at the buffer that will hold block 0. */
    s_buf_idx       = 0;
    s_rd_pend_valid = 0;
    s_rd_failed     = 0;

    return SCSI_ProcessRead(pdev, lun);
  }
  else /* Read Process ongoing */
  {
    return SCSI_ProcessRead(pdev, lun);
  }
}

/**
* @brief  SCSI_Write10
*         Process Write10 command
* @param  lun: Logical unit number
* @param  params: Command parameters
* @retval status
*/

static int8_t SCSI_Write10(USBD_HandleTypeDef  *pdev, uint8_t lun, uint8_t *params)
{
  USBD_MSC_BOT_HandleTypeDef  *hmsc = usbd_msc_get_hmsc();
  uint32_t len;

  if (hmsc->bot_state == USBD_BOT_IDLE) /* Idle */
  {
    /* case 8 : Hi <> Do */
    if ((hmsc->cbw.bmFlags & 0x80U) == 0x80U)
    {
      SCSI_SenseCode(pdev, hmsc->cbw.bLUN, ILLEGAL_REQUEST, INVALID_CDB);
      return -1;
    }

    /* Check whether Media is ready */
    if ((usbd_msc_get_fops())->IsReady(lun) != 0)
    {
      SCSI_SenseCode(pdev, lun, NOT_READY, MEDIUM_NOT_PRESENT);
      return -1;
    }

    /* Check If media is write-protected */
    if ((usbd_msc_get_fops())->IsWriteProtected(lun) != 0)
    {
      SCSI_SenseCode(pdev, lun, NOT_READY, WRITE_PROTECTED);
      return -1;
    }

    hmsc->scsi_blk_addr = ((uint32_t)params[2] << 24) |
                          ((uint32_t)params[3] << 16) |
                          ((uint32_t)params[4] << 8) |
                          (uint32_t)params[5];

    hmsc->scsi_blk_len = ((uint32_t)params[7] << 8) |
                         (uint32_t)params[8];

    /* check if LBA address is in the right range */
    if (SCSI_CheckAddressRange(pdev, lun, hmsc->scsi_blk_addr,
                               hmsc->scsi_blk_len) < 0)
    {
      return -1; /* error */
    }

    len = hmsc->scsi_blk_len * hmsc->scsi_blk_size;

    /* cases 3,11,13 : Hn,Ho <> D0 */
    if (hmsc->cbw.dDataLength != len)
    {
      SCSI_SenseCode(pdev, hmsc->cbw.bLUN, ILLEGAL_REQUEST, INVALID_CDB);
      return -1;
    }

    len = MIN(len, MSC_MEDIA_PACKET);

    /* Host data lands straight into s_buf[0] -- the task never copies it. */
    hmsc->bot_state = USBD_BOT_DATA_OUT;
    s_buf_idx = 0;
    USBD_LL_PrepareReceive(pdev, MSC_EPOUT_ADDR, s_buf[0], len);
  }
  else /* Write Process ongoing */
  {
    return SCSI_ProcessWrite(pdev, lun);
  }
  return 0;
}


/**
* @brief  SCSI_Verify10
*         Process Verify10 command
* @param  lun: Logical unit number
* @param  params: Command parameters
* @retval status
*/

static int8_t SCSI_Verify10(USBD_HandleTypeDef  *pdev, uint8_t lun, uint8_t *params)
{
  USBD_MSC_BOT_HandleTypeDef  *hmsc = usbd_msc_get_hmsc();

  if ((params[1] & 0x02U) == 0x02U)
  {
    SCSI_SenseCode(pdev, lun, ILLEGAL_REQUEST, INVALID_FIELED_IN_COMMAND);
    return -1; /* Error, Verify Mode Not supported*/
  }

  if (SCSI_CheckAddressRange(pdev, lun, hmsc->scsi_blk_addr,
                             hmsc->scsi_blk_len) < 0)
  {
    return -1; /* error */
  }
  hmsc->bot_data_length = 0U;
  return 0;
}

/**
* @brief  SCSI_CheckAddressRange
*         Check address range
* @param  lun: Logical unit number
* @param  blk_offset: first block address
* @param  blk_nbr: number of block to be processed
* @retval status
*/
static int8_t SCSI_CheckAddressRange(USBD_HandleTypeDef *pdev, uint8_t lun,
                                     uint32_t blk_offset, uint32_t blk_nbr)
{
  USBD_MSC_BOT_HandleTypeDef  *hmsc = usbd_msc_get_hmsc();

  if ((blk_offset + blk_nbr) > hmsc->scsi_blk_nbr)
  {
    SCSI_SenseCode(pdev, lun, ILLEGAL_REQUEST, ADDRESS_OUT_OF_RANGE);
    return -1;
  }
  return 0;
}

/**
* @brief  SCSI_ProcessRead
*         Handle Read Process
* @param  lun: Logical unit number
* @retval status
*/
static int8_t SCSI_ProcessRead(USBD_HandleTypeDef  *pdev, uint8_t lun)
{
  (void)pdev;
  (void)lun;

  /* Reached once per completed IN transfer, i.e. once per block, so exactly
     one signal goes out per block.  The task owns the read-ahead bookkeeping;
     nothing here may gate on shared state or a packet gets orphaned. */
  scsi_msc_signal_from_isr(SCSI_MSC_OP_READ);
  return 0;
}

/**
* @brief  SCSI_ProcessWrite
*         Handle Write Process
* @param  lun: Logical unit number
* @retval status
*/

static int8_t SCSI_ProcessWrite(USBD_HandleTypeDef  *pdev, uint8_t lun)
{
  (void)pdev;
  (void)lun;

  /* The host's data already landed in s_buf[s_buf_idx] via PrepareReceive --
     nothing to copy, just hand the buffer over to the task. */
  scsi_msc_signal_from_isr(SCSI_MSC_OP_WRITE);
  return 0;
}
/**
  * @}
  */


/**
  * @}
  */


/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
