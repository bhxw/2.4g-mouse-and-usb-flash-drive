/**
 * @file usbd_pma_db.h
 * @brief 自研 PMA 双缓冲（MSC 的 EP2 OUT / EP3 IN）：寄存器级状态机，绕开 ST HAL 的
 *        USE_USB_DOUBLE_BUFFER 路径（那条路对 MSC 的"31B CBW / 多包数据 / 13B CSW"
 *        混跑负载两个方向都不可用，见 local/dev/开发日志.md 2026-10-05 第十二~十七条）。
 *
 * 蓝本（两份，均已在本仓库或本地留存）：
 *   - TeenyUSB driver_stm32/tusb_dev_drv_stm32_fs.c（MIT，同一套 F1 USB_FS IP）：
 *     EP_KIND 当持久标志、运行期不切模式、每包先翻 SW_BUF 再只填刚释放的那块、
 *     每块 count 单独重设。
 *   - 旧 ST USB-FS-Device 库 usb_regs.c 的 FreeUserBuffer()：SW_BUF 翻转是无条件的。
 */
#ifndef __USBD_PMA_DB_H
#define __USBD_PMA_DB_H

#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_pcd.h"

/* 单向开关：置 0 即该方向退回 HAL 单缓冲。PMA 地址表不用改（两块用的就是同样的
 * 起始地址：OUT 0x100、IN 0x180），只是 SNG_BUF / DBL_BUF 的选择跟着变。 */
#ifndef MSC_DB_OUT
#define MSC_DB_OUT   1
#endif
#ifndef MSC_DB_IN
#define MSC_DB_IN    1
#endif

/* 双缓冲要求同一端点号只服务一个方向：F1 一个端点号只有一位 EP_KIND，
 * 且 btable 每端点的 4 个半字被两块共用 ⇒ MSC 的 IN 必须在 EP3。 */
#define PMA_DB_EP_ADDR_OUT   0x02U
#define PMA_DB_EP_ADDR_IN    0x83U

void     PMA_DB_Init(PCD_HandleTypeDef *hpcd);
uint8_t  PMA_DB_IsDbEp(uint8_t ep_addr);

HAL_StatusTypeDef PMA_DB_Transmit(uint8_t ep_addr, uint8_t *pbuf, uint16_t len);
HAL_StatusTypeDef PMA_DB_PrepareReceive(uint8_t ep_addr, uint8_t *pbuf, uint16_t len);

void     PMA_DB_OpenEp(uint8_t ep_addr);
void     PMA_DB_Reset(void);
void     PMA_DB_IRQHandler(void);

void     PMA_DB_Report(void);
void     PMA_DB_GetCounters(uint32_t *in_pkt, uint32_t *out_pkt,
                            uint32_t *resync, uint32_t *anom);

#endif /* __USBD_PMA_DB_H */
