#ifndef __SD_SPI_H
#define __SD_SPI_H

/**
 * @file sd_spi.h
 * @brief SD 卡 SPI 底层驱动（SPI1 默认映射 PA5/6/7，CS=PB12）
 *
 * 时序参考：Arduino SdFat (Sd2Card) + FAT 初始化流程（参见 参考历程/SD/utility/Sd2Card.cpp）
 * 首版采用阻塞传输确保可用；DMA 句柄 hdma_spi1_rx/tx 已由 CubeMX 配置，后续按需切换。
 */

#include <stdint.h>

#define SD_BLOCK_SIZE   512u

/** 返回 0 表示成功，非 0 为错误码（见 sd_spi.c 内注释） */
uint8_t SD_Init(void);
uint8_t SD_ReadBlock(uint32_t block, uint8_t *buf);
uint8_t SD_WriteBlock(uint32_t block, const uint8_t *buf);

#endif /* __SD_SPI_H */
