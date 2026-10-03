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

/**
 * @brief CMD25 流式多块写：Begin 打开会话 -> 连续 Chunk -> End 关闭
 *
 * 会话在整个过程中保持 CS 低电平，因此**期间不得调用任何其它 SD 接口**。
 * 若被违规调用，其它接口的 sd_cs_low() 会自动发 STOP_TRAN 终结会话（见 sd_spi.c）。
 * Chunk 只接受严格连续递增的块号，跳号返回非 0，需重新 Begin。
 * End 可安全重复调用；未 Begin 时为空操作。
 */
uint8_t SD_WriteBegin(uint32_t block);
uint8_t SD_WriteChunk(uint32_t block, const uint8_t *buf);
uint8_t SD_WriteEnd(void);

/** 查询总扇区数（CSD 解析；SDHC 为块地址模式）。成功返回 0。 */
uint8_t SD_GetBlockCount(uint32_t *blocks);

/** 已初始化成功？ */
uint8_t SD_Ready(void);

/**
 * @brief 取走并清零轮询圈数计数器（纯测量用）
 *
 * sd_wait_ready 与 0xFE 令牌循环每转一圈一次 sd_xfer，即一个字节时间。
 * 2026-10-03 实测标定（18MHz）：轮询每圈 1480ns，数据循环每字节 1188ns，
 * 其中纯线上时间都是 444ns，其余是 CPU 开销（轮询圈多出的一次 HAL_GetTick
 * 调用约 300ns）。精度远高于 HAL_GetTick 的 1ms。
 * 用途：把单块 SD 耗时拆成 线上字节 + 卡忙/卡读延迟 + CPU 开销 三段。
 */
uint32_t SD_TakeWaitReadyPolls(void);
uint32_t SD_TakeTokenPolls(void);

#endif /* __SD_SPI_H */
