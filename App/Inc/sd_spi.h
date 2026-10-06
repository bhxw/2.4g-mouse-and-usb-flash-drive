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
 * @brief CMD18 流式多块读：Begin 打开会话 -> 连续 Chunk -> End(CMD12) 关闭
 *
 * 约束与下面的 CMD25 写会话相同：会话期间 CS 保持低电平，**不得调用任何其它
 * SD 接口**；违规调用时其它接口的 sd_cs_low() 会自动发 CMD12 终结会话。
 * Chunk 只接受严格连续递增的块号，跳号返回非 0，需重新 Begin。
 * 卡只在收到时钟时才吐下一块，所以会话可以跨多次调用长期挂着而不丢数据。
 */
uint8_t SD_ReadBegin(uint32_t block);
uint8_t SD_ReadChunk(uint32_t block, uint8_t *buf);
uint8_t SD_ReadEnd(void);

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

/** 会话重开统计（纯测量用，usb_storage.c 的 [SD-RD]/[SD-WR] 行取用） */
typedef struct
{
    uint32_t n;         /* 本窗口内 Begin 调用次数（= 会话重开次数） */
    uint32_t close_cyc; /* 终结上一会话累计周期数：STOP_TRAN/CMD12 + 卡 busy */
    uint32_t open_cyc;  /* 新会话命令帧 + R1 等待累计周期数 */
} sd_sess_stat_t;

/**
 * @brief 取走并清零会话重开统计（is_read=0 取写会话，非 0 取读会话）
 *
 * 会话重开本来混在 usb_storage.c 的 sd 段均值里（那颗 t0 取在 Chunk 之前，Begin 落在里面），
 * 拆开是为了把两笔成本分开量：close = 上一笔的尾巴（含卡内部编程未结束时 sd_wait_ready 的
 * busy），open = 卡对 CMD25/CMD18 的响应。两笔都是设备侧成本，与"主机下一笔命令的节奏"无关。
 * close 只在 sd_session_close() 真有会话可终结时累计，n 数的是 Begin 次数，两者不一定相等。
 */
void SD_TakeSessionStats(uint8_t is_read, sd_sess_stat_t *st);

/**
 * @brief SD_Init 失败点诊断（成功时 step=0）
 *
 * step 语义：2=CMD0 失败，3=CMD8 的 R7 校验字节不是 0xAA，4=ACMD41 失败（SD v2 分支），
 * 5=CMD58 失败，6=ACMD41 失败（CMD8 回 0x05 的 v1/MMC 分支）。r1=0xFF = 卡在该步无应答。
 * 用途：某张卡上 [LOG] mount fail fr=3 一直刷时，一眼看出卡停在哪一步、回了什么，
 * 从而区分"卡不应答 / 卡回了错误码 / 文件系统层问题"。
 */
typedef struct
{
    uint32_t n;    /* 开机以来 SD_Init 调用次数 */
    uint8_t  step; /* 最近一次失败停在哪一步，0 = 成功 */
    uint8_t  r1;   /* 该步最后拿到的 R1 */
    uint8_t  v;    /* CMD8 的 R7 校验字节；0xEE = 应答不合规、跳过了 R7 校验；
                      0x05 = CMD8 报非法命令（v1/MMC 分支） */
    uint8_t  ocr;  /* CMD58 读回的 OCR 首字节（bit7 上电完成、bit6 CCS） */
    uint8_t  c59;  /* 初始化末尾 CMD59(CRC off) 的 R1，0xFF = 无应答 */
} sd_init_diag_t;

void SD_GetInitDiag(sd_init_diag_t *d);

/** 打印一行 [SDI]（仅本地调试用） */
void SD_LogInitDiag(void);

#endif /* __SD_SPI_H */
