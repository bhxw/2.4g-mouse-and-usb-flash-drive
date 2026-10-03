/**
 * @file sd_spi.c
 * @brief SD 卡 SPI 底层驱动（SPI1 默认映射 PA5/6/7，CS=PB12）
 *
 * 时序参考：Arduino SdFat (Sd2Card)：CMD0 -> CMD8 -> ACMD41 -> CMD58(OCR) 判定 SDHC；
 * 读写使用 0xFE/0xFC 起始令牌与忙等待。
 * 单字节（命令/令牌/CRC/应答）走裸寄存器 sd_xfer，512 字节数据段走 DMA
 * （DMA1_Channel2=SPI1_RX / Channel3=SPI1_TX，见 sd_dma_xfer）。
 */

#include "sd_spi.h"

#include "main.h"
#include "spi.h"       /* hspi1 */

extern SPI_HandleTypeDef hspi1;

/* ---------------- 引脚与速率 ---------------- */
#define SD_CS_PORT      GPIOB
#define SD_CS_PIN       GPIO_PIN_12

#define SPI1_PRESC_LOW  SPI_BAUDRATEPRESCALER_256   /* 初始化 <=400kHz(72/256=281k) */
/* /4=18MHz，是 SD 规格(25MHz)内的最大档位（/2=36MHz 超规）。
 * 2026-10-03 用 /4↔/8 对照实测：数据段每字节 1188ns 里只有 ~444ns 在线上，
 * 其余 ~766ns(55 周期) 是 sd_xfer 两次 SR 轮询的 CPU 开销 —— 即时钟再高也吃不动。
 * 解法是把 512 字节数据段交给 DMA（见下面的 sd_dma_xfer），而不是提 SCK。 */
#define SPI1_PRESC_HIGH SPI_BAUDRATEPRESCALER_4     /* 数据传输 18MHz */

/* ---------------- 错误码 ---------------- */
#define SD_ERR_NONE        0
#define SD_ERR_INIT         1   /* 通用初始化失败 */
#define SD_ERR_CMD0         2
#define SD_ERR_CMD8         3
#define SD_ERR_ACMD41       4
#define SD_ERR_CMD58        5
#define SD_ERR_READ         6
#define SD_ERR_WRITE        7
#define SD_ERR_TIMEOUT      8
#define SD_ERR_CMD25        9   /* 多块写 CMD25 无响应 */

/* 命令 */
#define SD_CMD0   0x00
#define SD_CMD8   0x08
#define SD_CMD9   0x09
#define SD_CMD17  0x11
#define SD_CMD24  0x18
#define SD_CMD25  0x19
#define SD_CMD55  0x37
#define SD_CMD58  0x3A
#define SD_ACMD41 0x29

#define SD_R1_IDLE   0x01
#define SD_R1_READY  0x00

/* 数据令牌与应答（参考历程/SD/utility/SdInfo.h） */
#define SD_TOKEN_WRITE_MULTI  0xFC  /* 多块写，每块数据前 */
#define SD_TOKEN_STOP_TRAN    0xFD  /* 多块写结束 */
#define SD_DATA_RES_MASK      0x1F
#define SD_DATA_RES_ACCEPTED  0x05

#define SD_WRITE_TIMEOUT_MS   600u  /* Sd2Card SD_WRITE_TIMEOUT */

static uint8_t s_hc = 0;    /* 1 = SDHC/SDXC，块地址模式 */
static uint8_t s_ready = 0; /* 初始化成功标志 */

/* CMD25 多块写会话状态 */
static uint8_t  s_multi_open  = 0;  /* 1 = CS 已拉低且 CMD25 已被卡接受 */
static uint32_t s_multi_next  = 0;  /* 会话期望的下一个块号（未做地址左移） */

/* 轮询圈数计数器（纯测量，不影响功能）。每圈 = SPI 上一个字节时间 = 444ns@18MHz */
static uint32_t s_poll_wait_ready = 0;  /* sd_wait_ready 圈数：卡写忙时间 */
static uint32_t s_poll_token      = 0;  /* 0xFE 令牌等待圈数：卡读延迟 */

static void sd_session_close(void);

/* ---------------- 底层原语 ---------------- */
static void sd_cs_high(void)
{
    HAL_GPIO_WritePin(SD_CS_PORT, SD_CS_PIN, GPIO_PIN_SET);
}

static void sd_cs_low(void)
{
    /* 任何新事务都要先终结遗留的 CMD25 会话，否则卡会一直停在多块写状态 */
    sd_session_close();
    HAL_GPIO_WritePin(SD_CS_PORT, SD_CS_PIN, GPIO_PIN_RESET);
}

/* 直接寄存器操作：全双工 SPI 单字节传输（发送 tx，同时接收）
 * 避免 HAL 每次调用的状态机开销（~5-10μs），保持 SPI 全双工模式不变。
 * 2026-10-03 实测标定：每字节 1188ns，其中线上仅 444ns，其余 766ns(55 周期)
 * 是这里两次 SR 轮询的 CPU 开销 —— 512 字节数据段因此改走上面的 DMA。
 * 本函数只留给命令/令牌/CRC/应答这些单字节。 */
static uint8_t sd_xfer(uint8_t b)
{
    while (!(SPI1->SR & SPI_SR_TXE)) {}
    *(__IO uint8_t *)&SPI1->DR = b;
    while (!(SPI1->SR & SPI_SR_RXNE)) {}
    return *(__IO uint8_t *)&SPI1->DR;
}

/* ---------------- 512 字节数据段 DMA ----------------
 * 只有数据段走 DMA，命令/令牌/CRC/应答这些单字节仍走 sd_xfer。
 *
 * 为什么不用 HAL_SPI_TransmitReceive_DMA：HAL 会接管 hspi->State 并假定自己拥有
 * 整次传输，而本文件其余部分全是裸寄存器操作（见 :89 的说明），混用会污染状态机；
 * 更硬的限制是 HAL 要求收发两个缓冲区各 len 字节 —— 本项目 RAM 只剩 224B
 * （map: RW_IRAM1 0x4f20/0x5000），拿不出 512B 的 0xFF 源和 512B 的丢弃槽。
 * 直接编程 DMA1_Channel2(SPI1_RX)/Channel3(SPI1_TX)，用 MINC 开/关分别实现
 * "变址搬进 buf"和"收发同一字节"，零额外 RAM。
 *
 * 通道无冲突：SD 用 SPI1(PA5/6/7)，2.4G 无线用 SPI2(PB13/14/15, NRF24L01.c:28)，
 * DMA1_Channel2/3 只被 SPI1 占用（spi.c:127/:143）。
 */
#define SD_DMA_RX_CH      DMA1_Channel2   /* spi.c:127 hdma_spi1_rx */
#define SD_DMA_TX_CH      DMA1_Channel3   /* spi.c:143 hdma_spi1_tx */
#define SD_DMA_TIMEOUT_MS 100u            /* 512B@18MHz 线上只要 227us，纯防卡死 */

/* CCR 基值：PSIZE/MSIZE 都是 00(byte，对齐 spi.c:131-132 的 PDATAALIGN_BYTE/
 * MDATAALIGN_BYTE)，PL=01(medium，对齐 spi.c:134)，NORMAL(不置 CIRC)，
 * DIR=1 表示内存->外设。EN 与 MINC 单独按次置。 */
#define SD_DMA_CCR_RX     (DMA_CCR_PL_0)
#define SD_DMA_CCR_TX     (DMA_CCR_DIR | DMA_CCR_PL_0)

static uint8_t       s_dma_sink;         /* 写数据段的接收丢弃槽（1 字节，MINC 关） */
static const uint8_t s_dma_dummy = 0xFF; /* 读数据段的发送源（1 字节常量，MINC 关） */

/* IFCR 是只写寄存器，所以用 = 而不是 |=；写 CGIFx 会连带清掉该通道的
 * TCIF/HTIF/TEIF（RM0008 14.3.2）。 */
static void sd_dma_stop(void)
{
    SD_DMA_TX_CH->CCR &= ~DMA_CCR_EN;
    SD_DMA_RX_CH->CCR &= ~DMA_CCR_EN;
    SPI1->CR2 &= ~(SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN);
    DMA1->IFCR = DMA_IFCR_CGIF2 | DMA_IFCR_CGIF3;
}

/* 一次 len 字节的全双工 DMA。tx/rx 都允许是单字节（靠 tx_inc/rx_inc 关掉 MINC）。
 * 返回 SD_ERR_NONE / SD_ERR_TIMEOUT。 */
static uint8_t sd_dma_xfer(const uint8_t *tx, uint8_t *rx,
                           uint32_t tx_inc, uint32_t rx_inc, uint16_t len)
{
    uint32_t t0;
    const uint32_t done = DMA_ISR_TCIF2 | DMA_ISR_TCIF3;

    sd_dma_stop();

    /* DR 后 SR：这个顺序既排空上次残留的接收字节（否则本次第一个 DMA 请求会搬旧数据），
     * 也是清 OVR 的唯一途径，与 __HAL_SPI_CLEAR_OVRFLAG 一致
     * （stm32f1xx_hal_spi.h:426-430）。 */
    (void)SPI1->DR;
    (void)SPI1->SR;

    SD_DMA_RX_CH->CPAR  = (uint32_t)&SPI1->DR;
    SD_DMA_RX_CH->CMAR  = (uint32_t)rx;
    SD_DMA_RX_CH->CNDTR = len;
    SD_DMA_RX_CH->CCR   = SD_DMA_CCR_RX | (rx_inc ? DMA_CCR_MINC : 0u);

    SD_DMA_TX_CH->CPAR  = (uint32_t)&SPI1->DR;
    SD_DMA_TX_CH->CMAR  = (uint32_t)tx;
    SD_DMA_TX_CH->CNDTR = len;
    SD_DMA_TX_CH->CCR   = SD_DMA_CCR_TX | (tx_inc ? DMA_CCR_MINC : 0u);

    SPI1->CR2 |= SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN;

    /* RX 必须先开：TX 一使能立刻产生时钟，RXNE 若无通道接住会置 OVR 并丢字节 */
    SD_DMA_RX_CH->CCR |= DMA_CCR_EN;
    SD_DMA_TX_CH->CCR |= DMA_CCR_EN;

    t0 = HAL_GetTick();
    while (((DMA1->ISR & done) != done) && ((HAL_GetTick() - t0) < SD_DMA_TIMEOUT_MS))
    {
    }
    if ((DMA1->ISR & done) != done)
    {
        sd_dma_stop();
        return SD_ERR_TIMEOUT;
    }

    /* TX 的 TCIF 只说明 512 字节都进了 DR，最后一字节还在移位寄存器里；
     * 不等 BSY 落下就返回，紧接着的 sd_xfer(CRC) 会和它撞车。 */
    t0 = HAL_GetTick();
    while ((SPI1->SR & SPI_SR_BSY) && ((HAL_GetTick() - t0) < SD_DMA_TIMEOUT_MS))
    {
    }

    sd_dma_stop();
    return SD_ERR_NONE;
}

/* 直接改写 SPI1 波特率预分频（不经过 HAL 状态机，避免 DeInit/Init 抖动） */
static void sd_spi_speed(uint32_t prescaler)
{
    __HAL_SPI_DISABLE(&hspi1);
    MODIFY_REG(SPI1->CR1, SPI_CR1_BR, (prescaler & SPI_CR1_BR));
    __HAL_SPI_ENABLE(&hspi1);
}

/* 发若干 0xFF 时钟 */
static void sd_dummy_clocks(uint32_t n)
{
    while (n--)
    {
        (void)sd_xfer(0xFF);
    }
}

/* 发送命令并等待 R1（跳过前导 0xFF）。返回 R1 或 0xFF（无响应） */
static uint8_t sd_cmd(uint8_t cmd, uint32_t arg, uint8_t crc)
{
    uint8_t i, r;

    /* 前导时钟 */
    (void)sd_xfer(0xFF);

    (void)sd_xfer(cmd | 0x40);
    (void)sd_xfer((uint8_t)(arg >> 24));
    (void)sd_xfer((uint8_t)(arg >> 16));
    (void)sd_xfer((uint8_t)(arg >> 8));
    (void)sd_xfer((uint8_t)arg);
    (void)sd_xfer(crc);

    for (i = 0; i < 16; i++)
    {
        r = sd_xfer(0xFF);
        if ((r & 0x80) == 0)
        {
            return r;
        }
    }
    return 0xFF;
}

/* CMD55 + 特定应用命令 */
static uint8_t sd_acmd(uint8_t cmd, uint32_t arg)
{
    if (sd_cmd(SD_CMD55, 0, 0x01) > 1)
    {
        return 0xFF;
    }
    return sd_cmd(cmd, arg, 0x01);
}

/* 等待卡忙结束（返回 0 表示不忙 / 超时返回 1）
 * s_poll_wait_ready：每转一圈一次 sd_xfer，即一个字节时间。
 * 2026-10-03 实测标定（18MHz）：每圈 1480ns = 线上 444ns + CPU 1036ns，
 * CPU 那部分含本循环每次迭代的 HAL_GetTick() 调用；数据循环每字节 1188ns。
 * 两参数模型在 6 个窗口上盲测误差 <1%。 */
static uint8_t sd_wait_ready(uint32_t timeout_ms)
{
    uint32_t t0 = HAL_GetTick();
    uint8_t r = 0xFF;
    do
    {
        s_poll_wait_ready++;
        r = sd_xfer(0xFF);
        if (r == 0xFF)
        {
            return 0;
        }
    } while ((HAL_GetTick() - t0) < timeout_ms);
    return 1;
}

/* 结束 CMD25 多块写会话：STOP_TRAN + 忙等待 + 释放 CS
 * 对应 Sd2Card::writeStop（参考历程/SD/utility/Sd2Card.cpp:633-644）。
 * 未开启会话时为空操作，可被 sd_cs_low() 无条件调用。 */
static void sd_session_close(void)
{
    if (!s_multi_open)
    {
        return;
    }
    s_multi_open = 0;

    (void)sd_wait_ready(SD_WRITE_TIMEOUT_MS);
    (void)sd_xfer(SD_TOKEN_STOP_TRAN);
    (void)sd_wait_ready(SD_WRITE_TIMEOUT_MS);
    sd_cs_high();
}

/* ---------------- 对外接口 ---------------- */
uint8_t SD_Init(void)
{
    GPIO_InitTypeDef gpio = {0};
    uint32_t t0;
    uint8_t r, ocr;
    uint16_t retry;

    s_hc = 0;
    s_ready = 0;

    /* CS 引脚（PB12）推挽输出，默认高 */
    __HAL_RCC_GPIOB_CLK_ENABLE();
    gpio.Pin = SD_CS_PIN;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(SD_CS_PORT, &gpio);
    sd_cs_high();

    /* 先低速 */
    sd_spi_speed(SPI1_PRESC_LOW);

    /* 上电至少 74 个时钟（CS 高） */
    sd_dummy_clocks(16);

    t0 = HAL_GetTick();
    sd_cs_low();
    retry = 0;
    do
    {
        r = sd_cmd(SD_CMD0, 0, 0x95);      /* CMD0：进入 SPI 模式 */
        if (r == SD_R1_IDLE)
        {
            break;
        }
        if ((HAL_GetTick() - t0) > 1000)
        {
            sd_cs_high();
            return SD_ERR_CMD0;
        }
        sd_dummy_clocks(8);
    } while (++retry < 8);
    if (r != SD_R1_IDLE)
    {
        sd_cs_high();
        return SD_ERR_CMD0;
    }

    /* CMD8：识别 SD v2 */
    r = sd_cmd(SD_CMD8, 0x000001AA, 0x87);
    if (r == SD_R1_IDLE)
    {
        /* 读 R7 4 字节，仅校验低字节 0xAA */
        for (uint8_t i = 0; i < 3; i++)
        {
            (void)sd_xfer(0xFF);
        }
        if (sd_xfer(0xFF) != 0xAA)
        {
            sd_cs_high();
            return SD_ERR_CMD8;
        }
        /* SD v2：ACMD41 带 HCS=1 */
        t0 = HAL_GetTick();
        do
        {
            r = sd_acmd(SD_ACMD41, 0x40000000);
            if (r == SD_R1_READY)
            {
                break;
            }
        } while ((HAL_GetTick() - t0) < 1000);
        if (r != SD_R1_READY)
        {
            sd_cs_high();
            return SD_ERR_ACMD41;
        }
        /* CMD58 读 OCR：CCS 位判断 SDHC */
        if (sd_cmd(SD_CMD58, 0, 0x01) != SD_R1_READY)
        {
            sd_cs_high();
            return SD_ERR_CMD58;
        }
        ocr = sd_xfer(0xFF);
        if ((ocr & 0xC0) == 0xC0)
        {
            s_hc = 1;
        }
        for (uint8_t i = 0; i < 3; i++)
        {
            (void)sd_xfer(0xFF);
        }
    }
    else if (r == 0x05)
    {
        /* SD v1 / MMC：不支持 CMD8，走普通 ACMD41 */
        t0 = HAL_GetTick();
        do
        {
            r = sd_acmd(SD_ACMD41, 0);
            if (r == SD_R1_READY)
            {
                break;
            }
        } while ((HAL_GetTick() - t0) < 1000);
        if (r != SD_R1_READY)
        {
            sd_cs_high();
            return SD_ERR_ACMD41;
        }
        s_hc = 0;
    }
    else
    {
        sd_cs_high();
        return SD_ERR_CMD8;
    }

    sd_cs_high();

    /* 切换到高速 */
    sd_spi_speed(SPI1_PRESC_HIGH);
    s_ready = 1;
    return SD_ERR_NONE;
}

uint8_t SD_Ready(void)
{
    return s_ready;
}

/* 取走并清零轮询圈数计数器。
 * 计数与取值可能分属不同任务（sd_log 也会走 sd_wait_ready），
 * 读-清序列非原子，最坏丢几个计数，对测量结论无影响。 */
uint32_t SD_TakeWaitReadyPolls(void)
{
    uint32_t n = s_poll_wait_ready;
    s_poll_wait_ready = 0;
    return n;
}

uint32_t SD_TakeTokenPolls(void)
{
    uint32_t n = s_poll_token;
    s_poll_token = 0;
    return n;
}

uint8_t SD_ReadBlock(uint32_t block, uint8_t *buf)
{
    uint32_t t0;
    uint8_t r, b;

    if (!s_hc)
    {
        block <<= 9;
    }

    sd_cs_low();
    r = sd_cmd(SD_CMD17, block, 0x01);
    if (r != SD_R1_READY)
    {
        sd_cs_high();
        return SD_ERR_READ;
    }

    /* 等待起始令牌 0xFE（圈数 = 卡的 NAND 读延迟，见 s_poll_token） */
    t0 = HAL_GetTick();
    do
    {
        s_poll_token++;
        b = sd_xfer(0xFF);
        if (b == 0xFE)
        {
            break;
        }
    } while ((HAL_GetTick() - t0) < 200);
    if (b != 0xFE)
    {
        sd_cs_high();
        return SD_ERR_TIMEOUT;
    }

    /* 512 字节数据段走 DMA：发常量 0xFF 当时钟（TX MINC 关），收进 buf（RX MINC 开） */
    if (sd_dma_xfer(&s_dma_dummy, buf, 0u, 1u, (uint16_t)SD_BLOCK_SIZE) != SD_ERR_NONE)
    {
        sd_cs_high();
        return SD_ERR_TIMEOUT;
    }
    (void)sd_xfer(0xFF);  /* CRC 高字节 */
    (void)sd_xfer(0xFF);  /* CRC 低字节 */
    sd_cs_high();
    return SD_ERR_NONE;
}

uint8_t SD_WriteBlock(uint32_t block, const uint8_t *buf)
{
    uint8_t r, b;

    if (!s_hc)
    {
        block <<= 9;
    }

    sd_cs_low();
    r = sd_cmd(SD_CMD24, block, 0x01);
    if (r != SD_R1_READY)
    {
        sd_cs_high();
        return SD_ERR_WRITE;
    }

    (void)sd_xfer(0xFE);  /* 起始令牌 */
    if (sd_dma_xfer(buf, &s_dma_sink, 1u, 0u, (uint16_t)SD_BLOCK_SIZE) != SD_ERR_NONE)
    {
        sd_cs_high();
        return SD_ERR_TIMEOUT;
    }
    (void)sd_xfer(0xFF);  /* CRC 高字节(忽略) */
    (void)sd_xfer(0xFF);  /* CRC 低字节(忽略) */

    /* 数据应答：低 5 位应为 0x05 */
    b = sd_xfer(0xFF);
    if ((b & 0x1F) != 0x05)
    {
        sd_cs_high();
        return SD_ERR_WRITE;
    }

    /* 等待编程完成（忙低电平结束） */
    if (sd_wait_ready(500))
    {
        sd_cs_high();
        return SD_ERR_TIMEOUT;
    }
    sd_cs_high();
    return SD_ERR_NONE;
}

/* ---------------- CMD25 流式多块写 ----------------
 * 会话时序对应 Sd2Card::writeStart / writeData / writeStop
 * （参考历程/SD/utility/Sd2Card.cpp:602-644 / :545-589）。
 *
 * 与 Sd2Card 的一处刻意差异：忙等待放在每块**之后**而非下一块之前。
 * Sd2Card 把 waitNotBusy 放到 writeData 入口，最后一块的忙等待由 writeStop 兜住；
 * 本项目 sd_storage_write 返回后 SCSI 层立刻回 CSW(PASSED)
 * （Middlewares/.../MSC/Src/usbd_msc_scsi.c:97-100），
 * 所以必须保证每块返回时数据都已被卡接受且不再忙。
 *
 * 不发 ACMD23 预擦除：MSC 层不知道一次 WRITE10 之后还有多少块，块数无法预告。
 */
uint8_t SD_WriteBegin(uint32_t block)
{
    uint8_t r;

    /* sd_cs_low() 内部会先终结遗留会话 */
    sd_cs_low();

    r = sd_cmd(SD_CMD25, s_hc ? block : (block << 9), 0x01);
    if (r != SD_R1_READY)
    {
        sd_cs_high();
        return SD_ERR_CMD25;
    }

    s_multi_open = 1;
    s_multi_next = block;
    return SD_ERR_NONE;
}

uint8_t SD_WriteChunk(uint32_t block, const uint8_t *buf)
{
    uint8_t b;

    if (!s_multi_open || block != s_multi_next)
    {
        return SD_ERR_WRITE;
    }

    (void)sd_xfer(SD_TOKEN_WRITE_MULTI);
    /* 512 字节数据段走 DMA：从 buf 变址搬出（TX MINC 开），收进 1 字节丢弃槽（RX MINC 关） */
    if (sd_dma_xfer(buf, &s_dma_sink, 1u, 0u, (uint16_t)SD_BLOCK_SIZE) != SD_ERR_NONE)
    {
        sd_session_close();
        return SD_ERR_TIMEOUT;
    }
    (void)sd_xfer(0xFF);  /* CRC 高字节(SPI 模式默认关闭 CRC) */
    (void)sd_xfer(0xFF);  /* CRC 低字节 */

    b = sd_xfer(0xFF);
    if ((b & SD_DATA_RES_MASK) != SD_DATA_RES_ACCEPTED)
    {
        sd_session_close();
        return SD_ERR_WRITE;
    }

    if (sd_wait_ready(SD_WRITE_TIMEOUT_MS))
    {
        sd_session_close();
        return SD_ERR_TIMEOUT;
    }

    s_multi_next = block + 1;
    return SD_ERR_NONE;
}

uint8_t SD_WriteEnd(void)
{
    sd_session_close();
    return SD_ERR_NONE;
}

/* 读 CSD 寄存器（CMD9） */
static uint8_t sd_read_csd(uint8_t csd[16])
{
    uint8_t b, r;

    sd_cs_low();
    r = sd_cmd(SD_CMD9, 0, 0x01);
    if (r != SD_R1_READY)
    {
        sd_cs_high();
        return SD_ERR_READ;
    }
    b = 0;
    for (uint32_t t0 = HAL_GetTick(); ; )
    {
        b = sd_xfer(0xFF);
        if (b == 0xFE)
        {
            break;
        }
        if ((HAL_GetTick() - t0) > 200)
        {
            sd_cs_high();
            return SD_ERR_TIMEOUT;
        }
    }
    for (uint8_t i = 0; i < 16; i++)
    {
        csd[i] = sd_xfer(0xFF);
    }
    (void)sd_xfer(0xFF);   /* CRC */
    (void)sd_xfer(0xFF);
    sd_cs_high();
    return SD_ERR_NONE;
}

uint8_t SD_GetBlockCount(uint32_t *blocks)
{
    uint8_t csd[16];
    uint8_t ver;

    if (blocks == NULL)
    {
        return SD_ERR_READ;
    }
    if (sd_read_csd(csd) != SD_ERR_NONE)
    {
        return SD_ERR_READ;
    }

    ver = (uint8_t)(csd[0] >> 6);   /* CSD_STRUCTURE */
    if (ver == 0)
    {
        /* SD v1：字节寻址 */
        uint8_t read_bl_len = (uint8_t)(csd[5] & 0x0F);
        uint16_t c_size = (uint16_t)(((csd[6] & 0x03) << 10) | (csd[7] << 2) | ((csd[8] & 0xC0) >> 6));
        uint8_t c_size_mult = (uint8_t)(((csd[9] & 0x03) << 1) | ((csd[10] & 0x80) >> 7));
        *blocks = ((uint32_t)c_size + 1U) << (c_size_mult + read_bl_len - 7U);
    }
    else if (ver == 1)
    {
        /* SDHC/SDXC：块地址 */
        uint32_t c_size = ((uint32_t)(csd[7] & 0x3F) << 16) | ((uint32_t)csd[8] << 8) | csd[9];
        *blocks = (c_size + 1U) << 10;
    }
    else
    {
        return SD_ERR_READ;
    }
    return SD_ERR_NONE;
}
