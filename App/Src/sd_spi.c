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

#include "console.h"   /* dbg_printf：仅 SD_LogInitDiag 用 */
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
#define SD_ERR_CMD18       10   /* 多块读 CMD18 无响应 */

/* 命令 */
#define SD_CMD0   0x00
#define SD_CMD8   0x08
#define SD_CMD9   0x09
#define SD_CMD12  0x0C
#define SD_CMD17  0x11
#define SD_CMD18  0x12
#define SD_CMD24  0x18
#define SD_CMD25  0x19
#define SD_CMD55  0x37
#define SD_CMD58  0x3A
#define SD_CMD59  0x3B
#define SD_ACMD41 0x29

#define SD_R1_IDLE   0x01
#define SD_R1_READY  0x00

/* 数据令牌与应答（参考历程/SD/utility/SdInfo.h） */
#define SD_TOKEN_WRITE_MULTI  0xFC  /* 多块写，每块数据前 */
#define SD_TOKEN_STOP_TRAN    0xFD  /* 多块写结束 */
#define SD_DATA_RES_MASK      0x1F
#define SD_DATA_RES_ACCEPTED  0x05

#define SD_WRITE_TIMEOUT_MS   600u  /* Sd2Card SD_WRITE_TIMEOUT */
#define SD_READ_TIMEOUT_MS    200u  /* 等 0xFE 起始令牌 / CMD12 后忙结束 */

static uint8_t s_hc = 0;    /* 1 = SDHC/SDXC，块地址模式 */
static uint8_t s_ready = 0; /* 初始化成功标志 */

/* CMD25 多块写会话状态 */
static uint8_t  s_multi_open  = 0;  /* 1 = CS 已拉低且 CMD25 已被卡接受 */
static uint32_t s_multi_next  = 0;  /* 会话期望的下一个块号（未做地址左移） */

/* CMD18 多块读会话状态。与写会话天然互斥：任一种 Begin 都走 sd_cs_low()，
 * 而它会先终结遗留会话。 */
static uint8_t  s_rd_open  = 0;     /* 1 = CS 已拉低且 CMD18 已被卡接受 */
static uint32_t s_rd_next  = 0;     /* 会话期望的下一个块号（未做地址左移） */

/* 轮询圈数计数器（纯测量，不影响功能）。每圈 = SPI 上一个字节时间 = 444ns@18MHz */
static uint32_t s_poll_wait_ready = 0;  /* sd_wait_ready 圈数：卡写忙时间 */
static uint32_t s_poll_token      = 0;  /* 0xFE 令牌等待圈数：卡读延迟 */

/* 会话重开统计（纯测量，不影响功能）：close 在 sd_session_close() 里按"被终结的是读会话
 * 还是写会话"分别归账，open 在 SD_ReadBegin/SD_WriteBegin 里累计。 */
static sd_sess_stat_t s_sess_wr = {0, 0, 0};
static sd_sess_stat_t s_sess_rd = {0, 0, 0};

static void sd_session_close(void);

/* ---------------- 初始化诊断（仅本地调试用，不影响功能） ----------------
 * 换卡后 SD_Init 反复失败时用它定位到具体一步。step/r1/v 全是卡的真应答，不是推断。
 * 取用点两处：sd_log.c 的 [LOG] mount fail 与 usb_storage.c 的 [CAP] preinit fail。
 */
static sd_init_diag_t s_diag = {0};

void SD_GetInitDiag(sd_init_diag_t *d)
{
    if (d != NULL)
    {
        *d = s_diag;
    }
}

void SD_LogInitDiag(void)
{
    dbg_printf("[SDI] n=%lu step=%u r1=%02X v=%02X ocr=%02X c59=%02X\r\n",
               (unsigned long)s_diag.n, (unsigned)s_diag.step,
               (unsigned)s_diag.r1, (unsigned)s_diag.v, (unsigned)s_diag.ocr,
               (unsigned)s_diag.c59);
}

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

/* CRC7（SD 命令 CRC）：生成多项式 x^7+x^3+1，对"命令+4 字节参数"做按位长除取 7 位余数，
 * 线上发的字节 = (CRC7 << 1) | 1（末位固定 1）。
 *
 * 2026-10-06 实测结论：**这张 32G 卡对命令 CRC7 是严格检查的**，而原驱动只对 CMD0/CMD8
 * 用真值（0x95/0x87），其余命令一律发哑元 0x01。1G 卡（合规卡，SPI 模式不查 CRC）因此
 * 一路正常；32G 卡则在 CMD55 就回 R1=0x09（bit3=CRC 错误），ACMD41 根本发不出去，
 * 初始化必然失败。所以这里改为每条命令都现算真值 —— 对合规卡是无害的（它们不查）。
 * 校验：CMD0→0x95、CMD8→0x87，与 Sd2Card/FatFs 的固定表值一致。 */
static uint8_t sd_crc7(const uint8_t *d5)
{
    uint8_t reg = 0;

    for (uint8_t i = 0; i < 5; i++)
    {
        for (uint8_t b = 0x80u; b != 0u; b >>= 1)
        {
            reg = (uint8_t)(reg << 1);
            if ((d5[i] & b) != 0u)
            {
                reg |= 1u;
            }
            if ((reg & 0x80u) != 0u)
            {
                reg ^= 0x89u;
            }
        }
    }
    for (uint8_t i = 0; i < 7; i++)   /* 补 7 个 0 完成除法 */
    {
        reg = (uint8_t)(reg << 1);
        if ((reg & 0x80u) != 0u)
        {
            reg ^= 0x89u;
        }
    }
    return (uint8_t)(((reg & 0x7Fu) << 1) | 1u);
}

/* ---------------- 写数据段 CRC16（2026-10-08 换卡实测新增） ----------------
 * 实测结论（`[SDW] win=FF FF 0B`，`local/captures/sd8_test.log`）：**这张 8G 卡在 SPI 模式下
 * 开着写数据 CRC 校验** —— 数据段发哑元 0xFF 0xFF 时，卡收完 512B 后回的是数据应答令牌
 * 0x0B（`0bxxx0sss1`，sss=101 = CRC 错误），于是每一块写都被拒。合规卡在 SPI 模式下默认
 * 不校验（Sd2Card 与 FatFs 因此都发哑元），所以 1G 卡与既有历程从未暴露这一点；按位哑元
 * 之外还必须先补令牌前的 Nwr（见 sd_wait_write_nwr），否则卡连令牌都收不到、根本不回应答。
 *
 * 这里按规范现算真值：CRC-16/CCITT（x^16+x^12+x^5+1、初值 0、MSB 优先），查表实现。
 * 校验（Python 按位实现逐位对照：200 组随机 + 全 0 + 全 FF + 本项目图案，零不一致）：
 * "123456789" → 0x31C3、512B 全 0xFF → 0x7FA1、512B 全 0 → 0x0000。
 * 代价：表占 Flash 512B；512B 数据段约 2.5k 周期 ≈ **35µs/块（推算，72MHz）**，相比写块
 * 实测 247µs/块约 +14%（第二十条里"按位 228µs/块"是未查表的算法）。置 0 退回哑元作对照。 */
#define SD_WRITE_CRC16  1

static const uint16_t s_crc16_tab[256] = {
    0x0000u, 0x1021u, 0x2042u, 0x3063u, 0x4084u, 0x50A5u, 0x60C6u, 0x70E7u,
    0x8108u, 0x9129u, 0xA14Au, 0xB16Bu, 0xC18Cu, 0xD1ADu, 0xE1CEu, 0xF1EFu,
    0x1231u, 0x0210u, 0x3273u, 0x2252u, 0x52B5u, 0x4294u, 0x72F7u, 0x62D6u,
    0x9339u, 0x8318u, 0xB37Bu, 0xA35Au, 0xD3BDu, 0xC39Cu, 0xF3FFu, 0xE3DEu,
    0x2462u, 0x3443u, 0x0420u, 0x1401u, 0x64E6u, 0x74C7u, 0x44A4u, 0x5485u,
    0xA56Au, 0xB54Bu, 0x8528u, 0x9509u, 0xE5EEu, 0xF5CFu, 0xC5ACu, 0xD58Du,
    0x3653u, 0x2672u, 0x1611u, 0x0630u, 0x76D7u, 0x66F6u, 0x5695u, 0x46B4u,
    0xB75Bu, 0xA77Au, 0x9719u, 0x8738u, 0xF7DFu, 0xE7FEu, 0xD79Du, 0xC7BCu,
    0x48C4u, 0x58E5u, 0x6886u, 0x78A7u, 0x0840u, 0x1861u, 0x2802u, 0x3823u,
    0xC9CCu, 0xD9EDu, 0xE98Eu, 0xF9AFu, 0x8948u, 0x9969u, 0xA90Au, 0xB92Bu,
    0x5AF5u, 0x4AD4u, 0x7AB7u, 0x6A96u, 0x1A71u, 0x0A50u, 0x3A33u, 0x2A12u,
    0xDBFDu, 0xCBDCu, 0xFBBFu, 0xEB9Eu, 0x9B79u, 0x8B58u, 0xBB3Bu, 0xAB1Au,
    0x6CA6u, 0x7C87u, 0x4CE4u, 0x5CC5u, 0x2C22u, 0x3C03u, 0x0C60u, 0x1C41u,
    0xEDAEu, 0xFD8Fu, 0xCDECu, 0xDDCDu, 0xAD2Au, 0xBD0Bu, 0x8D68u, 0x9D49u,
    0x7E97u, 0x6EB6u, 0x5ED5u, 0x4EF4u, 0x3E13u, 0x2E32u, 0x1E51u, 0x0E70u,
    0xFF9Fu, 0xEFBEu, 0xDFDDu, 0xCFFCu, 0xBF1Bu, 0xAF3Au, 0x9F59u, 0x8F78u,
    0x9188u, 0x81A9u, 0xB1CAu, 0xA1EBu, 0xD10Cu, 0xC12Du, 0xF14Eu, 0xE16Fu,
    0x1080u, 0x00A1u, 0x30C2u, 0x20E3u, 0x5004u, 0x4025u, 0x7046u, 0x6067u,
    0x83B9u, 0x9398u, 0xA3FBu, 0xB3DAu, 0xC33Du, 0xD31Cu, 0xE37Fu, 0xF35Eu,
    0x02B1u, 0x1290u, 0x22F3u, 0x32D2u, 0x4235u, 0x5214u, 0x6277u, 0x7256u,
    0xB5EAu, 0xA5CBu, 0x95A8u, 0x8589u, 0xF56Eu, 0xE54Fu, 0xD52Cu, 0xC50Du,
    0x34E2u, 0x24C3u, 0x14A0u, 0x0481u, 0x7466u, 0x6447u, 0x5424u, 0x4405u,
    0xA7DBu, 0xB7FAu, 0x8799u, 0x97B8u, 0xE75Fu, 0xF77Eu, 0xC71Du, 0xD73Cu,
    0x26D3u, 0x36F2u, 0x0691u, 0x16B0u, 0x6657u, 0x7676u, 0x4615u, 0x5634u,
    0xD94Cu, 0xC96Du, 0xF90Eu, 0xE92Fu, 0x99C8u, 0x89E9u, 0xB98Au, 0xA9ABu,
    0x5844u, 0x4865u, 0x7806u, 0x6827u, 0x18C0u, 0x08E1u, 0x3882u, 0x28A3u,
    0xCB7Du, 0xDB5Cu, 0xEB3Fu, 0xFB1Eu, 0x8BF9u, 0x9BD8u, 0xABBBu, 0xBB9Au,
    0x4A75u, 0x5A54u, 0x6A37u, 0x7A16u, 0x0AF1u, 0x1AD0u, 0x2AB3u, 0x3A92u,
    0xFD2Eu, 0xED0Fu, 0xDD6Cu, 0xCD4Du, 0xBDAAu, 0xAD8Bu, 0x9DE8u, 0x8DC9u,
    0x7C26u, 0x6C07u, 0x5C64u, 0x4C45u, 0x3CA2u, 0x2C83u, 0x1CE0u, 0x0CC1u,
    0xEF1Fu, 0xFF3Eu, 0xCF5Du, 0xDF7Cu, 0xAF9Bu, 0xBFBAu, 0x8FD9u, 0x9FF8u,
    0x6E17u, 0x7E36u, 0x4E55u, 0x5E74u, 0x2E93u, 0x3EB2u, 0x0ED1u, 0x1EF0u,
};

static uint16_t sd_crc16(const uint8_t *buf, uint32_t len)
{
    uint16_t crc = 0;

    while (len-- != 0u)
    {
        crc = (uint16_t)((crc << 8) ^ s_crc16_tab[(uint8_t)((crc >> 8) ^ *buf)]);
        buf++;
    }
    return crc;
}

/* 发数据段 CRC16 的两个字节（高字节在前），并**把这两个槽位的回读字节一并交给应答窗口** ——
 * 卡的应答令牌可能就紧跟其后，这两个字节的回读值不能丢（见 sd_wait_data_res 的窗口说明）。 */
static void sd_send_data_crc(const uint8_t *buf, uint8_t *crc_hi, uint8_t *crc_lo)
{
#if SD_WRITE_CRC16
    uint16_t c = sd_crc16(buf, SD_BLOCK_SIZE);
#else
    uint16_t c = 0xFFFFu;   /* 哑元：SPI 模式默认不校验的合规卡也接受 */
#endif

    *crc_hi = sd_xfer((uint8_t)(c >> 8));
    *crc_lo = sd_xfer((uint8_t)c);
}

/* 发送命令并等待 R1（跳过前导 0xFF）。返回 R1 或 0xFF（无响应）。
 * CRC7 由 sd_crc7() 现算 —— 不能再用哑元，见其注释（有卡严格检查）。 */
static uint8_t sd_cmd(uint8_t cmd, uint32_t arg)
{
    uint8_t frame[5];
    uint8_t i, r;

    frame[0] = (uint8_t)(cmd | 0x40);
    frame[1] = (uint8_t)(arg >> 24);
    frame[2] = (uint8_t)(arg >> 16);
    frame[3] = (uint8_t)(arg >> 8);
    frame[4] = (uint8_t)arg;

    /* 前导时钟 */
    (void)sd_xfer(0xFF);

    for (i = 0; i < 5; i++)
    {
        (void)sd_xfer(frame[i]);
    }
    (void)sd_xfer(sd_crc7(frame));

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

/* CMD55 + 特定应用命令（只有 SD_Init 用）。
 * `r > 1` 只拦"应答带错误位"（例如 CRC 错误 0x09）；0x00/0x01 都放行给 ACMD。 */
static uint8_t sd_acmd(uint8_t cmd, uint32_t arg)
{
    uint8_t r = sd_cmd(SD_CMD55, 0);

    if (r > 1)
    {
        return 0xFF;
    }
    return sd_cmd(cmd, arg);
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

/* 结束多块会话：读会话发 CMD12，写会话发 STOP_TRAN，之后都等忙结束再释放 CS。
 * 写侧对应 Sd2Card::writeStop（参考历程/SD/utility/Sd2Card.cpp:633-644），
 * 读侧对应 FatFs disk_read 多扇区分支收尾的 send_cmd(CMD12)。
 * 未开启会话时为空操作，可被 sd_cs_low() 无条件调用。
 *
 * 整段耗时按方向归到 s_sess_rd/s_sess_wr 的 close_cyc。语义与原先的逐分支 return 等价，
 * 只是把"到底有没有活干"提前判掉，好让计时只覆盖真做了事的那几次。 */
static void sd_session_close(void)
{
    sd_sess_stat_t *st;
    uint32_t t0;

    if (!s_rd_open && !s_multi_open)
    {
        return;
    }
    st = s_rd_open ? &s_sess_rd : &s_sess_wr;
    t0 = DWT->CYCCNT;

    if (s_rd_open)
    {
        s_rd_open = 0;
        /* CMD12 的应答是 R1b：sd_cmd 拿到 R1 后卡可能仍拉着 DO 表示忙 */
        (void)sd_cmd(SD_CMD12, 0);
        (void)sd_wait_ready(SD_READ_TIMEOUT_MS);
        sd_cs_high();
    }
    else
    {
        s_multi_open = 0;
        (void)sd_wait_ready(SD_WRITE_TIMEOUT_MS);
        (void)sd_xfer(SD_TOKEN_STOP_TRAN);
        (void)sd_wait_ready(SD_WRITE_TIMEOUT_MS);
        sd_cs_high();
    }

    st->close_cyc += DWT->CYCCNT - t0;
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

    s_diag.n++;
    s_diag.step = 0;
    s_diag.r1 = 0xFF;
    s_diag.v = 0xFF;
    s_diag.ocr = 0xFF;
    s_diag.c59 = 0xFF;

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
        r = sd_cmd(SD_CMD0, 0);            /* CMD0：进入 SPI 模式 */
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
    s_diag.r1 = r;
    if (r != SD_R1_IDLE)
    {
        s_diag.step = 2;
        sd_cs_high();
        return SD_ERR_CMD0;
    }

    /* CMD8：识别 SD v2。
     * 判据是 `r != 0x05`：合规 v2 卡回 0x01（进下面的 R7 校验），v1/MMC 卡回 0x05。
     * 放宽的原因是实测某 32G 卡回的是 0x00，随后字节是 F0 00 01 AA —— 回显 0x1AA 是对的
     * （说明参数被正确解析），但 R1 的 idle 位与 R7 首字节都不合规。原判据（只有 0x01）
     * 会让这张卡永远停在 SD_ERR_CMD8；放宽只改"本该直接失败"的那种情况，合规卡行为不变。 */
    r = sd_cmd(SD_CMD8, 0x000001AA);
    s_diag.r1 = r;
    if (r != 0x05)
    {
        if (r == SD_R1_IDLE)
        {
            /* 读 R7 4 字节，仅校验低字节 0xAA */
            for (uint8_t i = 0; i < 3; i++)
            {
                (void)sd_xfer(0xFF);
            }
            s_diag.v = sd_xfer(0xFF);
            if (s_diag.v != 0xAA)
            {
                s_diag.step = 3;
                sd_cs_high();
                return SD_ERR_CMD8;
            }
        }
        else
        {
            /* CMD8 应答不合规：跳过 R7 校验，按 SD v2 继续（理由见上面判据的说明） */
            s_diag.v = 0xEE;   /* 标记：没读 R7 */
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
        s_diag.r1 = r;
        if (r != SD_R1_READY)
        {
            s_diag.step = 4;
            sd_cs_high();
            return SD_ERR_ACMD41;
        }
        /* CMD58 读 OCR：CCS 位判断 SDHC */
        r = sd_cmd(SD_CMD58, 0);
        s_diag.r1 = r;
        if (r != SD_R1_READY)
        {
            s_diag.step = 5;
            sd_cs_high();
            return SD_ERR_CMD58;
        }
        ocr = sd_xfer(0xFF);
        s_diag.ocr = ocr;
        if ((ocr & 0xC0) == 0xC0)
        {
            s_hc = 1;
        }
        for (uint8_t i = 0; i < 3; i++)
        {
            (void)sd_xfer(0xFF);
        }
    }
    else
    {
        /* SD v1 / MMC：CMD8 报非法命令，走不带 HCS 的普通 ACMD41 */
        s_diag.v = r;   /* 留下 CMD8 的 0x05，否则会被下面的 ACMD41 应答覆盖 */
        t0 = HAL_GetTick();
        do
        {
            r = sd_acmd(SD_ACMD41, 0);
            if (r == SD_R1_READY)
            {
                break;
            }
        } while ((HAL_GetTick() - t0) < 1000);
        s_diag.r1 = r;
        if (r != SD_R1_READY)
        {
            s_diag.step = 6;
            sd_cs_high();
            return SD_ERR_ACMD41;
        }
        s_hc = 0;
    }

    /* 关掉卡的 CRC 校验：CMD59 SET_CRC_ON_OFF 带 arg=0。
     * 那张 32G 卡默认校验命令 CRC（见 sd_crc7 注释），而数据段 CRC16 本驱动发的是
     * 0xFF 0xFF 哑元（512B 不软算 CRC16，否则写吞吐会掉一大截）。命令 CRC 已由
     * sd_crc7() 保证正确，这里再把卡的校验关掉，让它回到 SPI 模式合规卡的默认行为。
     * 失败不致命，只记进 s_diag.c59 供 [SDI] 行观察。
     *
     * ⚠ 暂时置 0（不发送）：1G 合规卡上出现"读正常、写不稳定"，而读写路径唯一的
     * 不对称点就是数据段 CRC16 —— 写发 0xFF 0xFF 哑元、读把卡发的真值直接丢掉。
     * 卡一旦开始校验写数据块的 CRC16，写必然被拒、读完全不受影响，正是这个形状。
     * 批量之前驱动也发哑元而 1G 卡写得好，所以能"打开"这个校验的只有本条 CMD59。
     * 置 0 后 [SDI] 行会打 c59=EE 作为标记；假设成立与否再定它的最终去留。 */
#define SD_SEND_CMD59   0
#if SD_SEND_CMD59
    s_diag.c59 = sd_cmd(SD_CMD59, 0);
#else
    s_diag.c59 = 0xEE;   /* 标记：本条未发送 */
#endif
    sd_cs_high();

    /* 切换到高速 */
    sd_spi_speed(SPI1_PRESC_HIGH);
    s_ready = 1;
    dbg_printf("[SDI] ok hc=%u c59=%02X\r\n",
               (unsigned)s_hc, (unsigned)s_diag.c59);
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

void SD_TakeSessionStats(uint8_t is_read, sd_sess_stat_t *st)
{
    sd_sess_stat_t *src = is_read ? &s_sess_rd : &s_sess_wr;

    if (st != NULL)
    {
        *st = *src;
    }
    src->n = 0u;
    src->close_cyc = 0u;
    src->open_cyc = 0u;
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
    r = sd_cmd(SD_CMD17, block);
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

/* ---------------- CMD18 流式多块读 ----------------
 * 时序对应 FatFs disk_read 的多扇区分支：CMD18 -> N×(0xFE + 512B + 2B CRC) -> CMD12。
 * 全程 CS 保持低电平，卡只在主机给时钟时才吐下一块，所以会话可以跨多次调用
 * 一直挂着；块号断开、任何其它 SD 事务、显式 ReadEnd 都会终结它。
 *
 * 相比逐块 CMD17 省掉的是每块一次命令帧（前导 + 6 字节 + 最多 16 字节等 R1）
 * 和一次 CS 抬/拉；数据段本身（令牌 + 512B + CRC）一个字节都不少。
 */
uint8_t SD_ReadBegin(uint32_t block)
{
    uint32_t t0;
    uint8_t r;

    /* sd_cs_low() 内部会先终结遗留会话（含 CMD25 写会话） */
    sd_cs_low();

    s_sess_rd.n++;
    t0 = DWT->CYCCNT;
    r = sd_cmd(SD_CMD18, s_hc ? block : (block << 9));
    s_sess_rd.open_cyc += DWT->CYCCNT - t0;
    if (r != SD_R1_READY)
    {
        sd_cs_high();
        return SD_ERR_CMD18;
    }

    s_rd_open = 1;
    s_rd_next = block;
    return SD_ERR_NONE;
}

uint8_t SD_ReadChunk(uint32_t block, uint8_t *buf)
{
    uint32_t t0;
    uint8_t b;

    if (!s_rd_open || block != s_rd_next)
    {
        return SD_ERR_READ;
    }

    /* 等起始令牌，判据与 SD_ReadBlock 完全一致（只等 0xFE，超时 200ms）。
     * 上一版在这里改成 `(b & 0x80) == 0` 提前退出，导致挂载阶段全量读失败，
     * 详见开发日志 2026-10-04 第六条；本次复现把 CMD18 作为唯一变量。
     * 圈数照常计入 s_poll_token（卡读延迟）。 */
    t0 = HAL_GetTick();
    do
    {
        s_poll_token++;
        b = sd_xfer(0xFF);
        if (b == 0xFE)
        {
            break;
        }
    } while ((HAL_GetTick() - t0) < SD_READ_TIMEOUT_MS);
    if (b != 0xFE)
    {
        sd_session_close();
        return SD_ERR_TIMEOUT;
    }

    /* 512 字节数据段走 DMA：发常量 0xFF 当时钟（TX MINC 关），收进 buf（RX MINC 开） */
    if (sd_dma_xfer(&s_dma_dummy, buf, 0u, 1u, (uint16_t)SD_BLOCK_SIZE) != SD_ERR_NONE)
    {
        sd_session_close();
        return SD_ERR_TIMEOUT;
    }
    (void)sd_xfer(0xFF);  /* CRC 高字节(SPI 模式默认关闭 CRC) */
    (void)sd_xfer(0xFF);  /* CRC 低字节 */

    s_rd_next = block + 1;
    return SD_ERR_NONE;
}

uint8_t SD_ReadEnd(void)
{
    sd_session_close();
    return SD_ERR_NONE;
}

/* ---------------- 写侧补丁：令牌前的 Nwr + 应答窗口（2026-10-06 换卡实测新增） ----------------
 * 两处都是"读得到、写不进"那条对称破口的补丁：
 *   - 读路径本就是"轮询到 0xFE 为止"（超时 200ms），写路径原本只读一个字节且要求恰为 0x05；
 *   - 写路径在 R1 之后立刻发令牌，而参考实现（Sd2Card::writeData 入口）先 waitNotBusy。
 * 详见各自函数注释。 */

static void wr_fail_report(uint32_t block, uint8_t res, uint8_t err);  /* 定义在本节之后 */

/* 数据令牌前的 Nwr 等待：SD 物理层要求令牌与卡对写命令的 R1 应答之间至少隔 1 个字节，
 * 且此刻卡不能仍处忙态（DO 低）。本驱动原先把忙等待只放在"每块之后"（见下面 CMD25 注释），
 * 于是每个会话的第一块都是令牌紧跟 R1 —— 宽容的卡照收（1G 卡实测一路正常），严格的卡会丢
 * 令牌：令牌被丢之后，卡把紧接着的 512 字节当作"等令牌"期间的数据，载荷里一旦出现
 * 0xFC/0xFD 就被误认成起始令牌，应答令牌与忙信号因此都落在数据段内部发出 —— 主机侧只看
 * 到"没有令牌 / 只有忙"，正是换卡实测的形态（[SDF] res=FF / datares=00）。
 * sd_wait_ready 每转一圈 = 一个字节时间，卡不忙时它至少消耗 1 字节 = Nwr 下限。 */
static uint8_t sd_wait_write_nwr(void)
{
    return sd_wait_ready(SD_WRITE_TIMEOUT_MS);
}

/* 数据应答令牌：bit0 恒为 1、bit4 恒为 0（0bxxx0sss1；0x05=接受、0x0B=CRC 错、0x0D=写错）。
 * 判据必须是 (b & 0x11) == 0x01 —— "低 5 位 == 0x05" 会把卡拉忙时读到的 0x00 直接判死，
 * "bit0 == 1" 又会把空闲电平 0xFF 误认成令牌。
 *
 * 窗口从**刚发出去的两个 CRC 槽位**开始：卡若被允许立即应答，令牌就落在那里（原实现把这两
 * 个字节直接丢弃，于是"令牌其实来了、但看不到"）。扫描上限 SD_RES_SCAN 字节 —— 紧邻数据段
 * 的 1~2 个槽位才是合规位置，扫得太远只是把"卡根本不给令牌"拖成每次失败的 600ms 阻塞。
 * 窗口内始终没找到就返回最后读到的字节（0x00=卡只报忙 / 0xFF=空闲），交 [SDW] 行判读。 */
#define SD_RES_WIN   8u
#define SD_RES_SCAN  32u

static uint8_t s_res_win[SD_RES_WIN];   /* 应答窗口前 8 字节，失败时打轨迹 */
static uint8_t s_res_n;

static uint8_t sd_wait_data_res(uint8_t crc_hi, uint8_t crc_lo)
{
    uint8_t b = crc_hi;
    uint8_t n = 0;

    s_res_n = 0;
    s_res_win[s_res_n++] = crc_hi;
    s_res_win[s_res_n++] = crc_lo;

    for (;;)
    {
        if ((b & 0x11u) == 0x01u)
        {
            return b;
        }
        if (n++ >= SD_RES_SCAN)
        {
            return b;
        }
        b = sd_xfer(0xFF);
        if (s_res_n < SD_RES_WIN)
        {
            s_res_win[s_res_n++] = b;
        }
    }
}

/* 应答窗口轨迹：把前 8 个字节按时间顺序打成一行，配合 [SDW] 判"卡是只报忙、还是没吐字节" */
static void sd_res_trace_print(void)
{
    static const char hx[] = "0123456789ABCDEF";
    char hex[(3u * SD_RES_WIN) + 1u];
    uint8_t k = 0;

    for (uint8_t i = 0; i < s_res_n; i++)
    {
        hex[k++] = hx[s_res_win[i] >> 4];
        hex[k++] = hx[s_res_win[i] & 0x0Fu];
        hex[k++] = ' ';
    }
    hex[k] = '\0';
    dbg_printf("[SDW] win=%s\r\n", hex);
}

uint8_t SD_WriteBlock(uint32_t block, const uint8_t *buf)
{
    uint8_t r, b, crc_lo;

    if (!s_hc)
    {
        block <<= 9;
    }

    sd_cs_low();
    r = sd_cmd(SD_CMD24, block);
    if (r != SD_R1_READY)
    {
        sd_cs_high();
        return SD_ERR_WRITE;
    }

    /* 令牌前的 Nwr：卡必须不忙且与 R1 至少隔 1 个字节（见 sd_wait_write_nwr） */
    if (sd_wait_write_nwr())
    {
        wr_fail_report(block, 0xFF, 3u);
        sd_cs_high();
        return SD_ERR_TIMEOUT;
    }

    (void)sd_xfer(0xFE);  /* 起始令牌 */
    if (sd_dma_xfer(buf, &s_dma_sink, 1u, 0u, (uint16_t)SD_BLOCK_SIZE) != SD_ERR_NONE)
    {
        sd_cs_high();
        return SD_ERR_TIMEOUT;
    }
    sd_send_data_crc(buf, &b, &crc_lo);   /* 真 CRC16 的两个字节；回读值进应答窗口 */
    b = sd_wait_data_res(b, crc_lo);

    if ((b & SD_DATA_RES_MASK) != SD_DATA_RES_ACCEPTED)
    {
        wr_fail_report(block, b, 1u);
        sd_res_trace_print();
        sd_cs_high();
        return SD_ERR_WRITE;
    }

    /* 等待编程完成（忙低电平结束） */
    if (sd_wait_ready(500))
    {
        wr_fail_report(block, b, 2u);
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
    uint32_t t0;
    uint8_t r;

    /* sd_cs_low() 内部会先终结遗留会话 */
    sd_cs_low();

    s_sess_wr.n++;
    t0 = DWT->CYCCNT;
    r = sd_cmd(SD_CMD25, s_hc ? block : (block << 9));
    s_sess_wr.open_cyc += DWT->CYCCNT - t0;
    if (r != SD_R1_READY)
    {
        sd_cs_high();
        return SD_ERR_CMD25;
    }

    s_multi_open = 1;
    s_multi_next = block;
    return SD_ERR_NONE;
}

/* ---------------- 写失败明细（诊断，只在失败时打印） ----------------
 * 上限 8 行：主机重试时失败会连着来，不能让串口把 115200 堵死。
 * res = 卡的数据应答字节：0x05=接受、0x0B=CRC 错（写数据段的 CRC16 由 sd_crc16() 现算，
 * 2026-10-08 实测本卡的 0x0B 就是它；再出现说明数据段本身或时序有问题）、
 * 0x0D=写错、0x00=卡只报忙（未吐令牌）、0xFF=空闲（未吐令牌）。
 * err：1 = 应答非接受；2 = 写后忙不落；3 = 令牌前卡一直忙（Nwr 等待超时）。
 * 用途：读正常、写不稳定时，这一行直接指出是"卡拒了数据块"还是"卡不应答"；
 * 紧邻的 [SDW] win= 行是应答窗口前 8 个字节的时序轨迹（含刚发的两个 CRC 槽位）。 */
static uint8_t s_wfail;

static void wr_fail_report(uint32_t block, uint8_t res, uint8_t err)
{
    if (s_wfail < 8u)
    {
        s_wfail++;
        dbg_printf("[SDW] lba=%lu res=%02X err=%u n=%u\r\n",
                   (unsigned long)block, (unsigned)res, (unsigned)err, (unsigned)s_wfail);
    }
    else if (s_wfail == 8u)
    {
        s_wfail++;
        dbg_printf("[SDW] more suppressed\r\n");
    }
}

uint8_t SD_WriteChunk(uint32_t block, const uint8_t *buf)
{
    uint8_t b, crc_hi, crc_lo;

    if (!s_multi_open || block != s_multi_next)
    {
        return SD_ERR_WRITE;
    }

    /* 令牌前的 Nwr：会话第一块此前完全没等过（见 sd_wait_write_nwr） */
    if (sd_wait_write_nwr())
    {
        wr_fail_report(block, 0xFF, 3u);
        sd_session_close();
        return SD_ERR_TIMEOUT;
    }

    (void)sd_xfer(SD_TOKEN_WRITE_MULTI);
    /* 512 字节数据段走 DMA：从 buf 变址搬出（TX MINC 开），收进 1 字节丢弃槽（RX MINC 关） */
    if (sd_dma_xfer(buf, &s_dma_sink, 1u, 0u, (uint16_t)SD_BLOCK_SIZE) != SD_ERR_NONE)
    {
        sd_session_close();
        return SD_ERR_TIMEOUT;
    }
    sd_send_data_crc(buf, &crc_hi, &crc_lo);   /* 真 CRC16 的两个字节；回读值进应答窗口 */

    b = sd_wait_data_res(crc_hi, crc_lo);
    if ((b & SD_DATA_RES_MASK) != SD_DATA_RES_ACCEPTED)
    {
        wr_fail_report(block, b, 1u);
        sd_res_trace_print();
        sd_session_close();
        return SD_ERR_WRITE;
    }

    if (sd_wait_ready(SD_WRITE_TIMEOUT_MS))
    {
        wr_fail_report(block, b, 2u);
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
    r = sd_cmd(SD_CMD9, 0);
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

/* ---------------- 设备侧写自检（诊断用，交付前摘掉） ----------------
 * 为什么需要它：主机侧写要穿过 WRITE10 → MSC → BOT 一整条链，任何一段出问题都只表现为
 * "写不进"，没法把 SD-SPI 写本身单独判定。这里在开机时直接调 SD-SPI 的写接口：
 *   ① 读卡**最后一块** → 记 CRC16 → **原样写回**（CMD24）→ 读回 → CRC 比对；
 *   ② 同一块再走 CMD25 会话（Begin/Chunk/End，MSC 路径用的原语）重复一遍。
 * 用 sd_crc16() 比对而不保存整块（省 512B RAM），只占一个 512B 工作缓冲。
 * 写失败时前面会有 `[SDW]` 行给出应答字节与 `win=` 轨迹，那才是判据。
 * 选最后一块因为它最不可能属于任何文件；内容与读到的完全相同，路径正常时非破坏性。 */
#define SD_SELFTEST_WRITE  1

static uint8_t s_chk_buf[SD_BLOCK_SIZE];

void SD_WriteSelfTest(void)
{
#if SD_SELFTEST_WRITE
    uint32_t blocks = 0;
    uint32_t blk;
    uint16_t pre, post;
    uint8_t r;

    if (SD_GetBlockCount(&blocks) != 0 || blocks == 0u)
    {
        dbg_printf("[CHKW] no block count\r\n");
        return;
    }
    blk = blocks - 1u;

    /* ① CMD24 单块写（FatFs 走的就是这条） */
    if (SD_ReadBlock(blk, s_chk_buf) != 0)
    {
        dbg_printf("[CHKW] c24 blk=%lu pre-read fail\r\n", (unsigned long)blk);
        return;
    }
    pre = sd_crc16(s_chk_buf, SD_BLOCK_SIZE);
    r = SD_WriteBlock(blk, s_chk_buf);
    dbg_printf("[CHKW] c24 blk=%lu crc=%04X ret=%u\r\n",
               (unsigned long)blk, (unsigned)pre, (unsigned)r);
    if (SD_ReadBlock(blk, s_chk_buf) != 0)
    {
        dbg_printf("[CHKW] c24 post-read fail\r\n");
    }
    else
    {
        post = sd_crc16(s_chk_buf, SD_BLOCK_SIZE);
        dbg_printf("[CHKW] c24 verify crc=%04X %s\r\n", (unsigned)post,
                   (post == pre) ? "same" : "MISMATCH");
    }

    /* ② CMD25 会话写（U 盘路径用的原语） */
    if (SD_ReadBlock(blk, s_chk_buf) != 0)
    {
        dbg_printf("[CHKW] c25 pre-read fail\r\n");
        return;
    }
    pre = sd_crc16(s_chk_buf, SD_BLOCK_SIZE);
    r = SD_WriteBegin(blk);
    if (r == 0u)
    {
        r = SD_WriteChunk(blk, s_chk_buf);
    }
    (void)SD_WriteEnd();
    dbg_printf("[CHKW] c25 blk=%lu crc=%04X ret=%u\r\n",
               (unsigned long)blk, (unsigned)pre, (unsigned)r);
    if (SD_ReadBlock(blk, s_chk_buf) != 0)
    {
        dbg_printf("[CHKW] c25 post-read fail\r\n");
    }
    else
    {
        post = sd_crc16(s_chk_buf, SD_BLOCK_SIZE);
        dbg_printf("[CHKW] c25 verify crc=%04X %s\r\n", (unsigned)post,
                   (post == pre) ? "same" : "MISMATCH");
    }
    /* ③ CMD18 流式读会话（MSC 读路径用的原语；此前自检只覆盖 CMD17/24/25）
     * 背景：2026-10-08 抓包实测主机读 LBA0 被"瞬时拒绝"（CBW→CSW 仅 173µs、规范失败 CSW、
     * 无 STALL），而自检里的 CMD17 单块读是好的 —— 两条路径只差在这条会话机制上。 */
    r = SD_ReadBegin(blk);
    if (r == 0u)
    {
        r = SD_ReadChunk(blk, s_chk_buf);
    }
    (void)SD_ReadEnd();
    dbg_printf("[CHKW] c18 blk=%lu ret=%u\r\n", (unsigned long)blk, (unsigned)r);
    if (r == 0u)
    {
        post = sd_crc16(s_chk_buf, SD_BLOCK_SIZE);
        dbg_printf("[CHKW] c18 verify crc=%04X %s\r\n", (unsigned)post,
                   (post == pre) ? "same" : "MISMATCH");
    }

    dbg_printf("[CHKW] done wfail=%u\r\n", (unsigned)s_wfail);
#endif
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
