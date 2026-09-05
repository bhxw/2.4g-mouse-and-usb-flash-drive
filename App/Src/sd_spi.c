/**
 * @file sd_spi.c
 * @brief SD 卡 SPI 底层驱动（SPI1 默认映射 PA5/6/7，CS=PB12）
 *
 * 时序参考：Arduino SdFat (Sd2Card)：CMD0 -> CMD8 -> ACMD41 -> CMD58(OCR) 判定 SDHC；
 * 读写使用 0xFE 起始令牌与忙等待。首版为阻塞(HAL_SPI_TransmitReceive)实现。
 */

#include "sd_spi.h"

#include "main.h"
#include "spi.h"       /* hspi1 */

extern SPI_HandleTypeDef hspi1;

/* ---------------- 引脚与速率 ---------------- */
#define SD_CS_PORT      GPIOB
#define SD_CS_PIN       GPIO_PIN_12

#define SPI1_PRESC_LOW  SPI_BAUDRATEPRESCALER_256   /* 初始化 <=400kHz(72/256=281k) */
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

/* 命令 */
#define SD_CMD0   0x00
#define SD_CMD8   0x08
#define SD_CMD9   0x09
#define SD_CMD17  0x11
#define SD_CMD24  0x18
#define SD_CMD55  0x37
#define SD_CMD58  0x3A
#define SD_ACMD41 0x29

#define SD_R1_IDLE   0x01
#define SD_R1_READY  0x00

static uint8_t s_hc = 0;    /* 1 = SDHC/SDXC，块地址模式 */

/* ---------------- 底层原语 ---------------- */
static void sd_cs_high(void)
{
    HAL_GPIO_WritePin(SD_CS_PORT, SD_CS_PIN, GPIO_PIN_SET);
}

static void sd_cs_low(void)
{
    HAL_GPIO_WritePin(SD_CS_PORT, SD_CS_PIN, GPIO_PIN_RESET);
}

static uint8_t sd_xfer(uint8_t b)
{
    uint8_t rx = 0xFF;
    (void)HAL_SPI_TransmitReceive(&hspi1, &b, &rx, 1, 10);
    return rx;
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

/* 等待卡忙结束（返回 0 表示不忙 / 超时返回 1） */
static uint8_t sd_wait_ready(uint32_t timeout_ms)
{
    uint32_t t0 = HAL_GetTick();
    uint8_t r = 0xFF;
    do
    {
        r = sd_xfer(0xFF);
        if (r == 0xFF)
        {
            return 0;
        }
    } while ((HAL_GetTick() - t0) < timeout_ms);
    return 1;
}

/* ---------------- 对外接口 ---------------- */
uint8_t SD_Init(void)
{
    GPIO_InitTypeDef gpio = {0};
    uint32_t t0;
    uint8_t r, ocr;
    uint16_t retry;

    s_hc = 0;

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
    return SD_ERR_NONE;
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

    /* 等待起始令牌 0xFE */
    t0 = HAL_GetTick();
    do
    {
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

    for (uint16_t i = 0; i < SD_BLOCK_SIZE; i++)
    {
        buf[i] = sd_xfer(0xFF);
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
    for (uint16_t i = 0; i < SD_BLOCK_SIZE; i++)
    {
        (void)sd_xfer(buf[i]);
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
