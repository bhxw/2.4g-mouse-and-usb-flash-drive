/**
 * @file usb_storage.c
 * @brief MSC 介质层实现：LUN0 = SD 卡（扇区直通 sd_spi）。
 *        主机枚举/读写期间由上层保证 sd_log 暂停（单写者）。
 */
#include "usb_storage.h"

#include "sd_spi.h"
#include "console.h"
#include "main.h"       /* HAL_GetTick */
#include "rtos_api.h"
#include "usbd_msc_scsi.h"

#include <string.h>

/* 诊断：1=U盘不碰SD(固定容量/空读写)；0=真实SD */
#define SD_BYPASS_TEST  0

static volatile uint32_t s_last_active_ms = 0;
static uint32_t s_cap_blocks = 0;
static uint16_t s_cap_size = 0;
static uint8_t  s_cap_ok = 0;

/* ---------------- 传输耗时统计（每 256 块汇总一次） ----------------
 * 用 DWT->CYCCNT（72MHz，14ns 分辨率）而非 HAL_GetTick()：
 * 单块耗时在毫秒以下时 1ms 量化会给出 0/1，误差 ±100%，不可用。
 * 累加到 256 块再打印，一是摊薄量化误差，二是把 dbg_printf 自身的
 * ~2ms 阻塞开销（console.c:31 走 HAL_UART_Transmit + HAL_MAX_DELAY）
 * 从每块 1 次降到每 256 块 1 次。
 *
 * 每块时间拆成两段测：sd（SD_ReadBlock / SD_WriteChunk 内部）+ gap（两次调用之间）。
 * 校验式 sd_avg + gap_avg ≈ 墙钟/块数，闭合了预算才算成立。
 * SD 段再靠 sd_spi.c 的轮询圈数计数器细分为 线上字节 / 卡忙 / CPU 轮询开销。
 */
#define STAT_FLUSH_BLOCKS   256u

typedef struct
{
    uint32_t cnt;
    uint64_t sum;     /* 累计 SD 侧周期数 */
    uint32_t min;     /* 周期数 */
    uint32_t max;     /* 周期数 */
    uint64_t gap;     /* 累计块间间隔周期数（本次入口 - 上次出口） */
    uint32_t gap_n;   /* gap 样本数 */
} sd_stat_t;

static sd_stat_t s_rd_stat = {0, 0, 0xFFFFFFFFu, 0, 0, 0};
static sd_stat_t s_wr_stat = {0, 0, 0xFFFFFFFFu, 0, 0, 0};
static uint32_t  s_wr_sessions = 0;   /* 本窗口内 SD_WriteBegin 成功次数 */

/* 块间间隔探针：这段窗口里 EP2 已经重新挂好 PrepareReceive（usbd_msc_scsi.c:105），
 * 设备随时可收，NAK 期在 SD 那 620us 里面而不是 gap 里。所以 gap 是纯
 * USB 传输 + 主机 pacing + memcpy + 队列唤醒的时间 —— 也正是双缓冲重叠能吃掉的那部分。
 *
 * armed=0 表示还没有"上一次出口"，首个样本必须跳过：否则开机到首次写入之间的几百秒
 * 会污染平均，且 CYCCNT 是 32 位、59.6s 就回绕，那个减法结果毫无意义。 */
static uint32_t s_rd_last_exit;
static uint8_t  s_rd_gap_armed;
static uint32_t s_wr_last_exit;
static uint8_t  s_wr_gap_armed;

static void stat_dwt_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0u;
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;
}

static void stat_add(sd_stat_t *st, uint32_t cyc)
{
    st->cnt++;
    st->sum += cyc;
    if (cyc < st->min) st->min = cyc;
    if (cyc > st->max) st->max = cyc;
}

static void stat_add_gap(sd_stat_t *st, uint32_t cyc)
{
    st->gap += cyc;
    st->gap_n++;
}

/* 每 256 块仍然只打一行（不增加打印次数，免得和 sysmon 撞 UART 被 HAL_BUSY 静默丢掉）。
 * sd=avg/min/max；gap=每块平均；poll_tag/polls=本窗口内的轮询圈数总数
 * （打总数不打平均，免得整数除法把 <1 的值吃掉）；bps=blk/sess。
 *
 * sessions != 0 时才打印 bps：它是判断 CMD25 有没有真在流式工作的关键指标 ——
 * 接近 256 说明一个会话吃下了整个窗口；等于 1 说明每块都在重开会话，多块白做。
 * 整个字段消失则说明会话跨窗口一直开着（比 256 更好）。 */
static void stat_report(const char *tag, sd_stat_t *st, uint32_t sessions,
                        const char *poll_tag, uint32_t polls)
{
    uint32_t usp = SystemCoreClock / 1000000u;   /* 每微秒周期数 */
    if (usp == 0u) usp = 1u;

    if (sessions != 0u)
    {
        dbg_printf("[SD-%s] n=%lu sd=%lu/%lu/%luus gap=%luus %s=%lu bps=%lu tot=%lums\r\n",
                   tag,
                   (unsigned long)st->cnt,
                   (unsigned long)(st->sum / st->cnt / usp),
                   (unsigned long)(st->min / usp),
                   (unsigned long)(st->max / usp),
                   (unsigned long)(st->gap / st->gap_n / usp),
                   poll_tag,
                   (unsigned long)polls,
                   (unsigned long)(st->cnt / sessions),
                   (unsigned long)(st->sum / (SystemCoreClock / 1000u)));
    }
    else
    {
        dbg_printf("[SD-%s] n=%lu sd=%lu/%lu/%luus gap=%luus %s=%lu tot=%lums\r\n",
                   tag,
                   (unsigned long)st->cnt,
                   (unsigned long)(st->sum / st->cnt / usp),
                   (unsigned long)(st->min / usp),
                   (unsigned long)(st->max / usp),
                   (unsigned long)(st->gap / st->gap_n / usp),
                   poll_tag,
                   (unsigned long)polls,
                   (unsigned long)(st->sum / (SystemCoreClock / 1000u)));
    }

    st->cnt = 0u;
    st->sum = 0u;
    st->min = 0xFFFFFFFFu;
    st->max = 0u;
    st->gap = 0u;
    st->gap_n = 0u;
}

void usb_storage_ping(void)
{
    s_last_active_ms = HAL_GetTick();
}

uint32_t usb_storage_last_active_ms(void)
{
    return s_last_active_ms;
}

/* 访问前确保 SD 已初始化（usbstor 可能比 sd_log 早问容量） */
static int8_t storage_sd_ensure(void)
{
    if (!SD_Ready())
    {
        return (SD_Init() == 0) ? 0 : -1;
    }
    return 0;
}

/* SCSI INQUIRY 数据（36 字节） */
static const int8_t s_inquiry[] =
{
    0x00, 0x80, 0x02, 0x02,                 /* 直接访问 / RMB=1(可移动) / SCSI-2 */
    (36 - 5),                               /* additional length */
    0x00, 0x00, 0x00,
    'S', 'T', 'M', '3', '2', ' ', ' ', ' ',            /* Vendor: 8 */
    'M', 'o', 'u', 's', 'e', ' ', 'U', 'D', 'i', 's', 'k', ' ', ' ', ' ', ' ', ' ',  /* Product:16 */
    '1', '.', '0', '0',                     /* Rev: 4 */
};

void usb_storage_preinit(void)
{
    stat_dwt_init();
#if !SD_BYPASS_TEST
    uint32_t blocks = 0;
    if (storage_sd_ensure() == 0 && SD_GetBlockCount(&blocks) == 0)
    {
        s_cap_blocks = blocks;
        s_cap_size = SD_BLOCK_SIZE;
        s_cap_ok = 1;
        dbg_printf("[CAP] preinit ok blocks=%lu\r\n", (unsigned long)blocks);
    }
    else
    {
        dbg_printf("[CAP] preinit fail\r\n");
    }
#else
    (void)0;
#endif
}

void usb_storage_msc_task_init(void)
{
    rtos_queue_handle_t q = NULL;
    if (rtos_queue_create(2, 1, &q) == 0)
    {
        scsi_msc_set_signal_queue(q);
    }
}

static int8_t sd_storage_init(uint8_t lun)
{
    (void)lun;
    return 0;
}

static int8_t sd_storage_get_capacity(uint8_t lun, uint32_t *block_num, uint16_t *block_size)
{
    (void)lun;
    usb_storage_ping();
#if SD_BYPASS_TEST
    *block_num = 32768U;          /* 固定 16MB，便于观察枚举 */
    *block_size = SD_BLOCK_SIZE;
    return 0;
#else
    if (!s_cap_ok)
    {
        uint32_t blocks = 0;
        if (storage_sd_ensure() != 0 || SD_GetBlockCount(&blocks) != 0)
        {
            return -1;
        }
        s_cap_blocks = blocks;
        s_cap_size = SD_BLOCK_SIZE;
        s_cap_ok = 1;
    }
    *block_num = s_cap_blocks;
    *block_size = s_cap_size;
    return 0;
#endif
}

static int8_t sd_storage_is_ready(uint8_t lun)
{
    (void)lun;
    return 0;   /* 0 = 就绪（简化） */
}

static int8_t sd_storage_is_write_protected(uint8_t lun)
{
    (void)lun;
    return 0;
}

static int8_t sd_storage_read(uint8_t lun, uint8_t *buf, uint32_t blk_addr, uint16_t blk_len)
{
    uint32_t t_entry = DWT->CYCCNT;   /* 入口即取，gap 要含 storage_sd_ensure 的时间 */
    (void)lun;
    usb_storage_ping();
#if SD_BYPASS_TEST
    memset(buf, 0, (size_t)blk_len * SD_BLOCK_SIZE);
    return 0;
#else
    if (storage_sd_ensure() != 0)
    {
        return -1;
    }
    if (s_rd_gap_armed)
    {
        stat_add_gap(&s_rd_stat, t_entry - s_rd_last_exit);
    }
    s_rd_gap_armed = 1u;

    for (uint16_t i = 0; i < blk_len; i++)
    {
        uint32_t t0 = DWT->CYCCNT;
        if (SD_ReadBlock(blk_addr + i, buf + (uint32_t)i * SD_BLOCK_SIZE) != 0)
        {
            return -1;
        }
        stat_add(&s_rd_stat, DWT->CYCCNT - t0);
    }

    s_rd_last_exit = DWT->CYCCNT;
    if (s_rd_stat.cnt >= STAT_FLUSH_BLOCKS)
    {
        stat_report("RD", &s_rd_stat, 0u, "tokpoll", SD_TakeTokenPolls());
        s_rd_last_exit = DWT->CYCCNT;   /* 打印自身 ~6.5ms 不算进下一次 gap */
    }
    return 0;
#endif
}

static int8_t sd_storage_write(uint8_t lun, uint8_t *buf, uint32_t blk_addr, uint16_t blk_len)
{
    uint32_t t_entry = DWT->CYCCNT;
    (void)lun;
    usb_storage_ping();
#if SD_BYPASS_TEST
    return 0;   /* 空写 */
#else
    if (storage_sd_ensure() != 0)
    {
        return -1;
    }
    if (s_wr_gap_armed)
    {
        stat_add_gap(&s_wr_stat, t_entry - s_wr_last_exit);
    }
    s_wr_gap_armed = 1u;

    for (uint16_t i = 0; i < blk_len; i++)
    {
        uint32_t blk = blk_addr + i;
        const uint8_t *p = buf + (uint32_t)i * SD_BLOCK_SIZE;
        uint32_t t0 = DWT->CYCCNT;

        /* 会话跨多次 WRITE10 保持打开，只要块号连续就一直流下去。
         * 不连续（主机换了写入位置）时 SD_WriteChunk 会拒绝，重开会话再试。
         * 会话不在这里 End：留给下一次 Chunk 继续，或由任何其它 SD 事务的
         * sd_cs_low() 自动 STOP_TRAN 终结（sd_spi.c）。 */
        if (SD_WriteChunk(blk, p) != 0)
        {
            if (SD_WriteBegin(blk) != 0)
            {
                return -1;
            }
            s_wr_sessions++;
            if (SD_WriteChunk(blk, p) != 0)
            {
                return -1;
            }
        }
        stat_add(&s_wr_stat, DWT->CYCCNT - t0);
    }

    s_wr_last_exit = DWT->CYCCNT;
    if (s_wr_stat.cnt >= STAT_FLUSH_BLOCKS)
    {
        stat_report("WR", &s_wr_stat, s_wr_sessions, "wrpoll", SD_TakeWaitReadyPolls());
        s_wr_sessions = 0u;
        s_wr_last_exit = DWT->CYCCNT;   /* 打印自身 ~6.5ms 不算进下一次 gap */
    }
    return 0;
#endif
}

static int8_t sd_storage_get_max_lun(void)
{
    return 0;   /* 单 LUN */
}

USBD_StorageTypeDef USBD_SD_Storage_fops =
{
    sd_storage_init,
    sd_storage_get_capacity,
    sd_storage_is_ready,
    sd_storage_is_write_protected,
    sd_storage_read,
    sd_storage_write,
    sd_storage_get_max_lun,
    (int8_t *)s_inquiry,
};
