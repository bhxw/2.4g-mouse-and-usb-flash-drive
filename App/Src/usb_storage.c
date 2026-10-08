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
#include "usbd_pma_db.h"    /* 双缓冲层诊断计数，跟 [SD-RD]/[SD-WR] 行一起打 */

#include <string.h>

/* 诊断：1=U盘不碰SD(固定容量/空读写)；0=真实SD */
#define SD_BYPASS_TEST  0

/* 第二层（散块）写路径，第 1 步 —— **实测否决，默认关**。
 * 1 = 当前 CMD25 会话不连续、且本笔块数 <= MSC_WR_SINGLE_BLKS_MAX 时，逐块改走 CMD24 单块写；
 * 0 = 原路径（每笔散块重开一个 CMD25 会话）。两条路径共用同一份 CS/总线原语。
 * 2026-10-05 上板实测（40×8KB 小文件负载，A/B/A 三轮，见开发日志第十九条）：
 *   0（原路径）        p50 = 56.1ms/文件，sd 均值 0.88~3.1ms/块，会话 close 3.8~7ms/次
 *   阈值 1             p50 = 56.7ms/文件（无收益：主机元数据写多为 2 块，阈值 1 只覆盖少数笔）
 *   阈值 2（覆盖元数据写）p50 = 391ms/文件（慢 7 倍），sd 均值 7.4ms/块、**min 3.0ms/块**
 * 机制：CMD24 把"卡把多块写收尾"这段拖延成本从每次会话一次（已摊到 6~10 块）变成每块一次，
 * 卡侧单块同步编程实测 ≥3.0ms/块。⇒ 绕过 CMD25 会话没有收益，只有回退。
 * 代码留着只为记录"这条路试过"，不要在没有新证据时打开。 */
#define MSC_WR_SINGLE_BLOCK     0
#define MSC_WR_SINGLE_BLKS_MAX  2u  /* 仅在 MSC_WR_SINGLE_BLOCK=1 时有意义 */

static volatile uint32_t s_last_active_ms = 0;
static uint32_t s_cap_blocks = 0;
static uint16_t s_cap_size = 0;
static uint8_t  s_cap_ok = 0;

/* ---------------- MSC 入口流量计数（诊断用，语义见 usb_storage.h 的 msc_traffic_t） ----------------
 * 计数点在文件末尾的两个 `_timed` 包装里（fops 表指向它们），失败路径也必然进账 ——
 * 这正是 `[SD-RD]`/`[SD-WR]` 的结构性盲区：那两处失败分支都在 stat_add 之前 return，
 * 主机在重试退避期间串口一条都不出，与"主机没发"无法区分。
 * 写方只有 scsi_msc 任务（读/写 fops 全在任务上下文），读方只有 sysmon（优先级更低）；
 * 不清零，由 sysmon 存上一份快照算差值。 */
static msc_traffic_t s_traffic;
static uint32_t s_rd_last_end = 0xFFFFFFFFu;   /* 上次读的结尾块号；哨兵 = 还没有上一次 */
static uint32_t s_wr_last_end = 0xFFFFFFFFu;

void usb_storage_traffic(msc_traffic_t *t)
{
    if (t != NULL)
    {
        *t = s_traffic;
    }
}

/* ---------------- 传输耗时统计（每 256 块汇总一次） ----------------
 * 用 DWT->CYCCNT（72MHz，14ns 分辨率）而非 HAL_GetTick()：
 * 单块耗时在毫秒以下时 1ms 量化会给出 0/1，误差 ±100%，不可用。
 * 累加到 256 块再打印，一是摊薄量化误差，二是把 dbg_printf 自身的
 * ~2ms 阻塞开销（console.c:31 走 HAL_UART_Transmit + HAL_MAX_DELAY）
 * 从每块 1 次降到每 256 块 1 次。
 *
 * 每块时间拆成两段测：sd（SD_ReadChunk / SD_WriteChunk 内部，**每块记一次**）+
 * gap（两次 fops 调用之间，**每次调用记一次**）。
 * ⚠ 2026-10-05 起 MSC_STREAM_PACKET=1024 让 blk_count 常为 2：cnt 数块、gap_n 数调用，
 * 两者口径不同 —— closure 要按 (sd_avg*cnt + gap_sum)/cnt ≈ 墙钟/块数 验，
 * **不要再写 sd_avg + gap_avg ≈ 墙钟/块数**（旧注释那句只在 blk_count 恒为 1 时成立）。
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
    uint32_t gap_min; /* 周期数。gap 均值会被长空闲拉爆（见 stat_add_gap），稳态值要看 min */
    uint32_t gap_max; /* 周期数。逼近 59.6s 说明本窗口被空闲污染，均值不可用 */
    /* gap 样本的粗分布：h[0]=<1ms h[1]=1~3ms h[2]=3~10ms h[3]=>=10ms。
     * 光有 avg/min/max 判不出来：实测元数据窗 gap 均值 6282us 而 max=630332us，
     * 均值是被**一条** 630ms 样本顶起来的，不是 128 笔 6.3ms 的主机节奏 —— 分布才看得出。 */
    uint32_t gap_h[4];
} sd_stat_t;

static sd_stat_t s_rd_stat = {0, 0, 0xFFFFFFFFu, 0, 0, 0, 0xFFFFFFFFu, 0, {0, 0, 0, 0}};
static sd_stat_t s_wr_stat = {0, 0, 0xFFFFFFFFu, 0, 0, 0, 0xFFFFFFFFu, 0, {0, 0, 0, 0}};
static uint32_t  s_wr_sessions = 0;   /* 本窗口内 SD_WriteBegin 成功次数 */
static uint32_t  s_rd_sessions = 0;   /* 本窗口内 SD_ReadBegin 成功次数 */

/* 对向 fops 计数：一个方向的窗口行要知道"这期间另一个方向被调用了多少次、共花掉多少毫秒"。
 * gap 样本只覆盖同向两次调用之间，跨向调用的整段耗时（含对向的会话终结 / 卡 busy）会整段
 * 落进 gap；没有这个计数就分不清一条超长 gap 样本是"主机不说话"还是"对向调用在跑"。 */
static uint32_t s_x_rd_n   = 0;
static uint64_t s_x_rd_cyc = 0;
static uint32_t s_x_wr_n   = 0;
static uint64_t s_x_wr_cyc = 0;

/* 块间间隔探针：这段窗口里 EP2 已经重新挂好 PrepareReceive（usbd_msc_scsi.c:105），
 * 设备随时可收，NAK 期算在 sd 那一段里面而不是 gap 里。所以 gap 是纯
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

/* gap 的均值单独看没有意义：CYCCNT 是 32 位、59.6s 回绕，一次长空闲（主机不来读/写）
 * 就能把一个样本顶到接近回绕点，256 块平均下来整窗口失真 —— 实测出过 gap 均值 287ms
 * 而稳态只有 542us 的窗口，反推至少有 2 条几十秒级的样本。min 才是稳态值，
 * max 用来判断本窗口被污染了多少；max 逼近 59.6s 时应读作"≥59.6s，不可分辨"。 */
static void stat_add_gap(sd_stat_t *st, uint32_t cyc)
{
    uint32_t per_ms = SystemCoreClock / 1000u;   /* 每毫秒周期数（72MHz -> 72000） */

    st->gap += cyc;
    st->gap_n++;
    if (cyc < st->gap_min) st->gap_min = cyc;
    if (cyc > st->gap_max) st->gap_max = cyc;

    if (cyc < per_ms)              st->gap_h[0]++;
    else if (cyc < 3u * per_ms)    st->gap_h[1]++;
    else if (cyc < 10u * per_ms)   st->gap_h[2]++;
    else                           st->gap_h[3]++;
}

/* 每 256 块仍然只打一行（不增加打印次数，免得和 sysmon 撞 UART 被 HAL_BUSY 静默丢掉）。
 * 字段口径：
 *   n     本窗口块数；
 *   sd    avg/min/max（**每块**）：覆盖 Chunk 尝试 + 会话重开（t0 取在 Chunk 之前，Begin 落在里面）；
 *   gap   avg/min/max(样本数)（**每次 fops 调用**）：MSC_STREAM_PACKET=1024 使 blk_count 常为 2，
 *         gap_n ≈ 块数/2，与 sd 的每块均值**不可相加**，闭合验算用 (sd_avg*cnt + gap_avg*gap_n)
 *         ≈ 窗口墙钟。括号里是样本数 —— 均值离开样本数没有意义（见下）；
 *   gh    gap 样本分布 <1ms / 1~3ms / 3~10ms / >=10ms：用来区分"很多笔正常间隔"与
 *         "一条几百毫秒的停顿顶起均值"。实测元数据窗 gap 均值 6282us 而 max=630332us，
 *         上一轮把均值读成"每笔间隔 6.3ms × 128 笔"就是被这一个样本带偏的；
 *   x**   对向 fops：调用次数 / 总微秒（写行打 xrd，读行打 xwr）。gap 只覆盖同向两次调用之间，
 *         跨向调用的整段耗时也落在 gap 里，没有这个数就分不清是"主机不说话"还是"对向在跑"；
 *   sess  会话重开：Begin 次数 / close 累计us / open 累计us（close=STOP_TRAN/CMD12 + 卡 busy）；
 *   poll_tag/polls  本窗口轮询圈数总数（打总数不打平均，免得整数除法把 <1 的值吃掉）；
 *   bps=blk/sess。sessions=0 时整个字段不打印 —— 会话跨窗口一直开着，比"一窗 256"更好。 */
static void stat_report(const char *tag, sd_stat_t *st, uint32_t sessions,
                        const char *poll_tag, uint32_t polls)
{
    uint32_t usp = SystemCoreClock / 1000000u;   /* 每微秒周期数 */
    uint32_t db_rs = 0u;                         /* 自研双缓冲层：相位强复位次数 */
    uint32_t db_an = 0u;                         /* 自研双缓冲层：异常计数（空包/无缓冲/伪 CTR） */
    uint8_t  is_wr = (uint8_t)(tag[0] == 'W');
    sd_sess_stat_t sess = {0u, 0u, 0u};
    uint32_t x_n   = is_wr ? s_x_rd_n : s_x_wr_n;
    uint64_t x_cyc = is_wr ? s_x_rd_cyc : s_x_wr_cyc;
    if (usp == 0u) usp = 1u;
    PMA_DB_GetCounters(NULL, NULL, &db_rs, &db_an);
    SD_TakeSessionStats(is_wr ? 0u : 1u, &sess);
    if (is_wr)
    {
        s_x_rd_n = 0u;
        s_x_rd_cyc = 0u;
    }
    else
    {
        s_x_wr_n = 0u;
        s_x_wr_cyc = 0u;
    }

    if (sessions != 0u)
    {
        dbg_printf("[SD-%s] n=%lu sd=%lu/%lu/%luus gap=%lu/%lu/%luus(%lu) gh=%lu/%lu/%lu/%lu "
                   "x%s=%lu/%luus sess=%lu/%lu/%luus %s=%lu bps=%lu db=%lu/%lu\r\n",
                   tag,
                   (unsigned long)st->cnt,
                   (unsigned long)(st->sum / st->cnt / usp),
                   (unsigned long)(st->min / usp),
                   (unsigned long)(st->max / usp),
                   (unsigned long)(st->gap / st->gap_n / usp),
                   (unsigned long)(st->gap_min / usp),
                   (unsigned long)(st->gap_max / usp),
                   (unsigned long)st->gap_n,
                   (unsigned long)st->gap_h[0],
                   (unsigned long)st->gap_h[1],
                   (unsigned long)st->gap_h[2],
                   (unsigned long)st->gap_h[3],
                   is_wr ? "rd" : "wr",
                   (unsigned long)x_n,
                   (unsigned long)(x_cyc / usp),
                   (unsigned long)sess.n,
                   (unsigned long)(sess.close_cyc / usp),
                   (unsigned long)(sess.open_cyc / usp),
                   poll_tag,
                   (unsigned long)polls,
                   (unsigned long)(st->cnt / sessions),
                   (unsigned long)db_rs,
                   (unsigned long)db_an);
    }
    else
    {
        dbg_printf("[SD-%s] n=%lu sd=%lu/%lu/%luus gap=%lu/%lu/%luus(%lu) gh=%lu/%lu/%lu/%lu "
                   "x%s=%lu/%luus sess=%lu/%lu/%luus %s=%lu db=%lu/%lu\r\n",
                   tag,
                   (unsigned long)st->cnt,
                   (unsigned long)(st->sum / st->cnt / usp),
                   (unsigned long)(st->min / usp),
                   (unsigned long)(st->max / usp),
                   (unsigned long)(st->gap / st->gap_n / usp),
                   (unsigned long)(st->gap_min / usp),
                   (unsigned long)(st->gap_max / usp),
                   (unsigned long)st->gap_n,
                   (unsigned long)st->gap_h[0],
                   (unsigned long)st->gap_h[1],
                   (unsigned long)st->gap_h[2],
                   (unsigned long)st->gap_h[3],
                   is_wr ? "rd" : "wr",
                   (unsigned long)x_n,
                   (unsigned long)(x_cyc / usp),
                   (unsigned long)sess.n,
                   (unsigned long)(sess.close_cyc / usp),
                   (unsigned long)(sess.open_cyc / usp),
                   poll_tag,
                   (unsigned long)polls,
                   (unsigned long)db_rs,
                   (unsigned long)db_an);
    }

    st->cnt = 0u;
    st->sum = 0u;
    st->min = 0xFFFFFFFFu;
    st->max = 0u;
    st->gap = 0u;
    st->gap_n = 0u;
    st->gap_min = 0xFFFFFFFFu;
    st->gap_max = 0u;
    st->gap_h[0] = 0u;
    st->gap_h[1] = 0u;
    st->gap_h[2] = 0u;
    st->gap_h[3] = 0u;
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
        SD_LogInitDiag();
    }
#else
    (void)0;
#endif
}

void usb_storage_msc_task_init(void)
{
    rtos_queue_handle_t q = NULL;

    /* Runs before MX_USB_DEVICE_Init(), so the pipeline buffers exist before
       the first CBW can make the USB ISR dereference them. */
    if (scsi_msc_buffers_init() != 0)
    {
        dbg_printf("[MSC] pipeline buffer alloc failed\r\n");
    }

    if (rtos_queue_create(2, sizeof(scsi_msc_sig_t), &q) == 0)
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

/* 读路径失败明细（诊断，只在失败时打印，上限 8 行）
 * rd  = 首次 SD_ReadChunk 的返回码（非 0 = "会话不连续"这类拒绝；瞬时返回 = 根本没上 SPI）
 * beg = SD_ReadBegin 的返回码（0 = 会话已重开；10 = SD_ERR_CMD18 = 卡的 CMD18 应答不对；8 = 超时）
 * rd2 = 重开后的 SD_ReadChunk 返回码（0xFF = 没走到；6 = SD_ERR_READ；8 = SD_ERR_TIMEOUT = 等 0xFE 令牌超时）
 * us  = 这一块尝试的耗时（µs，DWT@72MHz）：几微秒 ⇒ 纯软件拒绝；≈200000 ⇒ 令牌超时 200ms */
static uint8_t s_rfail;

static void rd_fail_report(uint32_t blk, uint8_t rd, uint8_t beg, uint8_t rd2, uint32_t us)
{
    if (s_rfail < 8u)
    {
        s_rfail++;
        dbg_printf("[SDR] blk=%lu rd=%u beg=%u rd2=%u us=%lu n=%u\r\n",
                   (unsigned long)blk, (unsigned)rd, (unsigned)beg, (unsigned)rd2,
                   (unsigned long)us, (unsigned)s_rfail);
    }
    else if (s_rfail == 8u)
    {
        s_rfail++;
        dbg_printf("[SDR] more suppressed\r\n");
    }
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
        uint32_t blk = blk_addr + i;
        uint8_t *p = buf + (uint32_t)i * SD_BLOCK_SIZE;
        uint32_t t0 = DWT->CYCCNT;

        /* 会话跨多次 READ10 保持打开，只要块号连续就一直流下去（与写侧同构）。
         * 不连续（主机转去读 FAT/目录项）时 SD_ReadChunk 会拒绝，重开会话再试。
         * 会话不在这里 End：留给下一次 Chunk 继续，或由任何其它 SD 事务的
         * sd_cs_low() 自动发 CMD12 终结（sd_spi.c）。 */
        uint8_t rd = SD_ReadChunk(blk, p);

        if (rd != 0)
        {
            uint8_t beg = SD_ReadBegin(blk);
            uint8_t rd2 = 0xFFu;   /* 0xFF = 没走到重试 */

            if (beg == 0u)
            {
                s_rd_sessions++;
                rd2 = SD_ReadChunk(blk, p);
            }
            if (beg != 0u || rd2 != 0u)
            {
                rd_fail_report(blk, rd, beg, rd2, (DWT->CYCCNT - t0) / 72u);
                return -1;
            }
        }
        stat_add(&s_rd_stat, DWT->CYCCNT - t0);
    }

    s_rd_last_exit = DWT->CYCCNT;
    if (s_rd_stat.cnt >= STAT_FLUSH_BLOCKS)
    {
        stat_report("RD", &s_rd_stat, s_rd_sessions, "tokpoll", SD_TakeTokenPolls());
        s_rd_sessions = 0u;
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
#if MSC_WR_SINGLE_BLOCK
            /* 散块直通（第 1 步）：⚠ **实测否决、默认关** —— 依据见文件头 MSC_WR_SINGLE_BLOCK 的实测表。
             * 打开后小文件负载 p50 从 56ms 掉到 391ms：CMD24 会让卡对每块做同步编程（≥3.0ms/块），
             * 而 CMD25 会话收尾那段等待是**每会话一次**、且已摊到 6~10 块上。 */
            if (blk_len <= MSC_WR_SINGLE_BLKS_MAX)
            {
                for (uint16_t k = i; k < blk_len; k++)
                {
                    uint32_t t1 = DWT->CYCCNT;

                    if (SD_WriteBlock(blk_addr + k, buf + (uint32_t)k * SD_BLOCK_SIZE) != 0)
                    {
                        return -1;
                    }
                    stat_add(&s_wr_stat, DWT->CYCCNT - t1);
                }
                break;   /* 本笔剩下的块都已经按单块路径写完 */
            }
#endif
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

/* 对向 fops 计时包装：内容完全不变，只把整次调用（含所有提前 return 的失败路径）的墙钟
 * 记进 s_x_*，供另一方向的窗口行打印。fops 表指向这两个包装，不指向实现本身。 */
static int8_t sd_storage_read_timed(uint8_t lun, uint8_t *buf, uint32_t blk_addr, uint16_t blk_len)
{
    uint32_t t0 = DWT->CYCCNT;
    int8_t r = sd_storage_read(lun, buf, blk_addr, blk_len);
    uint32_t tend = blk_addr + blk_len;

    s_x_rd_n++;
    s_x_rd_cyc += DWT->CYCCNT - t0;

    /* MSC 入口流量（诊断）：整次调用无论成败都进账，见文件顶部的 s_traffic 说明。
     * 放在计时之后，不污染 s_x_*。 */
    s_traffic.rd_calls++;
    s_traffic.rd_blocks += blk_len;
    if (s_rd_last_end != 0xFFFFFFFFu && blk_addr != s_rd_last_end)
    {
        s_traffic.rd_jumps++;
    }
    s_rd_last_end = tend;
    if (r != 0)
    {
        s_traffic.rd_fails++;
    }
    return r;
}

static int8_t sd_storage_write_timed(uint8_t lun, uint8_t *buf, uint32_t blk_addr, uint16_t blk_len)
{
    uint32_t t0 = DWT->CYCCNT;
    int8_t r = sd_storage_write(lun, buf, blk_addr, blk_len);
    uint32_t tend = blk_addr + blk_len;

    s_x_wr_n++;
    s_x_wr_cyc += DWT->CYCCNT - t0;

    s_traffic.wr_calls++;
    s_traffic.wr_blocks += blk_len;
    if (s_wr_last_end != 0xFFFFFFFFu && blk_addr != s_wr_last_end)
    {
        s_traffic.wr_jumps++;
    }
    s_wr_last_end = tend;
    if (r != 0)
    {
        s_traffic.wr_fails++;
    }
    return r;
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
    sd_storage_read_timed,
    sd_storage_write_timed,
    sd_storage_get_max_lun,
    (int8_t *)s_inquiry,
};
