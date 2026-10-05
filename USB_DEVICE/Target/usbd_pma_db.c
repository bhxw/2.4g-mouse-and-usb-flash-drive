/**
 * @file usbd_pma_db.c
 * @brief 自研 PMA 双缓冲（EP2 OUT / EP3 IN）的寄存器级实现，见 usbd_pma_db.h 的文件头。
 *
 * ---------------------------------------------------------------------------
 * 语义速查（推导依据：RM0008 双缓冲 bulk 端点 + 别人的两份对照实现，互相印证）
 *
 * 双缓冲 bulk 端点里 DTOG_TX / DTOG_RX 不再表示 DATA0/DATA1，而是"两块 PMA 缓冲的
 * 指针 + 软件释放位"：
 *     IN （本层 EP3）：H = DTOG_TX = 硬件将要发出去的那块
 *                      S = DTOG_RX = 软件填完一块翻它一次 = 交还
 *     OUT（本层 EP2）：H = DTOG_RX = 硬件将要写进去的那块
 *                      S = DTOG_TX = 软件搬完一块翻它一次 = 交还
 *
 * 规则一：H != S 时硬件才动；硬件每次收发完自己翻 H（于是 H == S，端点自动 NAK）；
 *         软件每翻一次 S 就精确放行一块缓冲。
 * 规则二：CTR 中断发生的时刻必有 H == S（硬件刚翻完），所以"刚被动过的那一块"
 *         = S ? BUF0 : BUF1。两份参照实现在这一点上取的位不同但等价：
 *           HAL 的 IN 分支用 DTOG_TX（此时 == DTOG_RX）；TeenyUSB 用翻位前的 DTOG_RX。
 * 规则三：块号承担 DATA0/DATA1（BUF0=DATA0 / BUF1=DATA1），所以**连续多笔事务之间
 *         绝不能复位相位**，否则下一笔的 PID 会对不上（CSW 尤其致命）。这也正是
 *         HAL 那条"短事务自动切回单缓冲"（每笔 CSW 抖一次 EP_KIND + 清 DTOG）的病根。
 * 规则四：EP_KIND 只能在端点 disabled 时改（RM0008）⇒ 运行期一次都不改，
 *         只在 OpenEP / 相位异常时设一次。
 *
 * 唯一主动复位相位的场合：上一笔被 stall/abort 打断（相位不在空闲态）时，以及
 * OpenEP / USB 复位 —— 对应协议上"数据翻转位归零到 DATA0"的要求。
 * ---------------------------------------------------------------------------
 */

#include "usbd_pma_db.h"
#include "console.h"

#define DB_CNT_MASK   0x03FFU

typedef struct
{
    uint8_t   is_in;
    uint8_t   num;
    uint16_t  maxpacket;
    uint16_t  pma0;                 /* BUF0：地址槽 +0，count 槽 +2 */
    uint16_t  pma1;                 /* BUF1：地址槽 +4，count 槽 +6 */

    /* IN：整笔请求的推进进度 */
    uint8_t  *tx_buf;
    uint16_t  tx_total;
    uint16_t  tx_remain;            /* 还没搬进 PMA 的字节数 */
    uint8_t   tx_pushed;            /* 已经在硬件手里的块数（0/1/2） */

    /* OUT：整笔请求的接收进度 */
    uint8_t  *rx_buf;
    uint16_t  rx_size;
    uint16_t  rx_count;
} pma_db_ep_t;

/* 只有两个 DB 端点，不做 8 项查表（RAM 只剩 ~1.9KB，省一点是一点） */
static pma_db_ep_t        s_ep_out;
static pma_db_ep_t        s_ep_in;
static PCD_HandleTypeDef *s_hpcd;
static uint8_t            s_reported;

/* 诊断计数（都应恒为 0；非 0 即状态机出了偏差，跟着 [SD-RD]/[SD-WR] 行的 db= 字段看） */
static uint32_t s_in_pkt, s_out_pkt, s_resync, s_anom;

#define DB_USB        (s_hpcd->Instance)
#define DB_NUM(addr)  ((uint8_t)((addr) & 0x7FU))

static uint32_t db_crit_enter(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}

static void db_crit_exit(uint32_t primask)
{
    __set_PRIMASK(primask);
}

/* 端点地址 → 本层的端点上下文；不是本层管的返回 NULL */
static pma_db_ep_t *db_lookup(uint8_t ep_addr)
{
#if (MSC_DB_OUT == 1)
    if (ep_addr == PMA_DB_EP_ADDR_OUT)
    {
        return &s_ep_out;
    }
#endif
#if (MSC_DB_IN == 1)
    if (ep_addr == PMA_DB_EP_ADDR_IN)
    {
        return &s_ep_in;
    }
#endif
    return NULL;
}

uint8_t PMA_DB_IsDbEp(uint8_t ep_addr)
{
    return (db_lookup(ep_addr) != NULL) ? 1U : 0U;
}

static void db_reset_progress(pma_db_ep_t *e)
{
    e->tx_buf    = NULL;
    e->tx_total  = 0U;
    e->tx_remain = 0U;
    e->tx_pushed = 0U;
    e->rx_buf    = NULL;
    e->rx_size   = 0U;
    e->rx_count  = 0U;
}

/* 两块地址/包长从 HAL 的端点结构里取（唯一来源是 usbd_conf.c 的 HAL_PCDEx_PMAConfig
 * 与 HAL_PCD_EP_Open），本层不另立一份常量，避免两处对不上。 */
static void db_sync_ep(pma_db_ep_t *e)
{
    const PCD_EPTypeDef *h = (e->is_in != 0U) ? &s_hpcd->IN_ep[e->num] : &s_hpcd->OUT_ep[e->num];

    e->pma0      = h->pmaaddr0;
    e->pma1      = h->pmaaddr1;
    e->maxpacket = (uint16_t)h->maxpacket;
}

static uint16_t db_buf_addr(const pma_db_ep_t *e, uint8_t idx)
{
    return (idx != 0U) ? e->pma1 : e->pma0;
}

/* 把端点拉回一个已知相位：两个方向先 DIS → KIND=1 + 两块地址 → 清两个 DTOG →
 * 本方向 VALID、反方向 DIS。只在 OpenEP / 相位异常时调用。 */
static void db_force_init(pma_db_ep_t *e)
{
    PCD_SET_EP_TX_STATUS(DB_USB, e->num, USB_EP_TX_DIS);
    PCD_SET_EP_RX_STATUS(DB_USB, e->num, USB_EP_RX_DIS);

    PCD_SET_BULK_EP_DBUF(DB_USB, e->num);
    PCD_SET_EP_DBUF_ADDR(DB_USB, e->num, e->pma0, e->pma1);

    PCD_CLEAR_RX_DTOG(DB_USB, e->num);
    PCD_CLEAR_TX_DTOG(DB_USB, e->num);

    if (e->is_in != 0U)
    {
        PCD_SET_EP_TX_STATUS(DB_USB, e->num, USB_EP_TX_VALID);
    }
    else
    {
        PCD_SET_EP_RX_STATUS(DB_USB, e->num, USB_EP_RX_VALID);
    }
}

/* ---------------- IN（EP3）：一块一块往 PMA 里填 ---------------- */

/* 往 idx 号块搬一包（最多 maxpacket）；count 只写这一块，另一块不碰。 */
static void db_tx_fill(pma_db_ep_t *e, uint8_t idx)
{
    uint16_t len = e->tx_remain;

    if (len > e->maxpacket)
    {
        len = e->maxpacket;
    }

    if (idx == 0U)
    {
        PCD_SET_EP_DBUF0_CNT(DB_USB, e->num, 1U, len);   /* +2 槽，IN 侧是纯字节数 */
    }
    else
    {
        PCD_SET_EP_DBUF1_CNT(DB_USB, e->num, 1U, len);   /* +6 槽 */
    }

    if (len != 0U)
    {
        USB_WritePMA(DB_USB, e->tx_buf, db_buf_addr(e, idx), len);
    }

    e->tx_buf    += len;
    e->tx_remain -= len;
    e->tx_pushed++;
}

/* CTR_TX：硬件刚把一块发出去 */
static void db_in_complete(pma_db_ep_t *e)
{
    uint16_t ep    = PCD_GET_ENDPOINT(DB_USB, e->num);      /* H == S 的快照 */
    uint8_t  freed = ((ep & USB_EP_DTOG_RX) != 0U) ? 0U : 1U;

    e->tx_pushed--;

    /* 还有一块在硬件手里就先交还下一块（先翻位、再填，填的是刚空出来的那块） */
    if (e->tx_pushed != 0U)
    {
        PCD_RX_DTOG(DB_USB, e->num);
    }

    if (e->tx_remain != 0U)
    {
        db_tx_fill(e, freed);
        PCD_SET_EP_TX_STATUS(DB_USB, e->num, USB_EP_TX_VALID);
        return;
    }

    if (e->tx_pushed == 0U)
    {
        s_hpcd->IN_ep[e->num].xfer_count = e->tx_total;
        HAL_PCD_DataInStageCallback(s_hpcd, e->num);
    }
}

/* ---------------- OUT（EP2）：一块一块从 PMA 里搬 ---------------- */

/* 放行一块：只在"两块都空"或"两块都满"（H == S）时翻 S，已经放行过就不重复翻。 */
static void db_rx_arm(pma_db_ep_t *e)
{
    uint16_t ep  = PCD_GET_ENDPOINT(DB_USB, e->num);
    uint16_t tog = (uint16_t)(ep & (USB_EP_DTOG_RX | USB_EP_DTOG_TX));

    if ((tog == 0U) || (tog == (USB_EP_DTOG_RX | USB_EP_DTOG_TX)))
    {
        PCD_TX_DTOG(DB_USB, e->num);
    }
}

/* CTR_RX：硬件刚把一块写满 */
static void db_out_complete(pma_db_ep_t *e)
{
    uint16_t ep  = PCD_GET_ENDPOINT(DB_USB, e->num);        /* H == S 的快照 */
    uint8_t  idx = ((ep & USB_EP_DTOG_TX) != 0U) ? 0U : 1U; /* 刚被写满的那块 */
    uint16_t cnt = ((idx == 0U) ? (uint16_t)PCD_GET_EP_DBUF0_CNT(DB_USB, e->num)
                                : (uint16_t)PCD_GET_EP_DBUF1_CNT(DB_USB, e->num)) & DB_CNT_MASK;
    uint8_t  done;

    if (e->rx_buf == NULL)
    {
        /* 没挂缓冲就来了数据（不该发生）：只把两块房间重编程，不回调，免得给上层假长度 */
        s_anom++;
        PCD_SET_EP_DBUF0_CNT(DB_USB, e->num, 0U, e->maxpacket);
        PCD_SET_EP_DBUF1_CNT(DB_USB, e->num, 0U, e->maxpacket);
        return;
    }

    done = (uint8_t)(((cnt < e->maxpacket) || ((uint16_t)(e->rx_count + cnt) >= e->rx_size)) ? 1U : 0U);

    if (done == 0U)
    {
        /* 还要接着收：先放行刚消费掉的那块，硬件就能在我们搬数据的这段时间收下一包 */
        PCD_TX_DTOG(DB_USB, e->num);
    }

    if (cnt != 0U)
    {
        USB_ReadPMA(DB_USB, e->rx_buf + e->rx_count, db_buf_addr(e, idx), cnt);
    }
    else
    {
        s_anom++;
    }

    e->rx_count += cnt;

    /* 与 HAL 单缓冲路径同口径：xfer_count 就是 BOT 层 USBD_LL_GetRxDataSize 读的那个数 */
    s_hpcd->OUT_ep[e->num].xfer_count = e->rx_count;
    s_hpcd->OUT_ep[e->num].xfer_buff  = e->rx_buf + e->rx_count;
    s_hpcd->OUT_ep[e->num].xfer_len   = (e->rx_size > e->rx_count) ? (uint16_t)(e->rx_size - e->rx_count) : 0U;

    if (done != 0U)
    {
        /* 整笔结束：不再放行 ⇒ H == S，端点自动 NAK，等下一笔 PrepareReceive */
        HAL_PCD_DataOutStageCallback(s_hpcd, e->num);
    }
}

/* ---------------- 对外接口 ---------------- */

void PMA_DB_Init(PCD_HandleTypeDef *hpcd)
{
    s_hpcd = hpcd;

    s_ep_out.is_in     = 0U;
    s_ep_out.num       = DB_NUM(PMA_DB_EP_ADDR_OUT);
    s_ep_out.maxpacket = 0U;
    s_ep_out.pma0      = 0U;
    s_ep_out.pma1      = 0U;
    db_reset_progress(&s_ep_out);

    s_ep_in.is_in      = 1U;
    s_ep_in.num        = DB_NUM(PMA_DB_EP_ADDR_IN);
    s_ep_in.maxpacket  = 0U;
    s_ep_in.pma0       = 0U;
    s_ep_in.pma1       = 0U;
    db_reset_progress(&s_ep_in);

    /* 地址与 maxpacket 要等 HAL_PCD_EP_Open 之后才有，这里只登记端点归属 */
}

HAL_StatusTypeDef PMA_DB_Transmit(uint8_t ep_addr, uint8_t *pbuf, uint16_t len)
{
    pma_db_ep_t *e = db_lookup(ep_addr);
    PCD_EPTypeDef *he;
    uint32_t crit;
    uint16_t ep;
    uint8_t  first;

    if (e == NULL)
    {
        return HAL_ERROR;
    }

    he = &s_hpcd->IN_ep[e->num];
    he->xfer_buff  = pbuf;
    he->xfer_len   = len;
    he->xfer_count = 0U;
    he->is_in      = 1U;
    he->num        = e->num;

    crit = db_crit_enter();

    if ((e->tx_pushed != 0U) || (e->tx_remain != 0U))
    {
        s_anom++;
        db_crit_exit(crit);
        return HAL_BUSY;
    }

    db_sync_ep(e);
    e->tx_buf    = pbuf;
    e->tx_total  = len;
    e->tx_remain = len;

    ep = PCD_GET_ENDPOINT(DB_USB, e->num);
    if (((ep & (USB_EP_DTOG_RX | USB_EP_DTOG_TX)) != 0U) &&
        ((ep & (USB_EP_DTOG_RX | USB_EP_DTOG_TX)) != (USB_EP_DTOG_RX | USB_EP_DTOG_TX)))
    {
        /* 相位不在空闲态：上一笔被 stall/abort 打断过，先复位再发 */
        s_resync++;
        db_force_init(e);
        ep = PCD_GET_ENDPOINT(DB_USB, e->num);
    }

    first = ((ep & USB_EP_DTOG_RX) != 0U) ? 1U : 0U;   /* 软件指着哪块就先填哪块 */

    db_tx_fill(e, first);                              /* 一次填两块：硬件手里一块、软件再备一块 */
    if (e->tx_remain != 0U)
    {
        db_tx_fill(e, (uint8_t)(first ^ 1U));
    }
    if (e->tx_pushed == 0U)
    {
        db_tx_fill(e, first);                          /* len==0 的 ZLP 兜底 */
    }

    PCD_RX_DTOG(DB_USB, e->num);                       /* 交还：一次翻位放行（到）两块中的第一块 */
    PCD_SET_EP_TX_STATUS(DB_USB, e->num, USB_EP_TX_VALID);

    db_crit_exit(crit);
    return HAL_OK;
}

HAL_StatusTypeDef PMA_DB_PrepareReceive(uint8_t ep_addr, uint8_t *pbuf, uint16_t len)
{
    pma_db_ep_t *e = db_lookup(ep_addr);
    PCD_EPTypeDef *he;
    uint32_t crit;

    if (e == NULL)
    {
        return HAL_ERROR;
    }

    he = &s_hpcd->OUT_ep[e->num];
    he->xfer_buff  = pbuf;
    he->xfer_len   = len;
    he->xfer_count = 0U;
    he->is_in      = 0U;
    he->num        = e->num;

    crit = db_crit_enter();

    db_sync_ep(e);

    /* 两块都按 maxpacket 重编程（把上一笔硬件写回的裸长度盖掉），两块房间各 64B */
    PCD_SET_EP_DBUF0_CNT(DB_USB, e->num, 0U, e->maxpacket);
    PCD_SET_EP_DBUF1_CNT(DB_USB, e->num, 0U, e->maxpacket);

    e->rx_buf   = pbuf;
    e->rx_size  = len;
    e->rx_count = 0U;

    db_rx_arm(e);
    PCD_SET_EP_RX_STATUS(DB_USB, e->num, USB_EP_RX_VALID);

    db_crit_exit(crit);
    return HAL_OK;
}

void PMA_DB_OpenEp(uint8_t ep_addr)
{
    pma_db_ep_t *e = db_lookup(ep_addr);
    uint32_t crit;

    if (e == NULL)
    {
        return;
    }

    crit = db_crit_enter();

    db_sync_ep(e);
    db_reset_progress(e);

    /* HAL_ActivateEndpoint 已经按 doublebuffer==1 把 KIND/两块地址/清 DTOG 做过了；
     * 这里补的是它没做的 STAT_TX=VALID（它对 DB 的 IN 端点置的是 NAK）。 */
    db_force_init(e);

    db_crit_exit(crit);
}

void PMA_DB_Reset(void)
{
    uint32_t crit = db_crit_enter();

    db_reset_progress(&s_ep_out);
    db_reset_progress(&s_ep_in);

    /* 总线上复位后一律先关掉，等主机重新 SET_CONFIGURATION 走 OpenEP 再武装 */
#if (MSC_DB_OUT == 1)
    PCD_SET_EP_TX_STATUS(DB_USB, DB_NUM(PMA_DB_EP_ADDR_OUT), USB_EP_TX_DIS);
    PCD_SET_EP_RX_STATUS(DB_USB, DB_NUM(PMA_DB_EP_ADDR_OUT), USB_EP_RX_DIS);
#endif
#if (MSC_DB_IN == 1)
    PCD_SET_EP_TX_STATUS(DB_USB, DB_NUM(PMA_DB_EP_ADDR_IN), USB_EP_TX_DIS);
    PCD_SET_EP_RX_STATUS(DB_USB, DB_NUM(PMA_DB_EP_ADDR_IN), USB_EP_RX_DIS);
#endif

    db_crit_exit(crit);
}

void PMA_DB_IRQHandler(void)
{
    uint16_t istr;

    /* 只吃掉本层的 CTR，别的一律留给 HAL_PCD_IRQHandler。LP 与 HP 两条中断线共用本
     * 函数（F1 上双缓冲 bulk IN 的 CTR_TX 走 HP 线，见 usbd_conf.c 的 NVIC 说明），
     * 两条线同优先级 ⇒ 不会互相抢占，共用一份状态没有竞态。 */
    while (((istr = DB_USB->ISTR) & USB_ISTR_CTR) != 0U)
    {
        uint8_t num = (uint8_t)(istr & USB_ISTR_EP_ID);

#if (MSC_DB_OUT == 1)
        if (num == DB_NUM(PMA_DB_EP_ADDR_OUT))
        {
            PCD_CLEAR_RX_EP_CTR(DB_USB, num);
            s_out_pkt++;
            db_out_complete(&s_ep_out);
            continue;
        }
#endif
#if (MSC_DB_IN == 1)
        if (num == DB_NUM(PMA_DB_EP_ADDR_IN))
        {
            PCD_CLEAR_TX_EP_CTR(DB_USB, num);
            s_in_pkt++;
            if (s_ep_in.tx_pushed != 0U)
            {
                db_in_complete(&s_ep_in);
            }
            else
            {
                s_anom++;
            }
            continue;
        }
#endif
        break;   /* 不是本层负责的端点：交回 HAL_PCD_IRQHandler */
    }
}

void PMA_DB_Report(void)
{
    uint16_t ep_in;
    uint16_t ep_out;

    if (s_reported != 0U)
    {
        return;
    }
    s_reported = 1U;

    ep_in  = PCD_GET_ENDPOINT(DB_USB, DB_NUM(PMA_DB_EP_ADDR_IN));
    ep_out = PCD_GET_ENDPOINT(DB_USB, DB_NUM(PMA_DB_EP_ADDR_OUT));

    /* 一行打全：KIND 位（0x0100）在寄存器原值里直接可见 = 双缓冲真的生效了 */
    dbg_printf("[DB] IN%d db=%u pma=%03X/%03X R=%04X | OUT%d db=%u pma=%03X/%03X R=%04X | rs=%lu an=%lu\r\n",
               (unsigned)DB_NUM(PMA_DB_EP_ADDR_IN),
               (unsigned)s_hpcd->IN_ep[DB_NUM(PMA_DB_EP_ADDR_IN)].doublebuffer,
               (unsigned)s_hpcd->IN_ep[DB_NUM(PMA_DB_EP_ADDR_IN)].pmaaddr0,
               (unsigned)s_hpcd->IN_ep[DB_NUM(PMA_DB_EP_ADDR_IN)].pmaaddr1,
               (unsigned)ep_in,
               (unsigned)DB_NUM(PMA_DB_EP_ADDR_OUT),
               (unsigned)s_hpcd->OUT_ep[DB_NUM(PMA_DB_EP_ADDR_OUT)].doublebuffer,
               (unsigned)s_hpcd->OUT_ep[DB_NUM(PMA_DB_EP_ADDR_OUT)].pmaaddr0,
               (unsigned)s_hpcd->OUT_ep[DB_NUM(PMA_DB_EP_ADDR_OUT)].pmaaddr1,
               (unsigned)ep_out,
               (unsigned long)s_resync,
               (unsigned long)s_anom);
}

void PMA_DB_GetCounters(uint32_t *in_pkt, uint32_t *out_pkt, uint32_t *resync, uint32_t *anom)
{
    if (in_pkt != NULL)  { *in_pkt  = s_in_pkt; }
    if (out_pkt != NULL) { *out_pkt = s_out_pkt; }
    if (resync != NULL)  { *resync  = s_resync; }
    if (anom != NULL)    { *anom    = s_anom; }
}
