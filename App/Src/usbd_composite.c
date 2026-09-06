/**
 * @file usbd_composite.c
 * @brief HID(Mouse) + MSC(U盘) 复合设备类包装器
 *
 * 说明：旧版 Core 仅支持单类（pdev->pClass / pClassData 各一个），
 * 本包装器在回调入口处临时切换 pClassData 指向对应子类句柄，
 * 回调返回后恢复为 HID 句柄（USBD_HID_SendReport 由任务直接调用，需默认指向 HID）。
 */
#include "usbd_composite.h"

#include "usbd_core.h"
#include "usbd_ctlreq.h"
#include "usbd_hid.h"
#include "usbd_msc.h"
#include "usb_storage.h"
#include "console.h"

/* ---------------- 复合配置描述符（FS）：9 配置 + 25 HID 接口 + 23 MSC 接口 = 57 ---------------- */
#define COMP_CFG_SIZ      57U
#define COMP_IFACE_HID    0U
#define COMP_IFACE_MSC    1U

__ALIGN_BEGIN static uint8_t s_cfg_fs[COMP_CFG_SIZ] __ALIGN_END =
{
    /* 配置描述符 */
    0x09, USB_DESC_TYPE_CONFIGURATION,
    COMP_CFG_SIZ, 0x00,
    0x02,                       /* bNumInterfaces: HID + MSC */
    0x01, 0x00,                 /* bConfigurationValue / iConfiguration */
    0xE0, 0xFA,                 /* 总线供电 + 远程唤醒 / 500mA */

    /* ---- 接口0: HID Mouse ---- */
    0x09, USB_DESC_TYPE_INTERFACE,
    0x00, 0x00, 0x01,           /* iface0 / alt0 / 1 EP */
    0x03, 0x01, 0x02, 0x00,     /* HID / Boot / Mouse / iInterface */

    0x09, 0x21,                 /* HID 描述符 */
    0x11, 0x01, 0x00, 0x01,
    0x22, 0x4A, 0x00,           /* ReportDesc: 74B */

    0x07, USB_DESC_TYPE_ENDPOINT,
    0x81, 0x03, 0x04, 0x00, 0x0A,   /* EP1 IN 中断 4B 10ms */

    /* ---- 接口1: MSC ---- */
    0x09, USB_DESC_TYPE_INTERFACE,
    0x01, 0x00, 0x02,           /* iface1 / alt0 / 2 EP */
    0x08, 0x06, 0x50, 0x00,     /* MSC / SCSI transparent / BOT / iInterface */

    0x07, USB_DESC_TYPE_ENDPOINT,
    0x02, 0x02, 0x40, 0x00, 0x00,   /* EP2 OUT Bulk 64B */

    0x07, USB_DESC_TYPE_ENDPOINT,
    0x82, 0x02, 0x40, 0x00, 0x00,   /* EP2 IN Bulk 64B */
};

static void *s_hid_h = NULL;
static void *s_msc_h = NULL;

static void set_child(USBD_HandleTypeDef *pdev, void *h)
{
    pdev->pClassData = h;
}

/* ---------------- 类回调 ---------------- */
static uint8_t comp_init(USBD_HandleTypeDef *pdev, uint8_t cfgidx)
{
    uint8_t st;

    dbg_printf("[CMP] init cfg=%u\r\n", (unsigned)cfgidx);

    /* HID 先注册并初始化（默认 pClassData = HID 句柄） */
    st = USBD_HID.Init(pdev, cfgidx);
    s_hid_h = pdev->pClassData;
    dbg_printf("[CMP] hid init st=%u\r\n", (unsigned)st);
    if (st != USBD_OK)
    {
        return USBD_FAIL;
    }

    /* MSC：注册介质层后初始化 */
    USBD_MSC_RegisterStorage(pdev, &USBD_SD_Storage_fops);
    st = USBD_MSC.Init(pdev, cfgidx);
    s_msc_h = pdev->pClassData;
    dbg_printf("[CMP] msc init st=%u\r\n", (unsigned)st);
    if (st != USBD_OK)
    {
        /* MSC 失败不拖垮 HID：继续配置，仅 MSC 不可用 */
        set_child(pdev, s_hid_h);
        return USBD_OK;
    }

    set_child(pdev, s_hid_h);   /* 默认指向 HID */
    return USBD_OK;
}

static uint8_t comp_deinit(USBD_HandleTypeDef *pdev, uint8_t cfgidx)
{
    set_child(pdev, s_hid_h);
    (void)USBD_HID.DeInit(pdev, cfgidx);
    set_child(pdev, s_msc_h);
    (void)USBD_MSC.DeInit(pdev, cfgidx);
    set_child(pdev, s_hid_h);
    return USBD_OK;
}

static uint8_t comp_setup(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req)
{
    uint8_t ret;
    int is_msc_class;

    /* MSC 类请求：GET_MAX_LUN(0xFE) / BOT_RESET(0xFF) —— 部分主机 wIndex 发 0，需按 bRequest 兜底 */
    is_msc_class = (req->bRequest == BOT_GET_MAX_LUN) || (req->bRequest == BOT_RESET);

    dbg_printf("[CMP] setup t=%02X r=%02X v=%04X i=%04X\r\n",
               (unsigned)req->bmRequest, (unsigned)req->bRequest,
               (unsigned)req->wValue, (unsigned)req->wIndex);

    if (is_msc_class || req->wIndex == COMP_IFACE_MSC)
    {
        if (s_msc_h != NULL)
        {
            set_child(pdev, s_msc_h);
            ret = USBD_MSC.Setup(pdev, req);
        }
        else
        {
            USBD_CtlError(pdev, req);
            ret = USBD_FAIL;
        }
    }
    else if (req->wIndex == COMP_IFACE_HID)
    {
        set_child(pdev, s_hid_h);
        ret = USBD_HID.Setup(pdev, req);
    }
    else
    {
        USBD_CtlError(pdev, req);
        ret = USBD_FAIL;
    }
    set_child(pdev, s_hid_h);
    return ret;
}

static uint8_t comp_data_in(USBD_HandleTypeDef *pdev, uint8_t epnum)
{
    switch (epnum & 0x7FU)
    {
    case (HID_EPIN_ADDR & 0x7FU):
        set_child(pdev, s_hid_h);
        (void)USBD_HID.DataIn(pdev, epnum);
        break;
    case (MSC_EPIN_ADDR & 0x7FU):
        set_child(pdev, s_msc_h);
        (void)USBD_MSC.DataIn(pdev, epnum);
        break;
    default:
        break;
    }
    set_child(pdev, s_hid_h);
    return USBD_OK;
}

static uint8_t comp_data_out(USBD_HandleTypeDef *pdev, uint8_t epnum)
{
    if ((epnum & 0x7FU) == (MSC_EPOUT_ADDR & 0x7FU))
    {
        set_child(pdev, s_msc_h);
        (void)USBD_MSC.DataOut(pdev, epnum);
        set_child(pdev, s_hid_h);
    }
    return USBD_OK;
}

static uint8_t *comp_get_hs_cfg_desc(uint16_t *length)
{
    *length = sizeof(s_cfg_fs);
    return s_cfg_fs;
}

static uint8_t *comp_get_fs_cfg_desc(uint16_t *length)
{
    *length = sizeof(s_cfg_fs);
    return s_cfg_fs;
}

static uint8_t *comp_get_other_cfg_desc(uint16_t *length)
{
    *length = sizeof(s_cfg_fs);
    return s_cfg_fs;
}

static uint8_t *comp_get_qualifier_desc(uint16_t *length)
{
    *length = 0U;
    return NULL;
}

USBD_ClassTypeDef USBD_Composite =
{
    comp_init,
    comp_deinit,
    comp_setup,
    NULL,   /* EP0_TxSent */
    NULL,   /* EP0_RxReady */
    comp_data_in,
    comp_data_out,
    NULL,   /* SOF */
    NULL,   /* IsoINIncomplete */
    NULL,   /* IsoOUTIncomplete */
    comp_get_hs_cfg_desc,
    comp_get_fs_cfg_desc,
    comp_get_other_cfg_desc,
    comp_get_qualifier_desc,
};
