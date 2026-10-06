#ifndef __USB_STORAGE_H
#define __USB_STORAGE_H

#include "usbd_msc.h"

/**
 * @file usb_storage.h
 * @brief MSC 介质层：整张 SD 卡作为 LUN0（块大小 512B）。
 *        注意：与 sd_log 任务共享 SD，使用方需保证单写者（主机枚举期间暂停本地日志）。
 */
extern USBD_StorageTypeDef USBD_SD_Storage_fops;

/** 记录最近一次 MSC 介质访问时刻（主机读/写/容量查询都会触发） */
void usb_storage_ping(void);
uint32_t usb_storage_last_active_ms(void);

/** 主程序调用（非ISR）：预初始化 SD 并缓存容量，避免 usbstor 在中断里读 SD */
void usb_storage_preinit(void);

/** 创建 SCSI 延迟处理信号队列（在调度器启动前调用） */
void usb_storage_msc_task_init(void);

/**
 * @brief MSC 介质入口流量快照（诊断用，只读不清零）
 *
 * 为什么需要它：现有 `[SD-RD]`/`[SD-WR]` 是"累计成功满 256 块才打一行"的窗口汇总，
 * 且 `usb_storage.c` 里每一处失败分支都排在 `stat_add` **之前** return —— 主机反复重试
 * 失败命令时设备侧一个块都不记，串口表现与"主机根本没发"一模一样。这四个数把三态分开：
 *   calls=0                          → 主机没发，问题在主机侧
 *   calls 涨、blocks 不涨、fails 涨   → 设备在回错、主机在重试退避
 *   blocks 涨但很慢                   → 真是卡慢
 * jumps 是"起始块号不接上一次结尾"的次数：≈calls 说明主机在打散块（FAT/目录项），
 * ≈0 说明在顺序流。
 *
 * 计数点在 `sd_storage_read_timed` / `sd_storage_write_timed`（`USBD_SD_Storage_fops`
 * 实际指向的两个包装），所以每一次调用、每一次失败都必然进账，不必在失败分支上逐处埋点。
 */
typedef struct
{
    uint32_t rd_calls;   /* READ10 调用次数 */
    uint32_t rd_blocks;  /* 请求块数合计 */
    uint32_t rd_fails;   /* 返回非 0 的次数 */
    uint32_t rd_jumps;   /* 起始块号不与上次读结尾相接的次数 */
    uint32_t wr_calls;   /* WRITE10 调用次数 */
    uint32_t wr_blocks;  /* 请求块数合计 */
    uint32_t wr_fails;   /* 返回非 0 的次数 */
    uint32_t wr_jumps;   /* 起始块号不与上次写结尾相接的次数 */
} msc_traffic_t;

/** 取一份 MSC 入口流量快照（纯读，不清零；调用方自己存上一份算差值） */
void usb_storage_traffic(msc_traffic_t *t);

#endif /* __USB_STORAGE_H */
