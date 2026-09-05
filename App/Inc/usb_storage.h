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

#endif /* __USB_STORAGE_H */
