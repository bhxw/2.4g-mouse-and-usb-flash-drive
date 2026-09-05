#ifndef __USBD_COMPOSITE_H
#define __USBD_COMPOSITE_H

#include "usbd_def.h"

/**
 * @file usbd_composite.h
 * @brief HID(Mouse) + MSC(U盘) 复合设备类（老版单类 Core 上的包装器）
 *
 * 原理：注册一个“复合类”对象，Host 拿到的配置描述符包含两个接口——
 *   接口 0 = HID（EP IN 0x81）
 *   接口 1 = MSC（EP OUT 0x02 / EP IN 0x82）
 * EP0 的类/接口请求按 wIndex 路由给 HID 或 MSC；DataIn/DataOut 按 EP 路由。
 */
extern USBD_ClassTypeDef USBD_Composite;

#endif /* __USBD_COMPOSITE_H */
