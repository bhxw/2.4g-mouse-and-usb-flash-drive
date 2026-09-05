#ifndef __LOG_FORMAT_H
#define __LOG_FORMAT_H

#include <stdint.h>

/**
 * @file log_format.h
 * @brief 落盘日志记录格式 v1：定长 16 字节，小端序（与 tools/parse_log.py 对应）
 *
 *  偏移 字段     类型   说明
 *  0    magic    u16    LOG_MAGIC(0x4D52)
 *  2    ts_ms    u32    HAL_GetTick（开机毫秒）
 *  6    seq      u8     TX 包序号
 *  7    buttons  u8     bit0 左键 / bit1 右键
 *  8    x        i8     鼠标位移
 *  9    y        i8
 *  10   gx       i16    陀螺原始值
 *  12   gy       i16
 *  14   gz       i16
 */
#define LOG_MAGIC       0x4D52u

#pragma pack(push, 1)
typedef struct {
    uint16_t magic;
    uint32_t ts_ms;
    uint8_t  seq;
    uint8_t  buttons;
    int8_t   x;
    int8_t   y;
    int16_t  gx;
    int16_t  gy;
    int16_t  gz;
} LogRecord;   /* 2+4+1+1+1+1+2*3 = 16 字节（pack(1) 必须，否则 ARMCC 会插入 2 字节对齐填充） */
#pragma pack(pop)

/* 编译期断言：记录必须严格 16 字节，与 tools/parse_log.py 一致 */
typedef char log_record_size_check[(sizeof(LogRecord) == 16) ? 1 : -1];

#endif /* __LOG_FORMAT_H */
