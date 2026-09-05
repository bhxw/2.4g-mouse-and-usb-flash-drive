#ifndef __NRF_DEMO_H
#define __NRF_DEMO_H

#include "main.h"

/* ================= 角色选择 =================
 * 方式一(推荐): 直接在 Keil 工程 C/C++ → Define 栏加 DEMO_ROLE=DEMO_ROLE_RX
 * 方式二: 改下面这一行
 *   烧录板A(发送端)时: #define DEMO_ROLE DEMO_ROLE_TX
 *   烧录板B(接收端)时: #define DEMO_ROLE DEMO_ROLE_RX
 */
#define DEMO_ROLE_TX  1
#define DEMO_ROLE_RX  0
#ifndef DEMO_ROLE
#define DEMO_ROLE     DEMO_ROLE_RX    /* 默认发送端, 可被编译选项覆盖 */
#endif

/* 模拟鼠标数据包: 固定 32 字节, 与 NRF24L01 载荷宽度(TX_PLOAD_WIDTH)一致
 * 结构模仿真实空中鼠标: 位移 + 陀螺原始值 + 按键 */
typedef struct {
    int8_t x;          /* 鼠标水平位移 */
    int8_t y;          /* 鼠标垂直位移 */
    int16_t gx;         /* 陀螺仪 X */
    int16_t gy;         /* 陀螺仪 Y */
    int16_t gz;         /* 陀螺仪 Z */
    uint8_t buttons;    /* 按键: bit0=左键 bit1=右键 */
    uint8_t seq;        /* 包序号 */
    uint8_t pad[22];    /* 补齐到 32 字节 */
} MousePacket_t;        /* 5*2 + 2 + 20 = 32 */

void NRF_Demo_Init(void);
void NRF_Demo_Run(void);

#endif
