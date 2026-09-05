/* ============================================================================
 * NRF24L01 模拟鼠标数据包收发 Demo
 *
 * TX 板: 循环发送 4 个写死的模拟鼠标数据包(数值各不相同), 并实时显示
 *        当前发出包的内容; 发送结果(TX OK/MAXRT/FAIL)显示在第 1 行
 * RX 板: 接收数据包并显示内容; 超过 2s 未收到显示 RX LOST
 *
 * 数据包结构模仿真实空中鼠标:
 *   x/y = 鼠标位移, gx/gy/gz = 陀螺仪原始值, buttons = 按键(bit0左 bit1右)
 * 以后接真实数据时, 只需把 Demo_TX_Run 里 memcpy 的来源换成 MPU6050 数据
 * ==========================================================================*/

#include <string.h>
#include "NRF_Demo.h"
#include "NRF24L01.h"
#include "oled.h"

#define TX_INTERVAL_MS      500     /* 每个包间隔 */
#define RX_LOST_TIMEOUT_MS  2000    /* 掉线判定 */


typedef char MOUSE_PKT_SIZE_CHECK[(sizeof(MousePacket_t) == 32) ? 1 : -1];

/* 4 个写死的模拟鼠标数据包(数值不同, 循环发送):
 *   序号  含义                      x    y    gx    gy    gz  buttons */

#define MOUSE_PKT_COUNT  (sizeof(g_mouse_packets) / sizeof(g_mouse_packets[0]))

static MousePacket_t g_tx_pkt;
static MousePacket_t g_rx_pkt;
static uint32_t g_last_rx_time = 0;

/* 显示一个鼠标数据包的内容(第2~4行), 第1行由调用者负责 */
static void OLED_Show_Packet(const MousePacket_t *p)
{
	/* 行2: X:+010 Y:+010 */
	OLED_ShowSignedNum(2, 3, p->x, 3);
	OLED_ShowSignedNum(2, 10, p->y, 3);

	/* 行3: L:0 R:0 GX:+010 */
	OLED_ShowChar(3, 3, (p->buttons & 0x01) ? '1' : '0');  /* 左键 */
	OLED_ShowChar(3, 7, (p->buttons & 0x02) ? '1' : '0');  /* 右键 */
	OLED_ShowSignedNum(3, 12, p->gx, 3);

	/* 行4: GY:+010 GZ:+010 */
	OLED_ShowSignedNum(4, 4, p->gy, 3);
	OLED_ShowSignedNum(4, 12, p->gz, 3);
}

/* ============================ 发送端(板A) ============================ */
#if (DEMO_ROLE == DEMO_ROLE_TX)

static void Demo_TX_Run(void)
{
	static uint8_t idx = 0;
	static uint32_t last_send = 0;
	uint8_t sta;

	if (HAL_GetTick() - last_send < TX_INTERVAL_MS)
		return;
	last_send = HAL_GetTick();

	/* 取当前模拟包(以后换成真实 MPU6050 数据即可) */
	memcpy(&g_tx_pkt, &g_mouse_packets[idx], sizeof(g_tx_pkt));

	sta = NRF24L01_TxPacket((uint8_t*)&g_tx_pkt);

	/* 行1: 发送结果 + 包号 */
	if (sta == TX_OK)       OLED_ShowString(1, 1, "TX OK   ");
	else if (sta == MAX_TX) OLED_ShowString(1, 1, "TX MAXRT"); /* 对方未开/掉线 */
	else                    OLED_ShowString(1, 1, "TX FAIL ");
	OLED_ShowNum(1, 13, idx + 1, 1);	/* "PKT:x" */

	OLED_Show_Packet(&g_tx_pkt);

	idx = (idx + 1) % MOUSE_PKT_COUNT;	/* 循环切下一个包 */
}

/* ============================ 接收端(板B) ============================ */
#elif (DEMO_ROLE == DEMO_ROLE_RX)

static void Demo_RX_Run(void)
{
	if (NRF24L01_RxPacket((uint8_t*)&g_rx_pkt) == 0)	/* 收到一包 */
	{
		g_last_rx_time = HAL_GetTick();

		OLED_ShowString(1, 1, "RX LIVE ");
		OLED_ShowNum(1, 13, g_rx_pkt.seq, 1);	/* 包号跟随发送端 */

		OLED_Show_Packet(&g_rx_pkt);
	}
	else if (HAL_GetTick() - g_last_rx_time > RX_LOST_TIMEOUT_MS)
	{
		OLED_ShowString(1, 1, "RX LOST ");	/* 超过 2s 没收到 */
	}
}

#else
#error "DEMO_ROLE 未定义或不合法"
#endif

/* ============================ 对外接口 ============================ */
void NRF_Demo_Init(void)
{
	OLED_Init();
	NRF24L01_Init();

	/* 检测 NRF24L01 是否存在(SPI 通信是否正常) */
	if (NRF24L01_Check() != 0)
	{
		OLED_ShowString(1, 1, "NRF ERR!    ");
		OLED_ShowString(2, 1, "CHECK WIRE  ");
		while (1);	/* 模块异常, 停在错误界面 */
	}

	/* 固定显示布局(第2~4行, 与 OLED_Show_Packet 位置一致) */
	OLED_ShowString(1, 1, "TX OK   PKT:1");
	OLED_ShowString(2, 1, "X:+000 Y:+000");
	OLED_ShowString(3, 1, "L:0 R:0 GX:+000");
	OLED_ShowString(4, 1, "GY:+000 GZ:+000");

#if (DEMO_ROLE == DEMO_ROLE_TX)
	TX_Mode();
	OLED_Show_Packet(&g_mouse_packets[0]);	/* 先显示第一个包 */
#else
	RX_Mode();
	OLED_ShowString(1, 1, "RX WAIT ");
#endif
}

void NRF_Demo_Run(void)
{
#if (DEMO_ROLE == DEMO_ROLE_TX)
	Demo_TX_Run();
#else
	Demo_RX_Run();
#endif
}
