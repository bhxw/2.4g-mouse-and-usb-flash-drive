/* ============================================================================
 * NRF24L01 驱动 (HAL 库移植版)
 *
 * 原代码: 正点原子 NRF24L01 驱动 (基于寄存器 SPI: SPIx_ReadWriteByte /
 *         SPIx_SetSpeed / PAout() 等)
 * 移植改动:
 *   1. 底层字节交换 SPIx_ReadWriteByte()  -> HAL_SPI_TransmitReceive()
 *   2. SPIx_SetSpeed() (改 SPI 分频)      -> 不再需要, 速度由 CubeMX 的
 *        SPI2 BaudRatePrescaler 决定(当前 256 分频约 140kHz, 在 NRF24L01
 *        10MHz 上限以内, 可放心使用; 想要更快可在 CubeMX 中改小分频)
 *   3. PAout(4) / PCout(4) / PCin(5)      -> HAL_GPIO_WritePin / ReadPin
 *   4. sys.h 的 u8                        -> stdint 的 uint8_t
 *   5. NRF24L01_Init() 中的寄存器 GPIO 配置 -> HAL_GPIO_Init()
 *   6. 发送等待 IRQ 增加超时保护(原版死等, 模块异常时会卡死, 在
 *      FreeRTOS 任务里死等会饿死其他任务)
 * ==========================================================================*/

#include "NRF24L01.h"

/* 收发地址(两端必须一致; 空中鼠标场景通常是固定收发地址的组网) */
const uint8_t TX_ADDRESS[TX_ADR_WIDTH] = {0x34,0x43,0x10,0x10,0x01}; //发送地址
const uint8_t RX_ADDRESS[RX_ADR_WIDTH] = {0x34,0x43,0x10,0x10,0x01}; //接收地址

/* 底层 SPI 全双工交换一个字节: 发送时同时接收, 返回接收到的数据 */
static uint8_t NRF24L01_SPI_ReadWriteByte(uint8_t byte)
{
	uint8_t rx = 0xFF;
	HAL_SPI_TransmitReceive(&hspi2, &byte, &rx, 1, 100);
	return rx;
}

/* 初始化 NRF24L01 的 IO 口与 CSN/CE 默认电平 */
void NRF24L01_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	/* 使能三个 GPIO 口时钟(引脚宏可能落在任意端口, 统一打开, 不影响功能) */
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();

	/* CE: 推挽输出, 初始为低 */
	GPIO_InitStruct.Pin   = NRF24L01_CE_PIN;
	GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull  = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(NRF24L01_CE_PORT, &GPIO_InitStruct);
	HAL_GPIO_WritePin(NRF24L01_CE_PORT, NRF24L01_CE_PIN, GPIO_PIN_RESET);

	/* CSN: 推挽输出, 初始为高(不选中) */
	GPIO_InitStruct.Pin   = NRF24L01_CSN_PIN;
	GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull  = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(NRF24L01_CSN_PORT, &GPIO_InitStruct);
	HAL_GPIO_WritePin(NRF24L01_CSN_PORT, NRF24L01_CSN_PIN, GPIO_PIN_SET);

	/* IRQ: 输入上拉(NRF24L01 的 IRQ 是低有效开漏输出) */
	GPIO_InitStruct.Pin  = NRF24L01_IRQ_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(NRF24L01_IRQ_PORT, &GPIO_InitStruct);

	/* SPI2 由 CubeMX 的 MX_SPI2_Init() 初始化, 这里无需重复初始化 */
}

/* 检测 24L01 是否存在
 * 返回值: 0, 成功; 1, 失败 */
uint8_t NRF24L01_Check(void)
{
	uint8_t buf[5] = {0XA5,0XA5,0XA5,0XA5,0XA5};
	uint8_t i;

	NRF24L01_Write_Buf(NRF_WRITE_REG+TX_ADDR, buf, 5); //写入5个字节的地址
	NRF24L01_Read_Buf(TX_ADDR, buf, 5);            //读出写入的地址
	for(i=0;i<5;i++)
		if(buf[i] != 0XA5)
			break;
	if(i != 5)
		return 1;   //检测24L01错误
	return 0;       //检测到24L01
}

/* SPI 写寄存器
 * reg:   指定寄存器地址
 * value: 写入的值
 * 返回值: 本次读到的状态寄存器值 */
uint8_t NRF24L01_Write_Reg(uint8_t reg, uint8_t value)
{
	uint8_t status;

	NRF24L01_CSN_LOW();                 //使能SPI传输
	status = NRF24L01_SPI_ReadWriteByte(reg);   //发送寄存器号, 同时读回状态
	NRF24L01_SPI_ReadWriteByte(value);          //写入寄存器的值
	NRF24L01_CSN_HIGH();                //禁止SPI传输
	return status;
}

/* 读取 SPI 寄存器值
 * reg: 要读的寄存器
 * 返回值: 寄存器内容 */
uint8_t NRF24L01_Read_Reg(uint8_t reg)
{
	uint8_t reg_val;

	NRF24L01_CSN_LOW();                 //使能SPI传输
	NRF24L01_SPI_ReadWriteByte(reg);    //发送寄存器号
	reg_val = NRF24L01_SPI_ReadWriteByte(0XFF); //读取寄存器内容
	NRF24L01_CSN_HIGH();                //禁止SPI传输
	return reg_val;
}

/* 在指定位置读出指定长度的数据
 * reg:   寄存器(位置)
 * pBuf:  数据指针
 * len:   数据长度
 * 返回值: 本次读到的状态寄存器值 */
uint8_t NRF24L01_Read_Buf(uint8_t reg, uint8_t *pBuf, uint8_t len)
{
	uint8_t status, u8_ctr;

	NRF24L01_CSN_LOW();                 //使能SPI传输
	status = NRF24L01_SPI_ReadWriteByte(reg);   //发送寄存器值(位置), 并读取状态值
	for(u8_ctr=0; u8_ctr<len; u8_ctr++)
		pBuf[u8_ctr] = NRF24L01_SPI_ReadWriteByte(0XFF); //读出数据
	NRF24L01_CSN_HIGH();                //关闭SPI传输
	return status;
}

/* 在指定位置写指定长度的数据
 * reg:   寄存器(位置)
 * pBuf:  数据指针
 * len:   数据长度
 * 返回值: 本次读到的状态寄存器值 */
uint8_t NRF24L01_Write_Buf(uint8_t reg, uint8_t *pBuf, uint8_t len)
{
	uint8_t status, u8_ctr;

	NRF24L01_CSN_LOW();                 //使能SPI传输
	status = NRF24L01_SPI_ReadWriteByte(reg);   //发送寄存器值(位置), 并读取状态值
	for(u8_ctr=0; u8_ctr<len; u8_ctr++)
		NRF24L01_SPI_ReadWriteByte(*pBuf++);    //写入数据
	NRF24L01_CSN_HIGH();                //关闭SPI传输
	return status;
}

/* 启动 NRF24L01 发送一次数据
 * txbuf: 待发送数据首地址 (TX_PLOAD_WIDTH=32 字节)
 * 返回值: 发送完成状况 (MAX_TX=0x10 达到最大重发 / TX_OK=0x20 发送完成 / 0xFF 超时) */
uint8_t NRF24L01_TxPacket(uint8_t *txbuf)
{
	uint8_t sta;
	uint32_t t0 = HAL_GetTick();

	NRF24L01_CE_LOW();
	NRF24L01_Write_Buf(WR_TX_PLOAD, txbuf, TX_PLOAD_WIDTH); //写数据到TX BUF 32个字节
	NRF24L01_CE_HIGH();                                     //启动发送

	/* 等待发送完成(IRQ 拉低), 增加超时保护避免死等 */
	while(NRF24L01_IRQ_READ() != 0)
	{
		if((HAL_GetTick() - t0) > NRF24L01_TX_TIMEOUT)
			return 0xff; //超时, 发送失败
	}

	sta = NRF24L01_Read_Reg(STATUS);                //读取状态寄存器的值
	NRF24L01_Write_Reg(NRF_WRITE_REG+STATUS, sta);      //清除TX_DS或MAX_RT中断标志
	if(sta & MAX_TX)                                //达到最大重发次数
	{
		NRF24L01_Write_Reg(FLUSH_TX, 0xff);     //清除TX FIFO寄存器
		return MAX_TX;
	}
	if(sta & TX_OK)                                 //发送完成
	{
		return TX_OK;
	}
	return 0xff; //其他原因发送失败
}

/* 启动 NRF24L01 接收一次数据 (轮询方式, 无需等待 IRQ)
 * rxbuf: 接收数据缓冲区 (RX_PLOAD_WIDTH=32 字节)
 * 返回值: 0, 接收完成; 1, 没收到任何数据 */
uint8_t NRF24L01_RxPacket(uint8_t *rxbuf)
{
	uint8_t sta;

	sta = NRF24L01_Read_Reg(STATUS);        //读取状态寄存器的值
	NRF24L01_Write_Reg(NRF_WRITE_REG+STATUS, sta); //清除TX_DS或MAX_RT中断标志
	if(sta & RX_OK)                         //接收到数据
	{
		NRF24L01_Read_Buf(RD_RX_PLOAD, rxbuf, RX_PLOAD_WIDTH); //读取数据
		NRF24L01_Write_Reg(FLUSH_RX, 0xff); //清除RX FIFO寄存器
		return 0;
	}
	return 1; //没收到任何数据
}

/* 初始化 NRF24L01 到 RX 模式
 * 设置RX地址, 写RX数据宽度, 选择RF频道, 波特率和LNA HCURR
 * 当 CE 变高后, 即进入 RX 模式, 并可以接收数据了 */
void RX_Mode(void)
{
	NRF24L01_CE_LOW();
	NRF24L01_Write_Buf(NRF_WRITE_REG+RX_ADDR_P0, (uint8_t*)RX_ADDRESS, RX_ADR_WIDTH); //写RX节点地址

	NRF24L01_Write_Reg(NRF_WRITE_REG+EN_AA, 0x01);     //使能通道0的自动应答
	NRF24L01_Write_Reg(NRF_WRITE_REG+EN_RXADDR, 0x01); //使能通道0的接收地址
	NRF24L01_Write_Reg(NRF_WRITE_REG+RF_CH, 40);       //设置RF通信频率
	NRF24L01_Write_Reg(NRF_WRITE_REG+RX_PW_P0, RX_PLOAD_WIDTH); //选择通道0的有效数据宽度
	NRF24L01_Write_Reg(NRF_WRITE_REG+RF_SETUP, 0x0f);  //设置TX发射参数, 0db增益, 2Mbps, 低噪声增益开启
	NRF24L01_Write_Reg(NRF_WRITE_REG+CONFIG, 0x0f);    //PWR_UP, EN_CRC, 16BIT_CRC, 接收模式
	NRF24L01_CE_HIGH(); //CE为高, 进入接收模式
}

/* 初始化 NRF24L01 到 TX 模式
 * 设置TX地址, 写TX数据宽度, 设置RX自动应答的地址, 选择RF频道, 波特率和LNA HCURR
 * PWR_UP, CRC使能
 * CE 为高大于 10us, 则启动发送 */
void TX_Mode(void)
{
	NRF24L01_CE_LOW();
	NRF24L01_Write_Buf(NRF_WRITE_REG+TX_ADDR, (uint8_t*)TX_ADDRESS, TX_ADR_WIDTH);   //写TX节点地址
	NRF24L01_Write_Buf(NRF_WRITE_REG+RX_ADDR_P0, (uint8_t*)RX_ADDRESS, RX_ADR_WIDTH); //设置TX节点地址, 主要为了使能ACK

	NRF24L01_Write_Reg(NRF_WRITE_REG+EN_AA, 0x01);     //使能通道0的自动应答
	NRF24L01_Write_Reg(NRF_WRITE_REG+EN_RXADDR, 0x01); //使能通道0的接收地址
	NRF24L01_Write_Reg(NRF_WRITE_REG+SETUP_RETR, 0x1a);//设置自动重发间隔时间:500us + 86us; 最大自动重发次数:10次
	NRF24L01_Write_Reg(NRF_WRITE_REG+RF_CH, 40);       //设置RF通道为40
	NRF24L01_Write_Reg(NRF_WRITE_REG+RF_SETUP, 0x0f);  //设置TX发射参数, 0db增益, 2Mbps, 低噪声增益开启
	NRF24L01_Write_Reg(NRF_WRITE_REG+CONFIG, 0x0e);    //PWR_UP, EN_CRC, 16BIT_CRC, 发射模式(开TX_DS中断, 用于IRQ等待)
	NRF24L01_CE_HIGH(); //CE为高, 10us后启动发送
}
