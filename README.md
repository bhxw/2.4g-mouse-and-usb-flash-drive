# 2.4G 空中鼠标与 USB 复合接收器（RX / nrfrx-usb）

基于 STM32F103C8T6 的 2.4G 空中鼠标系统**接收端**：接收发射端（TX）经 nRF24L01 发来的鼠标数据包，以 **USB MSC+HID 复合设备（U盘 + 鼠标接收器）** 上报主机，同时把传感器/位移数据写入 SD 卡日志。

```
TX 发射端(MPU6050+nRF24L01)  --2.4G-->  RX 本仓库(nRF24L01) --> USB HID 主机
                                        (SPI2)               +  SD 卡日志(规划)
```

> 发射端为另一工程（其主函数参考副本见 `Reference/txmain.c`）。

## 当前状态

- ✅ **USB HID+MSC 复合设备（U盘+鼠标接收器）** 已合入 main 并硬件验证
  - 鼠标：nRF 收包 → HID 上报正常；U盘：Windows 识别、文件读写正确
  - SCSI 延迟处理：SD 读写在 FreeRTOS `scsi_msc` 任务中执行，不阻塞 USB ISR
  - 复合描述符 57B（HID EP1 + MSC EP2）；MSC 介质层=整张 SD 卡
  - 已知瓶颈：SD SPI 阻塞式传输，拷贝 ~4MB 约 2-3 分钟（已实测多块/DMA 收益有限，暂缓）
- ✅ **系统可靠性**：FreeRTOS 运行统计(CPU%)/任务栈高水位/链路丢包率监控；IWDG 看门狗在线（有界等待防启动卡死，长跑验证通过）
- ✅ 数据记录：FatFs(R0.16) + SD(SPI1 直接寄存器) 日志链路（DATA.LOG 定长二进制 + Python 解析）
- 待办：24h 长跑与功耗量化（需实机）；其余优化见 `接收端开发方案.md`

## 目录结构

```
├── App/                  ★ 应用层（用户代码，原 MDK-ARM/user 迁出）
│   ├── Src/               app_main / rf·scsi·sd_log·sysmon 任务、NRF24L01/NRF_Demo/OLED、sd_spi/diskio、usbd_composite/usb_storage、rtos_api
│   └── Inc/               对应头文件
├── Core/                 CubeMX 内核（main、中断、外设初始化）
├── Drivers/              CMSIS + STM32F1xx HAL
├── Middlewares/          ST USB 设备库（HID 类）
├── USB_DEVICE/           CubeMX USB 工程文件
├── MDK-ARM/              Keil 工程（mouse.uvprojx、FreeRTOS 内核；编译产物不入库）
├── Reference/            TX 端参考代码（txmain.c）
├── 项目架构.md            架构/数据流/引脚/已知问题
├── 接收端开发方案.md      升级方案 D1~D10、里程碑 M0~M5 与验收
├── 开发日志.md            变更与排障流水（长期维护）
└── mouse.ioc             CubeMX 工程
```

## 硬件与引脚

- MCU：STM32F103C8T6（72MHz / 64KB Flash / 20KB RAM），HSE 8MHz，SWD 调试
- USB：PA11/PA12（FS 设备）
- nRF24L01：SPI2 = PB13/14/15，CE=PB0，CSN=PB1，IRQ=PB5
- SD（已实现）：SPI1 默认映射 PA5/6/7 + CS=PB12，模块 VCC=5V；SPI 直接寄存器全双工传输
- OLED：PB10/11 软件 I2C；UART1（PA9/10）printf 调试
- 详细引脚/变更见 `接收端开发方案.md` §1

## 构建

1. STM32CubeMX：打开 `mouse.ioc` 重新生成（若改外设）
2. Keil MDK-ARM：打开 `MDK-ARM/mouse.uvprojx` 编译下载
   - 注意：修改工程文件组后需在 Keil 中重新加载工程
3. 调试：ST-Link SWD

## 里程碑

| 里程碑 | 内容 | 状态 |
|---|---|---|
| M0 | 时间基准单源化 + 外设重构 + 清理 | ✅ tag m0a/m0b |
| M1 | FreeRTOS 任务化骨架 + rtos 抽象层 | ✅ tag m1 |
| M2 | SD(SPI1)+FatFs 日志链路 DATA.LOG | ✅ tag m2a/m2b/m2（硬件实测） |
| M3 | USB MSC+HID 复合设备（SCSI 任务化） | ✅ tag m3（已合入 main，硬件验证） |
| M4 | 可靠性：sysmon 监控 + IWDG 看门狗 | ✅ 编译+长跑通过（24h/功耗待实测） |
| M5 | 自研微内核替换（可选） | 待开始 |

验收标准见 `接收端开发方案.md`。

## Git 约定

- `main` 仅放通过验收的基线；里程碑完成打 tag（`m0`…`m5`）
- 大改前先提交 checkpoint；回退用 `git reset --hard <tag>`
- 提交信息：`<type>: <简述>`（feat/fix/refactor/docs/chore）
- 详见 `开发日志.md`「Git 版本管理约定」

## 说明

- `参考历程/`、个人简历文档、硬件手册等**仅本地保留，不入库**（见 `.gitignore`）
| M2 | SD(SPI1) 驱动 + FatFs 日志链路(DATA.LOG) | ✅ tag m2a/m2b/m2（待硬件实测） |
| M3 | USB MSC+HID 复合设备 | ⏸ 挂起（分支 feature/usb-composite、test/msc-only；结论见开发日志） |
| M4 | 系统监控：运行统计/栈水位/链路计数 ✅；IWDG 暂禁(LSI 待实测) | ✅ m4a（待 24h+功耗） |
| M2 | SD(SPI1) 驱动 + FatFs 日志链路(DATA.LOG) | ✅ tag m2a/m2b/m2 |
| M3 | USB MSC+HID 复合（A 内联 → B 任务化） | ✅ 硬件验证通过 |
| M4 | 可靠性收尾（看门狗/24h/功耗） | 待开始 |
