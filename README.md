# 2.4G 空中鼠标与 USB 复合接收器（RX / nrfrx-usb）

基于 STM32F103C8T6 的 2.4G 空中鼠标系统**接收端**：接收发射端（TX）经 nRF24L01 发来的鼠标数据包，以 USB HID 上报主机；正在升级为 **USB MSC+HID 复合设备（U盘 + 鼠标接收器）** 并增加 SD 卡数据记录。

```
TX 发射端(MPU6050+nRF24L01)  --2.4G-->  RX 本仓库(nRF24L01) --> USB HID 主机
                                        (SPI2)               +  SD 卡日志(规划)
```

> 发射端为另一工程（其主函数参考副本见 `Reference/txmain.c`）。

## 当前状态

- 功能原型可运行：nRF 收包 → USB HID 鼠标上报
- 升级进行中：USB MSC+HID 复合、FatFs + SD(SPI1+DMA) 日志、FreeRTOS 任务化 → 见里程碑
- 代码评审遗留问题与修复归属：见 `项目架构.md` §7 与 `开发日志.md`

## 目录结构

```
├── App/                  ★ 应用层（用户代码，原 MDK-ARM/user 迁出）
│   ├── Src/               NRF24L01 / NRF_Demo / freertostask / MPU6050 / OLED 源文件
│   └── Inc/              对应头文件
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
- nRF24L01：SPI2 = PB13/14/15，CE=PB0，CSN=PB1，IRQ=PB5（规划迁 PA8）
- SD（规划）：SPI1 重映射 PB3/4/5 + CS=PB12（需 AFIO NOJTAG），模块 VCC=5V
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
| M0 | 引脚重构（SPI1+NOJTAG、nRF IRQ→PA8、SD CS=PB12）+ 时间基准修正 + 死代码清理 | 待开始 |
| M1 | FreeRTOS 任务化骨架 + rtos 抽象层 | 待开始 |
| M2 | SD(SPI1+DMA) + FatFs 日志 | 待开始 |
| M3 | USB MSC+HID 复合（A 内联 → B 任务化） | 待开始 |
| M4 | 可靠性收尾（看门狗/24h/功耗） | 待开始 |
| M5 | 自研微内核替换（可选） | 待开始 |

验收标准见 `接收端开发方案.md`。

## Git 约定

- `main` 仅放通过验收的基线；里程碑完成打 tag（`m0`…`m5`）
- 大改前先提交 checkpoint；回退用 `git reset --hard <tag>`
- 提交信息：`<type>: <简述>`（feat/fix/refactor/docs/chore）
- 详见 `开发日志.md`「Git 版本管理约定」

## 说明

- `参考历程/`、个人简历文档、硬件手册等**仅本地保留，不入库**（见 `.gitignore`）
