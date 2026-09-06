# 2.4G 空中鼠标与 USB 复合接收器（RX / nrfrx-usb）

基于 STM32F103C8T6 的 2.4G 空中鼠标系统**接收端**：接收发射端（TX）经 nRF24L01 发来的鼠标数据包，以 USB HID 上报主机；正在升级为 **USB MSC+HID 复合设备（U盘 + 鼠标接收器）** 并增加 SD 卡数据记录。

```
TX 发射端(MPU6050+nRF24L01)  --2.4G-->  RX 本仓库(nRF24L01) --> USB HID 主机
                                        (SPI2)               +  SD 卡日志(规划)
```

> 发射端为另一工程（其主函数参考副本见 `Reference/txmain.c`）。

## 当前状态

- ✅ **USB HID+MSC 复合设备已验证可用**（分支 `feature/composite-hid-msc`）
  - 鼠标：nRF 收包 → HID 上报，正常工作
  - U盘：Windows 正常识别，文件读写正确
  - SCSI 延迟处理已实现（SD 读写在 FreeRTOS 任务中执行，不阻塞 USB ISR）
  - 已知性能瓶颈：SD SPI 阻塞式传输，~4MB 文件约 2-3 分钟（后续可 DMA + 多块命令优化）
- 已实现：nRF 收包 → HID 鼠标上报 + SD 卡 → MSC U盘，复合描述符 57B（HID EP1 + MSC EP2）
- 已实现：FatFs + SD(SPI1) 日志链路（DATA.LOG 定长二进制记录）
- 待完成：M4 可靠性收尾（看门狗/24h 测试/功耗）；后续优化见 `接收端开发方案.md` §7

## 目录结构

```
├── App/                  ★ 应用层（用户代码，原 MDK-ARM/user 迁出）
│   ├── Src/               NRF24L01 / NRF_Demo / OLED 源文件
│   └── Inc/              对应头文件（NRF24L01/NRF_Demo/OLED）
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
- SD（已实现）：SPI1 默认映射 PA5/6/7 + CS=PB12，模块 VCC=5V；阻塞式批量 HAL_SPI 传输
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
