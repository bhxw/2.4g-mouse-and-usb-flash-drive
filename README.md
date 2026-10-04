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
  - ~~已知瓶颈：**实测 3890KB 拷入 85 秒 = 45.8KB/s = 10.93ms/块**~~ → **✅ 已解决（2026-10-03 实测：5283KB 拷入 17.5s = 302KB/s 端到端，6.6×；设备侧 1495µs/块）**。原诊断（`sd_wait_ready` 占 76%）正确，解法是 CMD25 流式多块跨 CBW 轮次（0 RAM 增量）；实测每块忙等待从 8.26ms 降到 **2.7µs**（`wrpoll=1536=6×256`）。剩下的 1495µs/块 = SD 622µs(42%) + 块间 USB gap 818µs(55%) + 打印 26µs，gap 已逼近 USB FS ~1.2MB/s 物理墙。~~**DMA 经实测两个方向都不值得做**（对写 0.7%，读 0~1ms/块已贴 USB 墙）~~ ← **⚠ 该结论已于 2026-10-03 整条撤回并已实现 DMA**：旧算法把每字节 CPU 开销估成 143ns，`/4↔/8` 对照实验解出的实测值是 **766ns/字节**（差约 5 倍）；且"读已贴墙"所依赖的"SD 读残差 ≈0"在算术上不可能成立（那张表 1.910+0.410 = 2.320ms 已超过它自己报的 2.244ms 总时长）。实测读 sd = **914µs/块 = 62%**。详见 `项目架构.md` 已知问题 #9~#11b 与 `接收端开发方案.md` §7.1
  - 🔧 **2026-10-03 已实现（分支 `perf/sd-cmd25-stream`，Keil 编译 0 Error/0 Warning）**：
    - **CMD25 流式多块写（已硬件实测通过）**：`sd_spi.c` 新增 `SD_WriteBegin`/`SD_WriteChunk`/`SD_WriteEnd`（时序按 `参考历程/SD/utility/Sd2Card.cpp:545-644`），会话跨多次 WRITE10 保持打开；`sd_cs_low()` 内自动补发 `0xFD`，任何其它 SD 事务都能终结遗留会话。原"⚠ 收益前提未验证"已证实成立
    - **512B 数据段改走 SPI1 DMA（待硬件实测）**：`sd_dma_xfer()` 直接编程 DMA1_CH2(RX)/CH3(TX)，摘掉每字节 766ns 的 CPU 轮询。不用 HAL 的 `*_DMA` API（会接管 `hspi->State`，且要求收发缓冲区各 512B 而链接器静态只剩 208B），改用 MINC 开/关做到**零 RAM 增量**
    - **测量探针**：每块 `dbg_printf` 改为 `DWT->CYCCNT` 累加、每 256 块汇总一行 `[SD-WR/RD] n=.. sd=avg/min/max gap=.. wrpoll/tokpoll=.. bps=.. tot=..ms`；`bps` 判定流式写是否真连起来（实测 49 个写窗口中 46 个该字段整行消失 = 会话跨窗口一直开着，比 ≈256 更好），`gap` 是纯 USB 段。两条路径的每块预算都闭合到 **2% 内**
- ✅ **系统可靠性**：FreeRTOS 运行统计(CPU%)/任务栈高水位/堆水位监控；IWDG 看门狗在线（有界等待防启动卡死，长跑验证通过）
  - ⚠ **2026-10-04 订正**：CPU%/栈水位这两项此前**只有代码、没有输出**——`sysmon.c` 里两行 `dbg_puts(buf)` 是注释掉的，烧进板的镜像把表格算完就丢弃（根因是流程性的：**Keil 的 Rebuild 与 Download 是两个独立动作**）。现已打通，首次拿到 MSC 负载下的实测栈水位表（`scsi_msc` 余量 216→104 字 = 416B）。同轮修掉四处静默失败：`configCHECK_FOR_STACK_OVERFLOW` 0→2 并装 hook（此前栈溢出完全静默）、`console.c` 开始统计 `HAL_BUSY` 丢字节（`drops=` 字段）、`sysmon_start()` 上报任务创建失败、绕开 FreeRTOS 11 兼容宏把 `configSTATS_BUFFER_MAX_LENGTH=0xFFFF` 传给 384B 栈缓冲的越界隐患。详见 `开发日志.md` 2026-10-04 第五条
- ✅ **RAM 余量（2026-10-04 实测）**：链接器静态只剩 **208B**，但 FreeRTOS 堆 10240B 里还有 **4824B 未分配**（`minEver == free` ⟹ 运行期零动态分配、无碎片）→ 真实可用 **5032B**。**双缓冲重叠（方向 D）未被 RAM 阻塞**，此前各文档记的"只剩 224B 故不可行"已撤回
- ⏭ **下一步**：MSC 双缓冲重叠（写 +34%、读 +99%，推算）。串行点在 `usbd_msc_scsi.c:86`(Write) 与 `:105`(PrepareReceive) 的先后顺序
- ✅ 数据记录：FatFs(R0.16) + SD(SPI1，512B 数据段 DMA) 日志链路（DATA.LOG 定长二进制 + Python 解析）
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
├── 项目架构.md            架构/数据流/引脚/已知问题
├── mouse.ioc             CubeMX 工程
└── local/                本地目录（已忽略，不入库）：TX 端参考代码、开发脚本、
                          开发日志/方案/问答等长期文档、数据手册与抓包材料
```

## 硬件与引脚

- MCU：STM32F103C8T6（72MHz / 64KB Flash / 20KB RAM），HSE 8MHz，SWD 调试
- USB：PA11/PA12（FS 设备）
- nRF24L01：SPI2 = PB13/14/15，CE=PB0，CSN=PB1，IRQ=PB5
- SD（已实现）：SPI1 默认映射 PA5/6/7 + CS=PB12，模块 VCC=5V；单字节走裸寄存器全双工，512B 数据段走 DMA1_CH2(RX)/CH3(TX)
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

- 参考资料与本地文档**不入库**：TX 端参考代码、硬件手册、抓包数据、开发文档等统一放在仓库根 `local/`（整目录已忽略），忽略规则只写扩展名与目录名，不点名任何具体文件
- **本文与 `项目架构.md` 中引用的 `开发日志.md`、`接收端开发方案.md`、`项目讲解问答归档.md`、`tools/`、`docs/` 同样仅本地保留**（位于 `local/dev/` 下，不随仓库发布）。本仓库只发布编译所需的源码、Keil 工程与这两份说明文档；文中行号与文件名按本地目录书写，便于维护者对照，不必在仓库里寻找它们
- **文档检索（省额度）**：四个大文档合计 380KB+，禁止整篇读入 AI 上下文。用
  `python tools/doc_lookup.py find <关键词> --files` 定位行号 → `section <文件> <行号>` 取该节；
  改完文档跑 `python tools/doc_lookup.py index` 刷新 `docs/INDEX.md`。规则见 `AGENTS.md`。
| M2 | SD(SPI1) 驱动 + FatFs 日志链路(DATA.LOG) | ✅ tag m2a/m2b/m2（待硬件实测） |
| M3 | USB MSC+HID 复合设备 | ⏸ 挂起（分支 feature/usb-composite、test/msc-only；结论见开发日志） |
| M4 | 系统监控：运行统计/栈水位/链路计数 ✅；IWDG 暂禁(LSI 待实测) | ✅ m4a（待 24h+功耗） |
| M2 | SD(SPI1) 驱动 + FatFs 日志链路(DATA.LOG) | ✅ tag m2a/m2b/m2 |
| M3 | USB MSC+HID 复合（A 内联 → B 任务化） | ✅ 硬件验证通过 |
| M4 | 可靠性收尾（看门狗/24h/功耗） | 待开始 |
