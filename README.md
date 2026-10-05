# 2.4G 空中鼠标与 USB 复合接收器（RX / nrfrx-usb）

基于 STM32F103C8T6 的 2.4G 空中鼠标系统**接收端**：接收发射端（TX）经 nRF24L01 发来的鼠标数据包，以 **USB MSC+HID 复合设备（U盘 + 鼠标接收器）** 上报主机，同时把传感器/位移数据写入 SD 卡日志。

```
TX 发射端(MPU6050+nRF24L01)  --2.4G-->  RX 本仓库(nRF24L01) --> USB HID 主机
                                        (SPI2)               +  SD 卡日志(规划)
```

> 发射端为另一工程（其主函数参考副本见 `local/dev/Reference/txmain.c`，仅本地保留，不在仓库内）。

## 当前状态

- ✅ **USB HID+MSC 复合设备（U盘+鼠标接收器）** 已合入 main 并硬件验证
  - 鼠标：nRF 收包 → HID 上报正常；U盘：Windows 识别、文件读写正确
  - SCSI 延迟处理：SD 读写在 FreeRTOS `scsi_msc` 任务中执行，不阻塞 USB ISR
  - 复合描述符 57B（HID EP1 IN + MSC EP2 OUT / EP3 IN）；MSC 介质层=整张 SD 卡
  - ✅ **鼠标上报提速（2026-10-04）**：TX 发送周期与 RX 侧 `RF_POLL_MS` 均改为 1ms，HID 端点 `bInterval` 10ms→1ms，主机侧上报率上限 100 Hz → 1000 Hz；⚠ 已知 HID 报告在端点忙时会静默丢弃（见 `项目架构.md` #22），1kHz 下仍需加报告队列，**未实测**。
  - 🔬 **SD 传输性能（2026-10-02 ~ 10-04 逐项实测收口）**：四个杠杆叠加后**写 45.8 → 657.5 KiB/s（14.4×，61MiB 单文件口径）**、**读 13MiB 墙钟 19.36s → 16s**
    - **CMD25 流式多块写**（10-03 实测）：5283KB 拷入 85s → **17.5s（6.6×）**；每块忙等待 8.26ms → **2.7µs**（`wrpoll=1536=6×256`），会话跨多次 WRITE10 保持打开，`sd_cs_low()` 内自动补发 `0xFD` 终结遗留会话
    - **512B 数据段走 SPI1 DMA**（10-04 实测）：`sd_dma_xfer()` 直接编程 DMA1_CH2(RX)/CH3(TX)，摘掉每字节 766ns 的 CPU 轮询；**写 sd 622 → 247µs、读 sd 914 → 546µs**。不用 HAL 的 `*_DMA` API（会接管 `hspi->State`，且要求收发缓冲区各 512B 而链接器静态当时只剩 208B——该数字是 10-03 口径，**2026-10-04 实测已回到 704B**，见下「RAM 余量」），改用 MINC 开/关做到**零 RAM 增量**
    - **MSC 双缓冲流水线**（10-04 实测）：`s_buf[2]` 读写复用（堆上各 512B），写把 `PrepareReceive` 提到阻塞的 SD 写之前、读做 1 块投机预取。**写墙钟 138s → 95s（61MiB 单文件，+45.3%）**，仪器口径 +41.7%（两个口径别混）；读仪器 1086 → 619µs/块。附带坐实了此前未分解的"~230µs 主机 pacing"**其实是 NAK 引起的主机重试延迟**
    - ✅ **MSC 流水线块 512→1024B（堆上，2026-10-05 落地并实测）**：每次 USB 传输搬 **1024B = 2 个媒体包**（新宏 `MSC_STREAM_PACKET`），`s_buf[2]` 仍从 FreeRTOS 堆取 → **链接器静态 RAM 不变（RAM 18264B / 89.2%，Flash 45800B 与基线同）**；上一行记的"堆上各 512B"已加大到 1024B，堆空闲实测 **2760B（minEver 2568）**。实测（sd_log 禁用下，理由见 #17）：512KB / 512.5KB 确定性图案写入 **666 / 637.6 KB/s**（`WriteThrough+Flush(true)` 真落卡计时）、读回后**逐字节一致**；`wrpoll=1536`、`bps` 整行消失 = CMD25 流式照旧
- ✅ **MSC 端点 PMA 双缓冲（2026-10-05 三轮收口）**：HAL 的 `USE_USB_DOUBLE_BUFFER` 路径实测**两个方向都不可用**（写卡死 >200s / 读崩到卷级损坏，详情见 `项目架构.md` #23），已彻底放弃；改为**自研寄存器级 DB 层**（`USB_DEVICE/Target/usbd_pma_db.c`，蓝本 TeenyUSB 的 `tusb_dev_drv_stm32_fs.c` + 旧 ST 库 `FreeUserBuffer`）后**上板可用**：开机自证 `[DB] IN3 db=1 pma=180/1C0 R=0133 | OUT2 db=1 pma=100/140 R=3142`（`KIND` 位运行期恒为 1）；图案写 698.5 / 627.6 KB/s（基线 666 / 637.6，**持平**）、拔插后冷读 **951.6 / 959.4 KB/s（881 / 898，+7%）**、逐字节一致；10 分钟拷机 `db=rs/an=0/0`（相位强复位/异常计数恒 0）、主机 `disk` **Event 51 = 0 条**。机制口径：命令往返不由 PMA 双缓冲决定 ⇒ 上限 ~1.0MB/s、实得 +7%（理由见 `项目架构.md` #23）。验证按 SOP 在 sd_log 注掉的那一版做，交付态已恢复启用
    - **CMD18 流式多块读**（10-04 实测）：与写侧三件套同构，**读 sd 593 → 241µs（−59%）**、`tokpoll` 195 → **3 圈/块**；241µs 已贴 512B@18MHz 的纯线时 227µs（推算），即**读 SD 段触到 SPI 线速地板**
    - **测量探针**：每块 `dbg_printf` 改为 `DWT->CYCCNT` 累加、每 256 块汇总一行 `[SD-WR/RD] n=.. sd=avg/min/max gap=.. wrpoll/tokpoll=.. bps=.. tot=..ms`；两条路径的每块预算都闭合到 **2% 内**。⚠ `bps` 字段**整行消失 = 流式在正常工作**（窗口内一次都没重开会话），不是失败——曾把它判反过
  - ⚠ **当前边界（10-04 结论，别在错误方向上继续投入）**：**写 gap 431 > sd 248 → USB 侧到顶**；**读 gap 297 > sd 241 → 同样是 USB/主机侧**。SD 段只剩线速地板（写 sd 里 228µs、读 sd 里 227µs 是 18MHz 上的纯线上时间），**SPI 提频到 36MHz 在两侧都归零收益**；2048B 突发缓冲那条路已随 `tokpoll` 归零而划掉。剩余空间只能从 USB 侧要
  - ⚠ **2026-10-05 写侧二层分解（实测，口径订正）**：写速率要分两层看 —— **纯数据流持续 0.92~0.97 MB/s**（每 512B 块 555µs：`sd` 261µs + `gap` 280µs，即"等主机 ≈ 等卡"，用掉有效线时 420µs/块的 76%）；**文件级端到端 698 KB/s** 比它低的那 25% 全在**元数据窗**（512KB 图案写的 8 个 `[SD-WR]` 窗口里 3 个：`gap` 均值 6.3ms/调用、`wrpoll` 1536 → 13.4 万、`bps=18~51` 说明会话被非连续小写切断，每块 3.6ms vs 干净窗 0.52ms）。⇒ 写侧下一个方向只能是元数据段（"CMD25 会话重开" vs "主机命令节奏"，先加仪表把两者分开），**SD 段与 PMA 双缓冲都不是瓶颈**（后者写侧实测 0% 收益）。计划与判据见本地保留的《第二层写路径优化计划书》；MSC 走裸块、卡上的 FAT 由主机维护，设备侧 FatFs 调参对本问题无效
  - ⚠ **评估流式性能必须用单个大文件**：多文件拷贝另算——6 文件 30.6MiB / 61s = **513.7 KiB/s**，比同固件单文件的 657.5 低 22%，多出的是每文件 FAT 链分配与目录写；非连续小写会打断 CMD25 会话（`usb_storage.c` 退回 `SD_WriteBegin` 重开）
- ✅ **系统可靠性**：FreeRTOS 运行统计(CPU%)/任务栈高水位/堆水位监控；IWDG 看门狗在线（有界等待防启动卡死，长跑验证通过）
  - ⚠ **2026-10-04 订正**：CPU%/栈水位这两项此前**只有代码、没有输出**——`sysmon.c` 里两行 `dbg_puts(buf)` 是注释掉的，烧进板的镜像把表格算完就丢弃（根因是流程性的：**Keil 的 Rebuild 与 Download 是两个独立动作**）。现已打通，首次拿到 MSC 负载下的实测栈水位表（`scsi_msc` 余量 216→104 字 = 416B）。同轮修掉四处静默失败：`configCHECK_FOR_STACK_OVERFLOW` 0→2 并装 hook（此前栈溢出完全静默）、`console.c` 开始统计 `HAL_BUSY` 丢字节（`drops=` 字段）、`sysmon_start()` 上报任务创建失败、绕开 FreeRTOS 11 兼容宏把 `configSTATS_BUFFER_MAX_LENGTH=0xFFFF` 传给 384B 栈缓冲的越界隐患。详见 `开发日志.md` 2026-10-04 第五条
- ✅ **RAM 余量（2026-10-04 实测）**：链接器静态余量 **704B**（2026-10-04，`MDK-ARM/mouse/mouse.map` 实测 RW_IRAM1 = 19776/20480；**208B 是 2026-10-03 的旧口径**），但 FreeRTOS 堆 10240B 里还有 **4824B 未分配**（`minEver == free` ⟹ 运行期零动态分配、无碎片）→ 真实可用 **5032B**（2026-10-05 更新：MSC 流水线块加到 1024B 后，堆空闲**实测 2760B / minEver 2568**——此前按 4824B 推算的 ≈3800B 偏乐观，差的 ~1KB 是日志队列 64×16B 当时尚未分配，见 `项目架构.md` #23/#24）。**双缓冲（+1024B）已按此落地**，此前各文档记的"只剩 224B 故不可行"已撤回
- ⏭ **下一步**：性能阶段关闭，转**鲁棒性/错误路径**。已按当前代码核实的两条 P0：`SCSI_ProcessCmd` 各 case **丢弃子处理器返回值**（只有 `default` 返回 -1，卡未就绪/越界/`dDataLength` 非法都走成功路径 → CSW PASSED + 垃圾数据，是错误处理的总病根）、`USBD_MSC_Init` 先赋 `s_hmsc` 后查 NULL 且句柄不 memset（重枚举继承 stale 状态）。另见既有条目 `项目架构.md` #12~#18
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
