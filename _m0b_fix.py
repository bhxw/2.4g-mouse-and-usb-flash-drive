# -*- coding: utf-8 -*-
"""M0b 修正：uvprojx 移除 MPU6050/freertostask 条目；文档引脚同步为最终版(SPI1=PA5-7, IRQ=PB5)"""
import re, xml.etree.ElementTree as ET

# ---- 1) uvprojx ----
p = "MDK-ARM/mouse.uvprojx"
s = open(p, encoding="utf-8").read()
names = ("MPU6050.c", "MPU6050.h", "MPU6050_Reg.h", "freertostask.c", "freertostask.h")
pat = re.compile(r"<File>\s*<FileName>(?:" + "|".join(re.escape(n) for n in names) + r")</FileName>.*?</File>", re.S)
s2, cnt = pat.subn("", s)
assert cnt == 5, f"uvprojx 应移除 5 条，实际 {cnt}"
ET.fromstring(s2)
open(p, "w", encoding="utf-8").write(s2)
print(f"uvprojx 移除 {cnt} 条 File 条目，XML 校验通过")

# ---- 2) 文档替换 ----
repls = [
    # 接收端开发方案.md
    ("接收端开发方案.md",
     "| D3 | SD 物理接口 | **SPI1 重映射（PB3/PB4/PB5）**，独立 CS；DMA1_CH2(RX)/CH3(TX)；AFIO `SWJ_CFG=NOJTAG` 释放 JTAG 脚（保留 SWD，调试器 ST-Link 不受影响） |",
     "| D3 | SD 物理接口 | **SPI1 默认映射（PA5/PA6/PA7）** + 独立 CS；DMA1_CH2(RX)/CH3(TX)；PA5 原 LED 让出 |"),
    ("接收端开发方案.md",
     "| D4 | nRF 不变 | 留在 SPI2（PB13/14/15）+ CE=PB0 + CSN=PB1；IRQ 从 PB5 **挪至 PA8**（PB5 被 SPI1_MOSI 占用） |",
     "| D4 | nRF 不变 | 留在 SPI2（PB13/14/15）+ CE=PB0 + CSN=PB1；IRQ=PB5（不变） |"),
    ("接收端开发方案.md",
     "| SD | SPI1 remap：SCK=PB3 / MISO=PB4 / MOSI=PB5 | 新占用（NOJTAG 释放 PB3/PB4） |",
     "| SD | SPI1 默认映射：SCK=PA5 / MISO=PA6 / MOSI=PA7 | 新占用（PA5 原 LED 让出） |"),
    ("接收端开发方案.md",
     "| nRF IRQ | PB5 → **PA8** | 必须挪（EXTI 输入） |",
     "| nRF IRQ | PB5（不变） | EXTI 输入 |"),
    ("接收端开发方案.md",
     "| AFIO | `SWJ_CFG=NOJTAG` | 释放 PB3/PB4（保留 SWD） |\n", ""),
    ("接收端开发方案.md",
     "- CubeMX：SPI1 remap + NOJTAG、SPI1 DMA（CH2/CH3）、SD CS=PB12、nRF IRQ=PA8、删无用外设",
     "- CubeMX：SPI1 默认映射 PA5/6/7 + DMA（CH2/CH3）✅已完成、SD CS=PB12 由驱动管理(M2)、nRF IRQ=PB5 保持、删 TIM2/I2C1/MPU6050 ✅已完成"),
    ("接收端开发方案.md",
     "已确认：PA8（nRF IRQ）、PB12（SD CS）在 `.ioc` 均未占用；SD 模块 VCC=5V（板载 LDO→3.3V + 电平转换），SPI 六线，支持 SD≤2G/SDHC≤32G，格式化 FAT16/FAT32。",
     "已确认：PB12（SD CS）空闲；PA8 无需占用（nRF IRQ 保持 PB5）；SD 模块 VCC=5V（板载 LDO→3.3V + 电平转换），SPI 六线，支持 SD≤2G/SDHC≤32G，格式化 FAT16/FAT32。"),
    ("接收端开发方案.md",
     "- [x] PA8（nRF IRQ）、PB12（SD CS）：`.ioc` 无占用，空闲确认（2026-09-05）",
     "- [x] PB12（SD CS）空闲；PA8 不再需要（nRF IRQ 保持 PB5）"),
    # 项目架构.md
    ("项目架构.md",
     "| SD 新增 | SPI1 重映射 SCK=PB3/MISO=PB4/MOSI=PB5 + CS=PB12 + DMA1_CH2/CH3 | 独立 SPI，避免与 nRF 争用 SPI2 |",
     "| SD 新增 | SPI1 默认映射 SCK=PA5/MISO=PA6/MOSI=PA7 + CS=PB12 + DMA1_CH2/CH3 | 独立 SPI，避免与 nRF 争用 SPI2（PA5 原 LED 让出） |"),
    ("项目架构.md",
     "| nRF IRQ 迁移 | PB5 → PA8 | PB5 让给 SPI1_MOSI |\n", ""),
    ("项目架构.md",
     "| AFIO | `SWJ_CFG=NOJTAG` | 释放 PB3/PB4（保留 SWD） |\n", ""),
    # README.md
    ("README.md",
     "- nRF24L01：SPI2 = PB13/14/15，CE=PB0，CSN=PB1，IRQ=PB5（规划迁 PA8）",
     "- nRF24L01：SPI2 = PB13/14/15，CE=PB0，CSN=PB1，IRQ=PB5"),
    ("README.md",
     "- SD（规划）：SPI1 重映射 PB3/4/5 + CS=PB12（需 AFIO NOJTAG），模块 VCC=5V",
     "- SD（规划）：SPI1 默认映射 PA5/6/7 + CS=PB12，模块 VCC=5V"),
]
for fname, old, new in repls:
    txt = open(fname, encoding="utf-8").read()
    c = txt.count(old)
    assert c >= 1, f"{fname}: 未找到待替换文本 -> {old[:50]}"
    open(fname, "w", encoding="utf-8").write(txt.replace(old, new))
    print(f"{fname}: 替换 {c} 处")
print("文档同步完成")
