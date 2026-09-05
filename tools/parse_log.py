#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
tools/parse_log.py — 解析 SD 卡 DATA.LOG（16B 定长记录，小端）
用法:
  python tools/parse_log.py <DATA.LOG> [--csv out.csv]
输出: 记录总数 / 时间跨度 / 平均速率；可选导出 CSV。
"""
import struct
import sys

REC = struct.Struct("<HIbbbbhhh")   # magic u16 ts u32 seq,u8 but,b,x,b,y b,gx h,gy h,gz h
assert REC.size == 16


def parse(path):
    recs = []
    with open(path, "rb") as f:
        while True:
            chunk = f.read(16)
            if len(chunk) < 16:
                break
            (magic, ts, seq, but, x, y, gx, gy, gz) = REC.unpack(chunk)
            if magic != 0x4D52:
                continue   # 不同步：跳过（理论不会发生，追加式写入）
            recs.append((ts, seq, but, x, y, gx, gy, gz))
    return recs


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return 1
    recs = parse(sys.argv[1])
    if not recs:
        print("no records")
        return 0
    ts0 = recs[0][0]
    ts1 = recs[-1][0]
    span = max(ts1 - ts0, 1)
    n = len(recs)
    print(f"records   : {n}")
    print(f"timespan  : {ts0} -> {ts1} ms ({span} ms)")
    print(f"avg rate  : {n * 1000.0 / span:.1f} rec/s")

    if len(sys.argv) >= 4 and sys.argv[2] == "--csv":
        with open(sys.argv[3], "w", encoding="utf-8") as f:
            f.write("ts_ms,seq,buttons,x,y,gx,gy,gz\n")
            for (ts, seq, but, x, y, gx, gy, gz) in recs:
                f.write(f"{ts},{seq},{but},{x},{y},{gx},{gy},{gz}\n")
        print(f"csv       : {sys.argv[3]}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
