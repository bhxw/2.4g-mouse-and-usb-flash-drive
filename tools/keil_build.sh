#!/usr/bin/env bash
# tools/keil_build.sh — 用 Keil uVision 命令行编译当前工程并汇总结果
# 用法:
#   bash tools/keil_build.sh            # 增量编译(Build)
#   bash tools/keil_build.sh rebuild    # 全量编译(Rebuild)
# 依赖: Keil MDK 已安装; 可用环境变量 UV4_EXE 覆盖默认路径
set -u

UV4EXE="${UV4_EXE:-/c/app/keil/UV4/UV4.exe}"
PRJ="MDK-ARM/mouse.uvprojx"
LOG="build_cli.log"

if [ ! -f "$UV4EXE" ]; then
  echo "[keil] UV4 不存在: $UV4EXE"
  exit 2
fi
if [ ! -f "$PRJ" ]; then
  echo "[keil] 工程不存在: $PRJ (请在项目根目录运行)"
  exit 2
fi

UV4="$(cygpath -w "$UV4EXE")"
PRJW="$(cygpath -w "$(pwd)/$PRJ")"
LOGW="$(cygpath -w "$(pwd)/$LOG")"

MODE="-b"
[ "${1:-}" = "rebuild" ] && MODE="-r"

rm -f "$LOG"
"$UV4" $MODE "$PRJW" -o "$LOGW" -j0 >/dev/null 2>&1
rc=$?

echo "[keil] exit=$rc mode=$MODE"
if [ -f "$LOG" ]; then
  echo "[keil] 统计:"
  grep -aE "[0-9]+ Error\(s\)|[0-9]+ Warning\(s\)" "$LOG" | tail -2
  echo "[keil] 报错明细(若有):"
  ERR=$(grep -acE ": error:|: fatal error:|Error: " "$LOG" || true)
  if [ "$ERR" -gt 0 ]; then
    grep -aE ": error:|: fatal error:|Error: " "$LOG" | head -40
  else
    echo "(无)"
  fi
else
  echo "[keil] 未生成日志（可能 Keil GUI 正占用工程，请关闭 uVision 后重试）"
  exit 2
fi
# UV4 退出码: 0=完全成功, 1=有警告, >1=有错误
if grep -aq "0 Error(s)" "$LOG"; then
  exit 0
fi
exit 1
