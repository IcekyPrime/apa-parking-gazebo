#!/bin/bash

# 项目根目录（由本脚本位置推出，克隆到任意路径都可用）
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# 1) 核实三次离线验证的目标车位确实不同
# 2) 新布局下 B 排路线段检查（check_route）
WS="$HOME/apa_ws"
for d in /opt/ros/*/setup.bash; do source "$d"; break; done
source "$WS/install/setup.bash" 2>/dev/null

echo "===== 1. 三次离线验证的目标/交接位姿对比 ====="
for st in B1 B3 B5; do
  echo "--- $st ---"
  grep -m4 -E '目标|车位|staging|交接|pose|\(-?[0-9]+\.[0-9]' "/tmp/verify_${st}.log" 2>/dev/null | head -4
done

echo
echo "===== 2. check_route 用法与 B 排路线 ====="
CR=""
for c in "$WS/scripts/check_route.py" "$WS/src/scripts/check_route.py"; do
  [ -f "$c" ] && CR="$c" && break
done
[ -z "$CR" ] && CR="$ROOT/scripts/check_route.py"
echo "脚本: $CR"
python3 "$CR" --help 2>&1 | head -10
