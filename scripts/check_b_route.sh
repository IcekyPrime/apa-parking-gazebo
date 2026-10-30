#!/bin/bash

# 项目根目录（由本脚本位置推出，克隆到任意路径都可用）
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WS="$HOME/apa_ws"
for d in /opt/ros/*/setup.bash; do source "$d"; break; done
source "$WS/install/setup.bash" 2>/dev/null

echo "===== 1. 三个离线验证日志开头（确认目标车位）====="
for st in B1 B3 B5; do
  echo "--- $st ---"
  head -8 "/tmp/verify_${st}.log"
done

echo
echo "===== 2. 新布局下 B 排路线（check_route 全量 → 只看 B 行）====="
python3 "$ROOT/scripts/check_route.py" > /tmp/route_all.log 2>&1
echo "退出码 $?"
grep -E '\-> B[0-9]' /tmp/route_all.log
echo "--- 异常行（穿墙/FAIL/无路线）---"
grep -E '穿墙点 [1-9]|FAIL|无路线|不可达' /tmp/route_all.log || echo "（无异常）"
