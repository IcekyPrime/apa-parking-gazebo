#!/bin/bash

# 项目根目录（由本脚本位置推出，克隆到任意路径都可用）
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# 真机测试前检查：install 生效性 + batch_real.sh 用法 + 环境清理
WS="$HOME/apa_ws"
for d in /opt/ros/*/setup.bash; do source "$d"; break; done
source "$WS/install/setup.bash" 2>/dev/null

echo "===== 1. install 空间 lot_layout.yaml 生效性 ====="
IL="$WS/install/parking_planner/share/parking_planner/config/lot_layout.yaml"
ls -la "$IL"
grep -n "parked_b" "$IL"

echo
echo "===== 2. 残留进程检查 ====="
n=$(pgrep -cf 'gz sim|gz-sim|parking_planner|scene_agents|slam_toolbox' || true)
echo "残留进程数: ${n:-0}"

echo
echo "===== 3. batch_real.sh 用法（头部 45 行）====="
head -45 "$WS/scripts/batch_real.sh" 2>/dev/null || \
  head -45 "$ROOT/scripts/batch_real.sh"
