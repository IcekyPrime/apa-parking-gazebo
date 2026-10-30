#!/bin/bash
# 全车位间隙损失对比：规划最小间隙 vs 执行最小间隙
cd "$(dirname "$0")/.."
source /opt/ros/jazzy/setup.bash
source "${APA_WS:-$HOME/apa_ws}/install/setup.bash"
STALLS=${*:-"W1 W4 W5 W7 W9 E0 E2 E3 E6 E7 E8"}
for s in $STALLS; do
  out=$(timeout 150 python3 scripts/probe_e2_gap.py "$s" 2>/dev/null \
        | grep -E '规划最小|执行最小|间隙损失' | sed 's/^ *//' | tr '\n' ' ')
  echo "$s | $out"
done
