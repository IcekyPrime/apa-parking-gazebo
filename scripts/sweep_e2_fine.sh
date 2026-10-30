#!/bin/bash

# 项目根目录（由本脚本位置推出，克隆到任意路径都可用）
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# E2 鲁棒边界细扫：0.41~0.43 × 横向 ±0.1/0.2/0.3 × 航向 0/±4°
cd ~/apa_ws && source install/setup.bash
VPS=$ROOT/scripts/verify_park_sim.py
for st in 0.43 0.42 0.41; do
  for lat in 0.1 -0.1 0.2 -0.2 0.3 -0.3; do
    for yaw in 0 4 -4; do
      out=$(PLAN_MAX_STEER=$st PLAN_AT_STAGE=1 HANDOFF_LAT=$lat HANDOFF_YAW=$yaw \
            timeout 150 python3 "$VPS" E2 2>&1 | grep -E '^E2')
      ok=$(echo "$out" | grep -qE '剐蹭|stuck|超差' && echo FAIL || echo pass)
      peak=$(echo "$out" | awk '{print $7}')
      gap=$(echo "$out" | awk '{print $11}')
      echo "$st lat=$lat yaw=$yaw -> $ok peak=$peak gap=$gap"
    done
  done
done
