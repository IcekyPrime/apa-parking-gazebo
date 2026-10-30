#!/bin/bash

# 项目根目录（由本脚本位置推出，克隆到任意路径都可用）
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# 大 ROI（真机同款 42.5 m 窗口的等效覆盖）下重测 E2 三个 steer 档 × 残差组合
cd ~/apa_ws && source install/setup.bash
VPS=$ROOT/scripts/verify_park_sim.py
for st in 0.44 0.42 0.40; do
  for lat in 0 -0.2 0.2 -0.3 0.3; do
    for yaw in 0 4 -4; do
      out=$(PLAN_MAX_STEER=$st ROI_HALF=25 PLAN_AT_STAGE=1 \
            HANDOFF_LAT=$lat HANDOFF_YAW=$yaw \
            timeout 150 python3 "$VPS" E2 2>&1 | grep -E '^E2')
      ok=$(echo "$out" | grep -qE '剐蹭|stuck|超差' && echo FAIL || echo pass)
      reason=$(echo "$out" | awk '{print $3}')
      nsw=$(echo "$out" | awk '{print $6}')
      peak=$(echo "$out" | awk '{print $7}')
      gap=$(echo "$out" | awk '{print $11}')
      echo "steer=$st lat=$lat yaw=$yaw -> $ok [$reason nsw=$nsw] peak=$peak gap=$gap"
    done
  done
done
