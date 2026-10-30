#!/bin/bash

# 项目根目录（由本脚本位置推出，克隆到任意路径都可用）
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# E2 稳定性扫描：PLAN_MAX_STEER × 双向交接偏差，找「两侧偏差都不发散」的曲率上限。
cd ~/apa_ws && source install/setup.bash
VPS=$ROOT/scripts/verify_park_sim.py
for st in 0.44 0.42 0.40 0.38; do
  for lat in 0.2 -0.2; do
    out=$(PLAN_MAX_STEER=$st PLAN_AT_STAGE=1 HANDOFF_LAT=$lat \
          timeout 150 python3 "$VPS" E2 2>&1 | grep -E '^E2')
    echo "steer=$st lat=$lat | $out"
  done
done
