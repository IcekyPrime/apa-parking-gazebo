#!/bin/bash
# 重跑指定 seed（逗号分隔），TRACE 落盘到 /tmp/E2n_seed<seed>.csv
# 用法: bash scripts/rerun_seeds.sh 3,5 [车位]
cd "$(dirname "$0")/.."
source /opt/ros/jazzy/setup.bash
source "${APA_WS:-$HOME/apa_ws}/install/setup.bash"
export APA_WS="${APA_WS:-$HOME/apa_ws}"
export POSE_DRIFT_AMP=${POSE_DRIFT_AMP:-0.15}
export POSE_NOISE=${POSE_NOISE:-0.03}
export POSE_YAW_NOISE=${POSE_YAW_NOISE:-0.2}
STALL=${2:-E2}
IFS=',' read -ra SEEDS <<< "${1:-3,5}"
for s in "${SEEDS[@]}"; do
  line=$(POSE_SEED="$s" TRACE_CSV="/tmp/${STALL}n_seed$s.csv" timeout 200 \
         python3 scripts/verify_park_sim.py "$STALL" 2>/dev/null | grep "^$STALL")
  echo "seed$s: ${line:-FAILED}"
done
