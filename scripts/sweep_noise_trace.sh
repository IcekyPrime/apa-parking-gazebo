#!/bin/bash
# E2 噪声鲁棒性多 seed 扫描，逐 seed 把 TRACE_CSV 落盘（分析末段跟踪行为用）。
# 用法：bash scripts/sweep_noise_trace.sh [车位] [seed数] [trace目录]
cd "$(dirname "$0")/.."
source /opt/ros/jazzy/setup.bash
source "${APA_WS:-$HOME/apa_ws}/install/setup.bash"
export APA_WS="${APA_WS:-$HOME/apa_ws}"
export POSE_DRIFT_AMP=${POSE_DRIFT_AMP:-0.15}
export POSE_NOISE=${POSE_NOISE:-0.03}
export POSE_YAW_NOISE=${POSE_YAW_NOISE:-0.2}
STALL=${1:-E2}
N=${2:-8}
TDIR=${3:-/tmp}
mkdir -p "$TDIR"
ok=0
for s in $(seq 1 "$N"); do
  line=$(POSE_SEED="$s" TRACE_CSV="$TDIR/${STALL}_seed$s.csv" timeout 200 \
         python3 scripts/verify_park_sim.py "$STALL" 2>/dev/null | grep "^$STALL")
  echo "seed$s: ${line:-FAILED}"
  # 空输出（崩溃/超时）不算达标
  if [ -n "$line" ] && ! echo "$line" | grep -qE '剐蹭|超差|stuck'; then
    ok=$((ok+1))
  fi
done
echo "达标 $ok/$N（噪声：漂移 ${POSE_DRIFT_AMP} m / 白噪 ${POSE_NOISE} m / 航向 ${POSE_YAW_NOISE}°）"
