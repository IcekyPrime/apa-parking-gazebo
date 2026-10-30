#!/bin/bash
# W9 交接偏差鲁棒性对照（临时脚本，验证完可删）
source /opt/ros/jazzy/setup.bash
source ~/apa_ws/install/setup.bash
cd "$(dirname "$0")"
for cfg in "0.3 0.15 0" "0.5 0.3 6" "0.3 0.15 8" "0.8 0.4 -8"; do
  set -- $cfg
  echo "== back=$1 lat=$2 yaw=$3 =="
  HANDOFF_BACK=$1 HANDOFF_LAT=$2 HANDOFF_YAW=$3 \
    python3 verify_park_sim.py W9 2>&1 | grep -E "^W9|达标"
done
