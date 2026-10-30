#!/bin/bash
# W1/E0 新表条目的压力验证：交接偏差 + 定位噪声（多种子）
source /opt/ros/jazzy/setup.bash
source ~/apa_ws/install/setup.bash
cd ~/apa_ws

echo "===== A. 交接偏差 back=1.0m lat=0.3m yaw=8deg ====="
for T in W1 E0; do
  echo "--- $T ---"
  HANDOFF_BACK=1.0 HANDOFF_LAT=0.3 HANDOFF_YAW=8 \
    python3 scripts/verify_park_sim.py "$T"
done

echo "===== B. 交接偏差 + 定位噪声 drift0.15 noise0.03 yaw0.2deg 多种子 ====="
for T in W1 E0; do
  for S in 1 2 3 4; do
    echo "--- $T seed=$S ---"
    HANDOFF_BACK=1.0 HANDOFF_LAT=0.3 HANDOFF_YAW=8 \
      POSE_DRIFT_AMP=0.15 POSE_NOISE=0.03 POSE_YAW_NOISE=0.2 POSE_SEED=$S \
      python3 scripts/verify_park_sim.py "$T"
  done
done
