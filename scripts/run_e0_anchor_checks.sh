#!/bin/bash
# E0 直线解的锚定收敛验证：PLAN_AT_STAGE=1 + 放宽 ANCHOR_YAW_TOL=10
# （弧线解 4° 门限的依据是曲率余量，直线解 κ=0 无此约束，需单独验证）
source /opt/ros/jazzy/setup.bash
source ~/apa_ws/install/setup.bash
cd ~/apa_ws

echo "===== E0 锚定开启, yaw_tol=10, 不同交接偏差 ====="
for HY in 4 6 8 10; do
  echo "--- E0 handoff lat0.2 yaw ${HY}deg ---"
  PLAN_AT_STAGE=1 ANCHOR_YAW_TOL=10 \
    HANDOFF_BACK=0.1 HANDOFF_LAT=0.2 HANDOFF_YAW=$HY \
    python3 scripts/verify_park_sim.py E0
done

echo "===== E0 锚定 + 噪声 drift0.15 noise0.03 yaw0.2 多种子 ====="
for S in 1 2 3 4; do
  echo "--- E0 seed=$S handoff lat0.2 yaw8deg ---"
  PLAN_AT_STAGE=1 ANCHOR_YAW_TOL=10 \
    HANDOFF_BACK=0.1 HANDOFF_LAT=0.2 HANDOFF_YAW=8 \
    POSE_DRIFT_AMP=0.15 POSE_NOISE=0.03 POSE_YAW_NOISE=0.2 POSE_SEED=$S \
    python3 scripts/verify_park_sim.py E0
done

echo "===== W1 对照: 锚定4度门限, 交接 lat0.2 yaw6deg ====="
PLAN_AT_STAGE=1 ANCHOR_YAW_TOL=4 \
  HANDOFF_BACK=0.1 HANDOFF_LAT=0.2 HANDOFF_YAW=6 \
  python3 scripts/verify_park_sim.py W1
echo "===== W1 对照: 同上 + 噪声 ====="
for S in 1 2 3 4; do
  echo "--- W1 seed=$S ---"
  PLAN_AT_STAGE=1 ANCHOR_YAW_TOL=4 \
    HANDOFF_BACK=0.1 HANDOFF_LAT=0.2 HANDOFF_YAW=6 \
    POSE_DRIFT_AMP=0.15 POSE_NOISE=0.03 POSE_YAW_NOISE=0.2 POSE_SEED=$S \
    python3 scripts/verify_park_sim.py W1
done
