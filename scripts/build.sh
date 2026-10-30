#!/bin/bash
# ============================================================
# 构建整个工作区（7 个包）
#   用法：bash scripts/build.sh [工作区路径，默认 ~/apa_ws]
# ============================================================
WS=${1:-$HOME/apa_ws}
source /opt/ros/jazzy/setup.bash || exit 1
cd "$WS" || exit 1
colcon build --symlink-install \
  --packages-select parking_lot_city parking_slam prediction_interfaces \
                   detection_tracking trajectory_prediction risk_assessment \
                   scene_agents parking_planner
echo "[build] 完成，install/ 下包："
ls "$WS/install" | tr '\n' ' '
echo
