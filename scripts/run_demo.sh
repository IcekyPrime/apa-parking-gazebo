#!/bin/bash
# ============================================================
# 一键运行全链路演示（无界面，可用于课堂录制/回归测试）
#
#   bash scripts/run_demo.sh                 # 默认：主路出发，自动选位泊车
#   PATROL=false bash scripts/run_demo.sh    # 跳过巡场，直接选位泊车
#   GOAL=W4      bash scripts/run_demo.sh    # 指定车位（场内 W0-W9/E0-E9/P0-P3、
#                                            #   南侧道路 B0-B5、西街 SW0-SW5、东街 SE0-SE5）
#   SX=-70 SY=15.65 bash scripts/run_demo.sh # 指定主路上的出生位置（任意位置都行）
#   SYAW=3.14 SX=60 SY=24.35 bash scripts/run_demo.sh   # 从西行车道出发（会自动绕行）
#   PARKER=true  bash scripts/run_demo.sh    # 干扰车恒速抢停 E8，测试动态避障/改选
#   PREDICT=false bash scripts/run_demo.sh   # 关闭 B 的预测/风险评估
#
# 需要图形界面时把 HEADLESS 设为 false，并保证 DISPLAY 可用（WSLg）
# ============================================================
WS=${WS:-$HOME/apa_ws}
HEADLESS=${HEADLESS:-true}
RVIZ=${RVIZ:-false}
PATROL=${PATROL:-false}
PREDICT=${PREDICT:-true}
PARKER=${PARKER:-false}
GOAL=${GOAL:-}
SX=${SX:--45.0}
SY=${SY:-15.65}
SYAW=${SYAW:-0.0}

source /opt/ros/jazzy/setup.bash
source "$WS/install/setup.bash"
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export LIBGL_ALWAYS_SOFTWARE=1
export DISPLAY=${DISPLAY:-:0}
export PYTHONUNBUFFERED=1
# 关键：关闭 Fast-DDS 共享内存传输，改用 UDP 回环（WSL2 必需）
export FASTRTPS_DEFAULT_PROFILES_FILE=\
"$WS/install/parking_planner/share/parking_planner/config/fastdds_udp_only.xml"

ARGS=("headless:=$HEADLESS" "use_rviz:=$RVIZ" "use_prediction:=$PREDICT"
      "drive_patrol:=$PATROL" "use_parker:=$PARKER"
      "spawn_x:=$SX" "spawn_y:=$SY" "spawn_yaw:=$SYAW")
[ -n "$GOAL" ] && ARGS+=("goal_stall:=$GOAL")

echo "[run_demo] ros2 launch parking_planner parking_demo.launch.py ${ARGS[*]}"
ros2 launch parking_planner parking_demo.launch.py "${ARGS[@]}"
