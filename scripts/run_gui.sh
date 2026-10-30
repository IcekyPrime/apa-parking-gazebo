#!/bin/bash
# ============================================================
# 带图形界面运行全链路演示（Gazebo 3D 场景 + RViz 数据视图）
#
#   bash scripts/run_gui.sh                          # Gazebo + RViz 两个窗口
#   GOAL=W5 bash scripts/run_gui.sh                  # 指定车位
#   SX=-70 bash scripts/run_gui.sh                   # 指定主路出生位置
#   RVIZ=false bash scripts/run_gui.sh               # 只看 Gazebo 3D
#   GZ=false bash scripts/run_gui.sh                 # 只看 RViz（无 Gazebo 窗口）
#   PARKER=true bash scripts/run_gui.sh              # 干扰车抢位场景
#
# 前提：WSL2 + WSLg（Windows 11，或 Windows 10 21H2 以上）。
#       在 WSL 里执行 `echo $DISPLAY`，有输出（如 :0）就说明可用。
# 本机没有 GPU 直通（/dev/dri 不存在），走的是 Mesa 软件渲染，帧率不高但够看。
# ============================================================
WS=${WS:-$HOME/apa_ws}
GOAL=${GOAL:-}
PREDICT=${PREDICT:-false}
PARKER=${PARKER:-false}
SX=${SX:--45.0}
SY=${SY:-15.65}
SYAW=${SYAW:-0.0}
RVIZ=${RVIZ:-true}
GZ=${GZ:-true}

# 有 /dev/dri 就交给 GPU，没有就显式走软件渲染（否则 OGRE 会报初始化失败）
if [ -d /dev/dri ]; then
  unset LIBGL_ALWAYS_SOFTWARE
else
  export LIBGL_ALWAYS_SOFTWARE=1
fi
export DISPLAY=${DISPLAY:-:0}

# WSL2 下反复启停会残留 Fast-DDS 共享内存，先清理再启动
bash "$(dirname "$0")/clean_env.sh" >/dev/null 2>&1

source /opt/ros/jazzy/setup.bash
source "$WS/install/setup.bash"
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export PYTHONUNBUFFERED=1
export FASTRTPS_DEFAULT_PROFILES_FILE=\
"$WS/install/parking_planner/share/parking_planner/config/fastdds_udp_only.xml"

# GZ=true  → Gazebo 图形界面（headless:=false）
# GZ=false → Gazebo 只跑服务端（headless:=true），画面交给 RViz
ARGS=("headless:=$([ "$GZ" = "true" ] && echo false || echo true)"
      "use_rviz:=$RVIZ"
      "use_prediction:=$PREDICT"
      "use_parker:=$PARKER"
      "drive_patrol:=false"
      "spawn_x:=$SX" "spawn_y:=$SY" "spawn_yaw:=$SYAW")
[ -n "$GOAL" ] && ARGS+=("goal_stall:=$GOAL")

echo "[run_gui] ros2 launch parking_planner parking_demo.launch.py ${ARGS[*]}"
echo "[run_gui] 提示：Gazebo 窗口会在约 13 s 后才出现——这是刻意的："
echo "[run_gui]       自车是运行期生成的实体，GUI 只有在「连接那一刻」取一次完整"
echo "[run_gui]       场景快照，太早连上来会看不到自车，所以等自车生成后再拉起 GUI。"
echo "[run_gui]       RViz 里 Fixed Frame 为 map，可看到地图、激光、规划路径、目标车位"
echo "[run_gui]       被操控小车是红色车（ego_vehicle），与蓝色/橙色干扰车区分"
if [ "$GZ" = "true" ]; then
  # 视角兜底：把相机锁到自车上（CameraTracking 插件的 /gui/track）。
  # 新编排下 GUI 本来就会给一个从原点看的视角，自车可能仍在画面外，所以补这一步。
  ( sleep 26; bash "$(dirname "$0")/focus_ego.sh" ) &
fi
# 位姿真值对照：记录 /vehicle_pose 与 /odom_gt，用来量化定位误差本身
# （离线能算到 0.1 m、真机却漂 1 m 的那部分，只可能来自这里）。
# 结束后分析： python3 scripts/pose_audit.py --analyze <csv>
if [ "${PAUDIT:-1}" = "1" ]; then
  PCSV=/tmp/pose_${GOAL:-auto}_$(date +%H%M%S).csv
  echo "[run_gui] 位姿对照记录 → $PCSV"
  echo "[run_gui] 结束后：python3 scripts/pose_audit.py --analyze $PCSV"
  ( python3 "$(dirname "$0")/pose_audit.py" --csv "$PCSV" >/dev/null 2>&1 ) &
fi
exec ros2 launch parking_planner parking_demo.launch.py "${ARGS[@]}"
