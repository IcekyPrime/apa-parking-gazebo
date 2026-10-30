#!/bin/bash
# 台架转向标定：空世界 + 自车 + ros_gz_bridge → 跑 bench_steer.py
#
#   bash scripts/bench_steer.sh
#
# 为什么要空世界：标定要跑 15 组「恒定曲率绕圈」，半径最小 4.3 m，
# 城市世界里必然撞到东西。空世界没有传感器渲染开销，物理步长也更稳。
WS=${WS:-$HOME/apa_ws}
[ -f "$WS/install/setup.bash" ] || WS=$HOME/apa_ws
HERE=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
B=${B:-/tmp/apa_bench}

source /opt/ros/jazzy/setup.bash
# shellcheck disable=SC1090
source "$WS/install/setup.bash"
export GZ_SIM_RESOURCE_PATH="$WS/src/parking_lot_city/models"
# [修复] 不清理 Fast-DDS SHM 残留时，新起的 python 节点会卡在发现阶段，
# 收不到 /odom_gt（3 s 等待直接超时退出）。与 run_gui.sh 同一套清理 + 传输配置。
bash "$HERE/clean_env.sh" >/dev/null 2>&1
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export PYTHONUNBUFFERED=1

mkdir -p "$B"
cat > "$B/empty.sdf" <<'SDF'
<?xml version="1.0" ?>
<sdf version="1.9">
  <world name="bench">
    <physics name="1ms" type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <model name="ground">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>600 600</size></plane></geometry>
          <surface><friction><ode><mu>1.3</mu><mu2>1.3</mu2></ode></friction></surface>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>600 600</size></plane></geometry>
          <material><ambient>0.55 0.57 0.60 1</ambient><diffuse>0.55 0.57 0.60 1</diffuse></material>
        </visual>
      </link>
    </model>
  </world>
</sdf>
SDF

cat > "$B/bridge.yaml" <<'YML'
- ros_topic_name: "/clock"
  gz_topic_name: "/clock"
  ros_type_name: "rosgraph_msgs/msg/Clock"
  gz_type_name: "gz.msgs.Clock"
  direction: GZ_TO_ROS
- ros_topic_name: "/cmd_vel"
  gz_topic_name: "/cmd_vel"
  ros_type_name: "geometry_msgs/msg/Twist"
  gz_type_name: "gz.msgs.Twist"
  direction: ROS_TO_GZ
- ros_topic_name: "/odom_gt"
  gz_topic_name: "/odom_gt"
  ros_type_name: "nav_msgs/msg/Odometry"
  gz_type_name: "gz.msgs.Odometry"
  direction: GZ_TO_ROS
YML

pkill -f 'gz sim' >/dev/null 2>&1
pkill -f parameter_bridge >/dev/null 2>&1
sleep 1

gz sim -s -r -v 2 "$B/empty.sdf" > "$B/gz.log" 2>&1 &
GZPID=$!
sleep 5
ros2 run ros_gz_bridge parameter_bridge \
  --ros-args -p config_file:="$B/bridge.yaml" > "$B/bridge.log" 2>&1 &
BRPID=$!
sleep 3
ros2 run ros_gz_sim create -name ego_vehicle \
  -file "$WS/src/parking_lot_city/models/ego_car/model.sdf" \
  -x 0 -y 0 -z 0.03 -Y 0 > "$B/create.log" 2>&1
# 实体是后插入的，插件加载 + 桥接发现都要时间；10 s 是实测稳妥值
sleep 10

python3 "$HERE/bench_steer.py"
RC=$?

kill -INT "$BRPID" "$GZPID" >/dev/null 2>&1
sleep 2
pkill -f 'gz sim' >/dev/null 2>&1
pkill -f parameter_bridge >/dev/null 2>&1
exit $RC
