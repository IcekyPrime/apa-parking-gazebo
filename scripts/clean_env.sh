#!/bin/bash
# ============================================================
# 彻底清理 ROS / Gazebo 残留进程与 Fast-DDS 共享内存段
#
# 用途：WSL2 下反复启动/杀死仿真后，Fast-DDS 的 SHM 残留会让新进程卡死在
# rcl_node_init（进程活着但一个 topic 都不创建）。每次重新跑演示前执行本脚本。
# ============================================================
pkill -9 -f 'gz sim' 2>/dev/null
pkill -9 -f 'gz-sim' 2>/dev/null
pkill -9 -f parameter_bridge 2>/dev/null
pkill -9 -f slam_toolbox 2>/dev/null
pkill -9 -f tf_bridge 2>/dev/null
pkill -9 -f demo_driver 2>/dev/null
pkill -9 -f planner_node 2>/dev/null
pkill -9 -f cmd_mux 2>/dev/null
pkill -9 -f parking_eval 2>/dev/null
pkill -9 -f detection_node 2>/dev/null
pkill -9 -f predictor_node 2>/dev/null
pkill -9 -f risk_node 2>/dev/null
pkill -9 -f 'ros2 topic' 2>/dev/null
pkill -9 -f 'ros2 launch' 2>/dev/null
pkill -9 -f 'ros2-daemon' 2>/dev/null
sleep 1
rm -f /dev/shm/fastrtps_* 2>/dev/null || sudo rm -f /dev/shm/fastrtps_* 2>/dev/null
echo "[clean_env] 剩余相关进程："
ps -eo pid,etime,cmd | grep -E "gz sim|planner_node|cmd_mux|slam_toolbox|tf_bridge|demo_driver|ros2 (launch|topic)" \
  | grep -v grep || echo "  （无）"
echo "[clean_env] 剩余 fastrtps 共享内存段：$(ls /dev/shm 2>/dev/null | grep -c fastrtps)"
