#!/bin/bash
# 加固版硬清场：循环确认「一个模拟进程都不剩」再返回。
# 旧版只 pkill 一遍 + sleep 4：如果上一轮的 launches 还在启动途中（gz 正在
# spawn），pkill 打空，一个 ppid=1 的孤儿 gz 会留下来。两个 Gazebo 服务端
# 同时跑 = 8 GB 内存直接打爆 → 后续所有节点 exit -9（真机 P1 两次 OOM 的根因）。
# 注意：这里**绝不能** pkill -f 'batch_real.sh' —— 本脚本就是被 batch_real.sh
# 调用的，那样会把批量自己杀掉（第一次改就踩了这个坑）。
for i in 1 2 3 4 5 6 7 8; do
  pkill -9 -f 'parking_planner' 2>/dev/null
  pkill -9 -f 'scene_agents' 2>/dev/null
  pkill -9 -f 'pedestrian_agent' 2>/dev/null
  pkill -9 -f 'traffic_driver' 2>/dev/null
  pkill -9 -f 'parking_lot_city' 2>/dev/null
  pkill -9 -f 'gz sim' 2>/dev/null
  pkill -9 -f 'gz-sim' 2>/dev/null
  pkill -9 -f 'ruby.*gz' 2>/dev/null
  pkill -9 -f 'ros_gz' 2>/dev/null
  pkill -9 -f 'parameter_bridge' 2>/dev/null
  pkill -9 -f 'slam_toolbox' 2>/dev/null
  pkill -9 -f 'tf_bridge' 2>/dev/null
  pkill -9 -f 'detection_node' 2>/dev/null
  pkill -9 -f 'predictor_node' 2>/dev/null
  pkill -9 -f 'risk_node' 2>/dev/null
  pkill -9 -f 'parking_eval' 2>/dev/null
  pkill -9 -f 'slam_eval' 2>/dev/null
  sleep 2
  n=$(pgrep -cf 'gz sim|gz-sim|parking_planner|scene_agents|slam_toolbox' 2>/dev/null)
  n=${n:-0}
  [ "$n" -eq 0 ] && break
done
# 清 Fast-DDS 共享内存段（残留会让下轮节点「活着但不建话题」）
rm -f /dev/shm/fastrtps_* /dev/shm/fastdds_* 2>/dev/null
n=$(pgrep -cf 'gz sim|gz-sim|parking_planner|scene_agents|slam_toolbox' 2>/dev/null)
echo "残留进程: ${n:-0}"
free -m | sed -n 2p
