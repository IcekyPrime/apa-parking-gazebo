#!/bin/bash

# 项目根目录（由本脚本位置推出，克隆到任意路径都可用）
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# 诊断：Windows 源码 vs WSL 工作区 vs 生成物 的同步状态
set -u
WS="$HOME/apa_ws"
WIN="$ROOT/src"

W="$WS/src/parking_lot_city/worlds/city_parking_lot.sdf"
L="$WS/src/parking_planner/config/lot_layout.yaml"

echo "===== 1. 关键文件时间戳（WSL 工作区）====="
ls -la --time-style=full-iso \
  "$WS/src/parking_lot_city/tools/build_world.py" \
  "$W" \
  "$WS/src/parking_planner/parking_planner/planner_node.py" \
  "$L" 2>&1

echo
echo "===== 2. Windows 源码 vs WSL 工作区 diff（仅统计）====="
for f in \
  "parking_lot_city/tools/build_world.py" \
  "parking_planner/parking_planner/planner_node.py" \
  "parking_planner/parking_planner/route_planner.py" \
  "parking_planner/parking_planner/lot_layout.py" \
  "parking_planner/config/route_graph.yaml" \
  "parking_planner/launch/parking_demo.launch.py" \
  "scene_agents/scene_agents/path_agent.py"; do
  if [ -f "$WIN/$f" ] && [ -f "$WS/src/$f" ]; then
    n=$(diff "$WIN/$f" "$WS/src/$f" | grep -c '^[<>]' || true)
    echo "$f : $n 行差异"
  else
    echo "$f : 文件缺失 WIN=$([ -f "$WIN/$f" ] && echo Y || echo N) WSL=$([ -f "$WS/src/$f" ] && echo Y || echo N)"
  fi
done

echo
echo "===== 3. world: sidewalk_south_road 三段位姿 ====="
grep -A1 'name="sidewalk_south_road' "$W" | grep 'pose>' | sed 's/^ *//'

echo
echo "===== 4. world: parked_b / traffic 停放车位姿 ====="
grep -B0 -A1 'name="parked_b' "$W" | grep -E 'name="parked_b|pose>' | sed 's/^ *//' | head -20

echo
echo "===== 5. world: back_stall_edge 位姿与尺寸 ====="
awk '/name="back_stall_edge"/,/<\/model>/' "$W" | grep -E 'pose>|<size>' | sed 's/^ *//'

echo
echo "===== 6. lot_layout.yaml: B 排相关条目 ====="
grep -n "parked_b\|back_stall\|name: B" "$L" | head -20

echo
echo "===== 7. lot_layout.yaml 头部（生成时间）====="
head -15 "$L"
