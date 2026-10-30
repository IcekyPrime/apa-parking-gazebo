#!/bin/bash
# ============================================================
# 检查「先验车位布局 yaml」是否与 Gazebo world 同步
#
# 为什么需要：config/lot_layout.yaml 是 make_layout.py 从 world 生成的**产物**。
# 改了 lot_layout.py 里的车位几何（位置 / 朝向 / 尺寸）却忘了重新生成 yaml，
# 运行时会继续用旧数据，表现为「目标朝向不对」「预备位姿跑进人行道」
# 「入位报 2D 栅格不连通」，非常难查（本项目实际踩过）。
#
# 用法：
#   bash scripts/check_layout.sh            # 一致返回 0，不一致返回 1
#   bash scripts/check_layout.sh --verbose  # 逐条打印每个车位
#
# 不一致时按提示重新生成：
#   cd ~/apa_ws && python3 -m parking_planner.make_layout
# ============================================================
WS=${WS:-$HOME/apa_ws}
HERE=$(cd "$(dirname "$0")" && pwd)

source /opt/ros/jazzy/setup.bash
# shellcheck disable=SC1091
source "$WS/install/setup.bash" 2>/dev/null

exec python3 "$HERE/check_layout.py" "$@"
