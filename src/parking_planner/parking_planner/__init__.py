"""泊车规划控制层（同学 C）。

模块划分：
- `lot_layout`  从 Gazebo world 解析停车位与障碍物，统一到 map 帧
- `grid`        占据栅格、膨胀、测地距离场
- `geometry`    位姿、角度、有向矩形
- `reeds_shepp` Reeds-Shepp 曲线（含前后换挡）
- `hybrid_astar` Hybrid A* 运动学搜索
- `controller`  纯跟踪 + 纵向 PID
- `planner_node` 任务状态机（选位 → 规划 → 执行 → 重规划）
"""

__version__ = "0.1.0"
