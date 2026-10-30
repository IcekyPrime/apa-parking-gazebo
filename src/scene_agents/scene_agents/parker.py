#!/usr/bin/env python3
"""干扰车 B：从主路匀速驶入停车场，停进指定车位。

用途：制造「目标车位被抢占 / 泊车走廊被动态占用」的场景，检验自车的
风险联动（减速 / 停车等待 / 重新选位）与重规划能力。

默认停进东排 E8（世界坐标 (17.0, -2.25)，车头朝 +x）。路线全程在车道与
停车场过道上，不穿越人行道。话题（`/traffic2/odom_gt`、`/traffic2/cmd_vel`）
由 launch 以参数传入，与干扰车 A 完全隔离。
"""

from .path_agent import PathAgent

# 世界坐标：主路东行 → 入口连接段 → 场内北过道 → 东排过道 → 驶入 E8
ROUTE = [
    (-30.0, 18.5),
    (-8.0, 18.5),
    (0.0, 14.0),
    (0.0, 7.0),
    (0.0, 3.0),
    (7.0, 2.0),
    (8.6, -2.25),
    (12.0, -2.25),
    (17.0, -2.25),
]


def main(args=None):
    import rclpy
    from rclpy.executors import ExternalShutdownException
    rclpy.init(args=args)
    node = PathAgent('parker', ROUTE)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
