#!/usr/bin/env python3
"""干扰车 B（橙色）：与 A **同向**跑同一条全地图闭环，靠相位错开。

为什么改成同向
--------------
原来 B 是逆时针（A 顺时针），本意是让两台车在路口形成对向相遇的干扰。
但实测这条路走不通：两条环路在很多**单车道段**上是反向通行的（南侧道路、
场内过道、车道口），而"遇车让行"本质是对称的 —— 两台车互相都看到对方在
自己的前方走廊里，于是同时刹停，形成**永久死锁**。实测两台车从出生起
v=0.00 一直冻在南侧道路上相距 5 m 正面对顶，后面的红色自车和三名行人全被
堵死。只把车道口分成进出两车道解决不了问题，因为共线段到处都是。

同向的方案从结构上消除了对顶：任何共线段都退化成「跟车」，前车永远在动，
后车最多排队等待，不存在双方互等的情形。相位差由随机起点 + spawn_gap
（与其它车至少 12 m）保证，不会两台车贴在一起出生。

航点直接复用 A 的表，避免两份表各写各的、以后改一条忘另一条。
"""

from .path_agent import PathAgent
from .traffic_driver import ROUTE as ROUTE_A

# 与 A 完全相同的顺时针闭环（主干道东行 → 东街南行 → 南侧道路西行 →
# 西街北行 → 穿过三处斑马线 → 进停车场绕一圈 → 回主干道）。
ROUTE = ROUTE_A


def main(args=None):
    import rclpy
    from rclpy.executors import ExternalShutdownException
    rclpy.init(args=args)
    node = PathAgent('traffic_driver_b', ROUTE)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
