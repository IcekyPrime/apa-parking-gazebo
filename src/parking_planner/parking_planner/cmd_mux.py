#!/usr/bin/env python3
"""控制指令仲裁：把「同学 A 的航点驱动」与「同学 C 的泊车控制器」合到 `/cmd_vel`。

Gazebo 的阿克曼插件只有一个 `/cmd_vel` 入口，而建图巡场（A）与泊车（C）都要
驱动车辆，因此需要一个仲裁层：

1. 规划层声明 `mode=planner` 时，泊车指令优先（高级任务优先）；
2. 否则若航点驱动仍在发指令（0.5 s 内有消息），转发驱动指令；
3. 两者都没有 → 输出零速（避免上一帧指令被 Gazebo 一直保持，A 的驱动脚本
   走完路径后不再发布，若不兜底车辆会一直往前开）。
"""

import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String


class CmdMux(Node):
    def __init__(self):
        super().__init__('cmd_mux')

        self.declare_parameter('driver_topic', '/cmd_vel_driver')
        self.declare_parameter('planner_topic', '/cmd_vel_planner')
        self.declare_parameter('mode_topic', '/control_mode')
        self.declare_parameter('out_topic', '/cmd_vel')
        self.declare_parameter('driver_timeout', 0.5)
        self.declare_parameter('planner_timeout', 0.3)
        self.declare_parameter('enable_driver', True)

        gp = self.get_parameter
        self.driver_timeout = float(gp('driver_timeout').value)
        self.planner_timeout = float(gp('planner_timeout').value)
        self.enable_driver = bool(gp('enable_driver').value)

        self.mode = 'idle'
        self.driver_cmd = None
        self.driver_stamp = 0.0
        self.planner_cmd = None
        self.planner_stamp = 0.0
        self.last_source = ''

        self.create_subscription(Twist, gp('driver_topic').value, self.on_driver, 10)
        self.create_subscription(Twist, gp('planner_topic').value, self.on_planner, 10)
        self.create_subscription(String, gp('mode_topic').value, self.on_mode, 10)
        self.pub = self.create_publisher(Twist, gp('out_topic').value, 10)
        self.create_timer(0.05, self.on_timer)

        self.get_logger().info('指令仲裁已启动：driver / planner -> /cmd_vel')

    def on_driver(self, msg):
        self.driver_cmd = msg
        self.driver_stamp = time.time()

    def on_planner(self, msg):
        self.planner_cmd = msg
        self.planner_stamp = time.time()

    def on_mode(self, msg):
        self.mode = msg.data.strip().lower()

    def on_timer(self):
        now = time.time()
        source = 'stop'
        cmd = Twist()

        if self.mode == 'planner' and self.planner_cmd is not None \
                and (now - self.planner_stamp) < self.planner_timeout:
            cmd = self.planner_cmd
            source = 'planner'
        elif self.enable_driver and self.driver_cmd is not None \
                and (now - self.driver_stamp) < self.driver_timeout:
            cmd = self.driver_cmd
            source = 'driver'

        self.pub.publish(cmd)
        if source != self.last_source:
            self.get_logger().info(f'指令来源切换：{self.last_source or "-"} -> {source}')
            self.last_source = source


def main(args=None):
    rclpy.init(args=args)
    node = CmdMux()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
