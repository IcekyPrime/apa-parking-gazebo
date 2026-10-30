#!/usr/bin/env python3
"""评测记录节点：记录轨迹、风险等级变化与规划状态，收尾时打印/落盘指标。

用于技术报告的实验数据（轨迹图、风险时序、泊车精度）。
"""

import csv
import json
import math
import os
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import String


class ParkingEval(Node):
    def __init__(self):
        super().__init__('parking_eval')
        self.declare_parameter('csv', os.path.expanduser('~/parking_eval_trace.csv'))
        self.declare_parameter('rate', 5.0)
        self.declare_parameter('pose_topic', '/vehicle_pose')
        self.declare_parameter('risk_topic', '/risk_level')
        self.declare_parameter('status_topic', '/parking_status')

        self.csv_path = self.get_parameter('csv').value
        self.pose = None
        self.status = {}
        self.risk = 'safe'
        self.rows = []
        self.distance = 0.0
        self.risk_log = []
        self.t0 = time.time()
        self.writer = None
        self._last_final = None      # 终态只打印一次

        self.create_subscription(PoseStamped, self.get_parameter('pose_topic').value,
                                 self.on_pose, 20)
        self.create_subscription(String, self.get_parameter('risk_topic').value,
                                 self.on_risk, 10)
        self.create_subscription(String, self.get_parameter('status_topic').value,
                                 self.on_status, 10)

        os.makedirs(os.path.dirname(os.path.abspath(self.csv_path)), exist_ok=True)
        self._fp = open(self.csv_path, 'w', newline='', encoding='utf-8')
        self.writer = csv.writer(self._fp)
        self.writer.writerow(['t', 'x', 'y', 'yaw', 'risk', 'state', 'stall'])

        rate = float(self.get_parameter('rate').value)
        self.create_timer(1.0 / rate, self.on_timer)
        self.get_logger().info(f'评测记录：{self.csv_path}')

    def on_pose(self, msg):
        p = msg.pose.position
        q = msg.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        if self.pose is not None:
            self.distance += math.hypot(p.x - self.pose[0], p.y - self.pose[1])
        self.pose = (p.x, p.y, yaw)

    def on_risk(self, msg):
        level = msg.data.strip().lower()
        if level != self.risk:
            self.risk_log.append((round(time.time() - self.t0, 1), self.risk, level))
            self.risk = level

    def on_status(self, msg):
        try:
            self.status = json.loads(msg.data)
        except Exception:
            pass

    def on_timer(self):
        if self.pose is None:
            return
        self.writer.writerow([round(time.time() - self.t0, 1),
                              round(self.pose[0], 3), round(self.pose[1], 3),
                              round(self.pose[2], 3),
                              self.risk, self.status.get('state', ''),
                              self.status.get('stall', '')])
        self._fp.flush()
        state = self.status.get('state', '')
        if state in ('DONE', 'INCOMPLETE', 'FAILED'):
            # 只在「终态第一次出现」时打印一次：原来每个周期都打，DONE 之后
            # 会以 5 Hz 刷屏，把真正的排查信息全冲掉了。
            if state != self._last_final:
                self._last_final = state
                plan = self.status.get('plan', {})
                self.get_logger().info(
                    f'[评测] 终态={state} 车位={self.status.get("stall")} '
                    f'行驶={self.distance:.1f} m 路径={plan.get("length")} m '
                    f'误差={plan.get("err_xy")} m/{plan.get("err_yaw_deg")}° '
                    f'规划耗时={plan.get("time")} s 风险切换={self.risk_log}')
        elif state not in ('', 'DONE', 'INCOMPLETE', 'FAILED'):
            self._last_final = None


def main(args=None):
    rclpy.init(args=args)
    node = ParkingEval()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
