#!/usr/bin/env python3
"""碰撞风险评估节点。

订阅 /predicted_trajectories、/vehicle_pose、/target_spot（可选），在 SLAM 地图坐标系下
做时空重叠检测，发布 /risk_level（std_msgs/String: safe/warning/danger）。

策略（地图坐标系）：
- 泊车走廊：从自车当前位姿到"目标车位"的带状区域（半宽 corridor_half_width）。
  目标车位坐标来自 /target_spot（geometry_msgs/PointStamped，frame_id=map）；
  未收到时用参数 target_spot_x / target_spot_y 兜底。
  目标车位是否为空由同学 C 的空闲车位检测决定，本节点不关心、不写死。
- /predicted_trajectories 是车体（激光雷达）坐标系，先用 /vehicle_pose 的位姿
  变换到地图坐标系，再与走廊做时空重叠检测。
- 首次进入走廊的时间 ttc：<= 1s 判 danger，<= 2s 判 warning，否则 safe。
- /map 暂不直接参与走廊几何，订阅以备后续用地图可通行区域细化/校验走廊。
"""

import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PointStamped, PoseStamped

from prediction_interfaces.msg import PredictedTrajectories

CORRIDOR_HALF_WIDTH = 1.2    # 走廊半宽（米），可调
DANGER_T = 1.0               # 危险时间阈值（秒）
WARNING_T = 2.0              # 警告时间阈值（秒）


class RiskNode(Node):
    def __init__(self):
        super().__init__('risk_node')
        # 自车位姿话题可配置：默认 /vehicle_pose，对接 A 时用参数指向其实际定位话题（如 /ego_pose）
        self.declare_parameter('pose_topic', '/vehicle_pose')
        pose_topic = self.get_parameter('pose_topic').value

        self.create_subscription(PredictedTrajectories, '/predicted_trajectories',
                                 self.on_traj, 10)
        self.create_subscription(OccupancyGrid, '/map', self.on_map, 10)
        self.create_subscription(PoseStamped, pose_topic, self.on_pose, 10)
        self.create_subscription(PointStamped, '/target_spot', self.on_target, 10)
        self.pub = self.create_publisher(String, '/risk_level', 10)

        # 目标车位兜底参数（/target_spot 未发布时使用），单位：地图坐标系米
        self.declare_parameter('target_spot_x', 17.0)
        self.declare_parameter('target_spot_y', -9.75)
        self.declare_parameter('corridor_half_width', CORRIDOR_HALF_WIDTH)

        self.map_msg = None
        self.pose = None      # (x, y, yaw) 地图坐标系
        self.target = None    # (x, y) 地图坐标系
        self.trajs = None
        self.last_level = 'safe'
        self.get_logger().info('风险评估节点已启动')

    def on_map(self, msg):
        self.map_msg = msg

    def on_pose(self, msg):
        q = msg.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.pose = (msg.pose.position.x, msg.pose.position.y, yaw)
        self.evaluate()

    def on_target(self, msg):
        # 目标车位为地图坐标（frame_id 应为 map）；空位判定由 C 负责，这里直接用坐标
        self.target = (msg.point.x, msg.point.y)
        self.evaluate()

    def on_traj(self, msg):
        self.trajs = msg.trajectories
        self.evaluate()

    def _target(self):
        if self.target is not None:
            return self.target
        return (self.get_parameter('target_spot_x').value,
                self.get_parameter('target_spot_y').value)

    def evaluate(self):
        if self.pose is None:
            return
        level = 'safe'
        if self.trajs:
            ex, ey, eyaw = self.pose
            tx, ty = self._target()
            hw = self.get_parameter('corridor_half_width').value
            for tr in self.trajs:
                ttc = self._intrude_time(tr, ex, ey, eyaw, tx, ty, hw)
                if ttc is not None:
                    if ttc <= DANGER_T:
                        level = 'danger'
                        break
                    if ttc <= WARNING_T:
                        level = 'warning'

        if level != self.last_level:
            self.last_level = level
            self.get_logger().info(f'风险等级: {level}')
        msg = String()
        msg.data = level
        self.pub.publish(msg)

    @staticmethod
    def _to_map(lx, ly, ex, ey, eyaw):
        """车体（激光雷达）坐标 -> 地图坐标。"""
        c, s = math.cos(eyaw), math.sin(eyaw)
        return ex + c * lx - s * ly, ey + s * lx + c * ly

    def _intrude_time(self, tr, ex, ey, eyaw, tx, ty, hw):
        """返回目标进入自车->目标车位走廊的最早时间（秒），未进入返回 None。"""
        dt = tr.dt if tr.dt > 0.0 else 0.4
        for k in range(len(tr.future_x)):
            mx, my = self._to_map(tr.future_x[k], tr.future_y[k], ex, ey, eyaw)
            if self._point_segment_dist(mx, my, ex, ey, tx, ty) <= hw:
                return (k + 1) * dt
        return None

    @staticmethod
    def _point_segment_dist(px, py, ax, ay, bx, by):
        """点到线段 ab 的最短距离。"""
        dx, dy = bx - ax, by - ay
        l2 = dx * dx + dy * dy
        if l2 == 0.0:
            return math.hypot(px - ax, py - ay)
        t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / l2))
        return math.hypot(px - (ax + t * dx), py - (ay + t * dy))


def main(args=None):
    rclpy.init(args=args)
    node = RiskNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
