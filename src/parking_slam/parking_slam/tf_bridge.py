#!/usr/bin/env python3
"""TF / 位姿桥接节点。

作用（把开源 slam_toolbox 与整车其他模块接起来）：
1. 订阅 `/odom`（Gazebo 阿克曼插件输出的底盘里程计），定时发布 `odom -> base_footprint`；
2. 用 `/tf_static` 发布静态外参 `base_footprint -> lidar_link`；
3. 查询 `map -> base_footprint`，以 `geometry_msgs/PoseStamped`（frame_id=map）
   持续发布 `/vehicle_pose`，供预测层与规划层使用。

说明：slam_toolbox 只负责 `map -> odom` 的校正量，`odom -> base` 由本节点提供，
两者复合即得到车辆在地图坐标系下的位姿。

[同学 C 修复记录]
原实现把 `base_footprint -> lidar_link` 与 `odom -> base_footprint` 一起，
只在收到 `/odom` 回调时发布。而 Gazebo 的 AckermannSteering 在车辆静止时不发
`/odom`，于是：
  (a) 车辆静止期间 TF 时间轴停止，slam_toolbox 的 message_filter 查不到扫描时刻
      的 lidar 变换，持续打印 "Message Filter dropping message ... queue is full"，
      无法建图；
  (b) 静止起步的最初几秒同样缺变换。
修复方式：静态外参改走 `/tf_static`（对所有时间有效），`odom -> base` 改为
按固定频率（默认 20 Hz）用当前时钟重发最近一次里程计位姿。
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster, Buffer, \
    TransformListener, LookupException, ConnectivityException, ExtrapolationException


def yaw_to_quat(yaw):
    return math.sin(yaw * 0.5), math.cos(yaw * 0.5)


class TfBridge(Node):
    def __init__(self):
        super().__init__('slam_tf_bridge')

        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('pose_topic', '/vehicle_pose')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('lidar_frame', 'lidar_link')
        self.declare_parameter('lidar_x', -0.15)
        self.declare_parameter('lidar_y', 0.0)
        self.declare_parameter('lidar_z', 0.72)
        self.declare_parameter('publish_rate', 20.0)

        self.map_frame = self.get_parameter('map_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value

        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_broadcaster = StaticTransformBroadcaster(self)
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)

        odom_topic = self.get_parameter('odom_topic').value
        self.create_subscription(Odometry, odom_topic, self.on_odom, 20)
        self.pose_pub = self.create_publisher(
            PoseStamped, self.get_parameter('pose_topic').value, 20)

        # 静态外参只发一次，对所有时间有效（不依赖 /odom 回调）
        self._publish_static_extrinsics()

        rate = float(self.get_parameter('publish_rate').value)
        self.create_timer(1.0 / rate, self.publish_vehicle_pose)
        self.create_timer(1.0 / rate, self.republish_odom_tf)

        self.has_odom = False
        self.last_odom_xy_yaw = None
        self.get_logger().info(
            f'TF 桥接已启动：{odom_topic} -> {self.odom_frame}->{self.base_frame}，'
            f'并发布 map->base 位姿（{rate:.0f} Hz）')

    # ------------------------------------------------------------------ 静态外参
    def _publish_static_extrinsics(self):
        """base_footprint -> lidar_link 的固定外参（雷达在车体前方 0.15 m、高 0.72 m）。"""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.base_frame
        t.child_frame_id = self.get_parameter('lidar_frame').value
        t.transform.translation.x = float(self.get_parameter('lidar_x').value)
        t.transform.translation.y = float(self.get_parameter('lidar_y').value)
        t.transform.translation.z = float(self.get_parameter('lidar_z').value)
        t.transform.rotation.w = 1.0
        self.static_broadcaster.sendTransform(t)

    # -------------------------------------------------------------------- 里程计
    def on_odom(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.last_odom_xy_yaw = (p.x, p.y, yaw)
        self.has_odom = True

    def republish_odom_tf(self):
        """按当前时钟重发 odom -> base。车辆静止（/odom 停发）时仍需维持 TF 时间轴。"""
        if self.last_odom_xy_yaw is None:
            return
        x, y, yaw = self.last_odom_xy_yaw
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = 0.0
        t.transform.rotation.z, t.transform.rotation.w = yaw_to_quat(yaw)
        self.tf_broadcaster.sendTransform(t)

    # ------------------------------------------------------------ 地图坐标系位姿
    def publish_vehicle_pose(self):
        if not self.has_odom:
            return
        try:
            tr = self.buffer.lookup_transform(
                self.map_frame, self.base_frame, rclpy.time.Time(),
                timeout=Duration(seconds=0.05))
        except (LookupException, ConnectivityException, ExtrapolationException):
            return

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame
        msg.pose.position.x = tr.transform.translation.x
        msg.pose.position.y = tr.transform.translation.y
        msg.pose.position.z = 0.0
        msg.pose.orientation = tr.transform.rotation
        self.pose_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TfBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
