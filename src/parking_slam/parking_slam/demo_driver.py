#!/usr/bin/env python3
"""航点驱动节点（纯跟踪控制）。

用于建图阶段让自车沿固定路线巡场：把折线航点做圆角平滑后，用纯跟踪
（pure pursuit）跟踪路径，向 `/cmd_vel` 发布 `geometry_msgs/Twist`。

Gazebo 的阿克曼插件把 Twist 解释为（线速度, 角速度），内部换算成前轮转角：
    delta = atan(wheel_base * angular.z / linear.x)
因此这里只需给出期望的角速度，无需直接控制转角。

参数（与 parking_lot_city/launch/city_mapping.launch.py 保持一致）：
    odom_topic, odom_world, spawn_x, spawn_y, spawn_yaw,
    cmd_topic, waypoints, speed, lookahead, fillet_radius, path_step,
    max_steer, loop, wheel_base
"""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String


def wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def parse_waypoints(text):
    pts = []
    for seg in text.split(';'):
        seg = seg.strip()
        if not seg:
            continue
        x, y = seg.split(',')
        pts.append((float(x), float(y)))
    return pts


def fillet_path(pts, radius):
    """折线圆角化：每个内部顶点用二次贝塞尔替代，返回稠密折线。"""
    if len(pts) < 3:
        return list(pts)
    out = [pts[0]]
    for i in range(1, len(pts) - 1):
        p0, p1, p2 = pts[i - 1], pts[i], pts[i + 1]
        v0 = (p1[0] - p0[0], p1[1] - p0[1])
        v1 = (p2[0] - p1[0], p2[1] - p1[1])
        l0 = math.hypot(*v0)
        l1 = math.hypot(*v1)
        if l0 < 1e-6 or l1 < 1e-6:
            out.append(p1)
            continue
        u0 = (v0[0] / l0, v0[1] / l0)
        u1 = (v1[0] / l1, v1[1] / l1)
        cos_t = max(-1.0, min(1.0, u0[0] * u1[0] + u0[1] * u1[1]))
        turn = math.acos(cos_t)
        if turn < 0.08:                      # 近乎直线，不做圆角
            out.append(p1)
            continue
        t = min(radius / max(math.tan(turn / 2.0), 1e-3), 0.45 * l0, 0.45 * l1)
        a = (p1[0] - u0[0] * t, p1[1] - u0[1] * t)
        b = (p1[0] + u1[0] * t, p1[1] + u1[1] * t)
        n = max(4, int(turn * 6))
        for k in range(n + 1):
            s = k / n
            x = (1 - s) ** 2 * a[0] + 2 * (1 - s) * s * p1[0] + s ** 2 * b[0]
            y = (1 - s) ** 2 * a[1] + 2 * (1 - s) * s * p1[1] + s ** 2 * b[1]
            out.append((x, y))
    out.append(pts[-1])
    # 去重
    dedup = [out[0]]
    for p in out[1:]:
        if math.hypot(p[0] - dedup[-1][0], p[1] - dedup[-1][1]) > 1e-3:
            dedup.append(p)
    return dedup


def resample(path, step):
    if len(path) < 2:
        return path
    out = [path[0]]
    acc = 0.0
    for i in range(1, len(path)):
        ax, ay = path[i - 1]
        bx, by = path[i]
        seg = math.hypot(bx - ax, by - ay)
        if seg < 1e-9:
            continue
        while acc + seg >= step:
            r = (step - acc) / seg
            px, py = ax + (bx - ax) * r, ay + (by - ay) * r
            out.append((px, py))
            ax, ay = px, py
            seg = math.hypot(bx - ax, by - ay)
            acc = 0.0
        acc += seg
    if math.hypot(out[-1][0] - path[-1][0], out[-1][1] - path[-1][1]) > 0.05:
        out.append(path[-1])
    return out


class DemoDriver(Node):
    def __init__(self):
        super().__init__('demo_driver')

        self.declare_parameter('odom_topic', '/odom_gt')
        self.declare_parameter('odom_world', True)
        self.declare_parameter('spawn_x', 0.0)
        self.declare_parameter('spawn_y', 0.0)
        self.declare_parameter('spawn_yaw', 0.0)
        self.declare_parameter('cmd_topic', '/cmd_vel')
        self.declare_parameter('waypoints', '')
        self.declare_parameter('speed', 1.2)
        self.declare_parameter('lookahead', 2.5)
        self.declare_parameter('fillet_radius', 5.0)
        self.declare_parameter('path_step', 0.4)
        self.declare_parameter('max_steer', 0.6)
        self.declare_parameter('wheel_base', 2.70)
        self.declare_parameter('loop', False)
        self.declare_parameter('tolerance', 0.6)
        self.declare_parameter('status_topic', '/driver_status')

        self.spawn = (float(self.get_parameter('spawn_x').value),
                      float(self.get_parameter('spawn_y').value),
                      float(self.get_parameter('spawn_yaw').value))
        self.odom_world = bool(self.get_parameter('odom_world').value)
        self.speed = float(self.get_parameter('speed').value)
        self.lookahead = float(self.get_parameter('lookahead').value)
        self.max_steer = float(self.get_parameter('max_steer').value)
        self.wheel_base = float(self.get_parameter('wheel_base').value)
        self.loop = bool(self.get_parameter('loop').value)
        self.tolerance = float(self.get_parameter('tolerance').value)

        raw = parse_waypoints(self.get_parameter('waypoints').value)
        if len(raw) < 2:
            raise RuntimeError('waypoints 参数为空或不足两个点')
        path = fillet_path(raw, float(self.get_parameter('fillet_radius').value))
        self.path = resample(path, float(self.get_parameter('path_step').value))
        self.path_len = self._path_length(self.path)
        self.get_logger().info(f'路径已生成：{len(self.path)} 点，长度 {self.path_len:.1f} m')

        self.pose = None
        self.idx = 0
        self.done = False
        self._done_announced = False

        self.create_subscription(Odometry, self.get_parameter('odom_topic').value,
                                 self.on_odom, 20)
        self.pub = self.create_publisher(Twist, self.get_parameter('cmd_topic').value, 20)
        self.pub_status = self.create_publisher(
            String, self.get_parameter('status_topic').value, 10)
        self.create_timer(0.05, self.control)      # 20 Hz

    # ---------------------------------------------------------------- 工具函数
    @staticmethod
    def _path_length(path):
        return sum(math.hypot(path[i][0] - path[i - 1][0], path[i][1] - path[i - 1][1])
                   for i in range(1, len(path)))

    def on_odom(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        if self.odom_world:
            self.pose = (p.x, p.y, yaw)
        else:
            sx, sy, syaw = self.spawn
            c, s = math.cos(syaw), math.sin(syaw)
            self.pose = (sx + c * p.x - s * p.y, sy + s * p.x + c * p.y,
                         wrap(syaw + yaw))

    def _nearest_ahead(self, x, y):
        """在当前位置附近向前搜索最近点，避免路径回折时跳到后面。"""
        best, best_d = self.idx, 1e9
        for i in range(self.idx, min(self.idx + 80, len(self.path))):
            d = math.hypot(self.path[i][0] - x, self.path[i][1] - y)
            if d < best_d:
                best_d, best = d, i
        return best

    def _lookahead_point(self, x, y, yaw):
        """沿路径向前找距离不小于 lookahead 的点。"""
        for i in range(self.idx, len(self.path)):
            if math.hypot(self.path[i][0] - x, self.path[i][1] - y) >= self.lookahead:
                return self.path[i], i
        return self.path[-1], len(self.path) - 1

    # ------------------------------------------------------------------ 控制
    def control(self):
        if self.pose is None:
            return
        if self.done:
            # [同学 C 修复] 原实现在到达终点后只发一帧零速就 return，
            # Gazebo 的阿克曼插件会一直保持上一条速度指令，车辆不会停。
            # 这里改为持续发零速，并对外广播 done 状态供仲裁层切换控制权。
            self.pub.publish(Twist())
            if not self._done_announced:
                self.pub_status.publish(String(data='done'))
                self.get_logger().info('航点路径已走完，停车（持续保持零速）')
                self._done_announced = True
            return
        self.pub_status.publish(String(data='running'))
        x, y, yaw = self.pose

        self.idx = self._nearest_ahead(x, y)
        remaining = sum(math.hypot(self.path[i][0] - self.path[i - 1][0],
                                   self.path[i][1] - self.path[i - 1][1])
                        for i in range(self.idx + 1, len(self.path)))
        if remaining < self.tolerance:
            if self.loop:
                self.idx = 0
            else:
                self.done = True
                self.pub.publish(Twist())
                self.get_logger().info('航点路径已走完，停车')
                return

        target, _ = self._lookahead_point(x, y, yaw)
        dx, dy = target[0] - x, target[1] - y
        ld = max(math.hypot(dx, dy), 1e-3)
        alpha = wrap(math.atan2(dy, dx) - yaw)

        # 纯跟踪曲率 + 转角限幅
        curvature = 2.0 * math.sin(alpha) / ld
        steer = math.atan(curvature * self.wheel_base)
        steer = max(-self.max_steer, min(self.max_steer, steer))

        v = self.speed
        if abs(alpha) > 0.7:                 # 大转角减速，避免切内道
            v *= max(0.45, 1.0 - (abs(alpha) - 0.7))

        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = v * math.tan(steer) / self.wheel_base
        self.pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = DemoDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
