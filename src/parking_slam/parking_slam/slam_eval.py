#!/usr/bin/env python3
"""定位精度评估：把 SLAM 位姿与里程计推算位姿分别和 Gazebo 真值比对。

订阅：
    /vehicle_pose   SLAM 定位结果（map 坐标系）
    /odom           轮式里程计（相对）
    /odom_gt        Gazebo 世界真值位姿

输出：
    CSV（时间、估计、真值、里程计推算）与一份 ATE 汇总（末行打印 + .txt）。

用法：
    ros2 run parking_slam slam_eval --ros-args -p csv:=~/apa_ws/logs/ate.csv
"""

import math
import os

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry


def wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class SlamEval(Node):
    def __init__(self):
        super().__init__('slam_eval')
        self.declare_parameter('csv', os.path.expanduser('~/apa_ws/logs/ate.csv'))
        self.csv_path = self.get_parameter('csv').value
        os.makedirs(os.path.dirname(self.csv_path), exist_ok=True)

        self.gt = None
        self.est = None
        self.odom_raw = None
        self.odom_world = None          # 里程计积分到世界坐标
        self.spawn = None               # 首帧真值作为世界原点，保证与 /odom_gt 对齐

        self.create_subscription(PoseStamped, '/vehicle_pose', self.on_est, 20)
        self.create_subscription(Odometry, '/odom', self.on_odom, 20)
        self.create_subscription(Odometry, '/odom_gt', self.on_gt, 20)

        self.rows = []
        self.create_timer(0.2, self.tick)      # 5 Hz 采样

    # ------------------------------------------------------------------ 回调
    def on_est(self, msg):
        self.est = (msg.pose.position.x, msg.pose.position.y,
                    yaw_of(msg.pose.orientation))

    def on_gt(self, msg):
        p = msg.pose.pose.position
        self.gt = (p.x, p.y, yaw_of(msg.pose.pose.orientation))

    def on_odom(self, msg):
        p = msg.pose.pose.position
        yaw = yaw_of(msg.pose.pose.orientation)
        self.odom_raw = (p.x, p.y, yaw)
        if self.spawn is None:
            self.spawn = self.odom_raw
        sx, sy, syaw = self.spawn
        c, s = math.cos(syaw), math.sin(syaw)
        self.odom_world = (sx + c * p.x - s * p.y,
                           sy + s * p.x + c * p.y,
                           wrap(syaw + yaw))

    # ------------------------------------------------------------------ 采样
    def tick(self):
        if self.gt is None:
            return
        t = self.get_clock().now().nanoseconds * 1e-9
        est = self.est or (float('nan'),) * 3
        odom = self.odom_world or (float('nan'),) * 3
        self.rows.append((t, est[0], est[1], odom[0], odom[1],
                          self.gt[0], self.gt[1]))

    # ------------------------------------------------------------------ 收尾
    def report(self):
        if not self.rows:
            self.get_logger().warn('没有采集到评估数据')
            return
        with open(self.csv_path, 'w') as f:
            f.write('t,est_x,est_y,odom_x,odom_y,gt_x,gt_y\n')
            for r in self.rows:
                f.write('%.3f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n' % r)

        def ate(ix, iy):
            errs = [math.hypot(r[ix] - r[5], r[iy] - r[6]) for r in self.rows
                    if not math.isnan(r[ix])]
            if not errs:
                return float('nan')
            return math.sqrt(sum(e * e for e in errs) / len(errs))

        def ate_aligned(ix, iy):
            """先按**首帧**平移对齐再算 RMSE（消除 map 帧与世界帧的原点差）。"""
            rs = [r for r in self.rows if not math.isnan(r[ix])]
            if not rs:
                return float('nan')
            dx0 = rs[0][ix] - rs[0][5]
            dy0 = rs[0][iy] - rs[0][6]
            e2 = [(r[ix] - dx0 - r[5]) ** 2 + (r[iy] - dy0 - r[6]) ** 2 for r in rs]
            return math.sqrt(sum(e2) / len(e2))

        est_ate = ate(1, 2)
        odom_ate = ate(3, 4)
        # [同学 C 修复] 两条支线都必须**先去原点偏置再算 ATE**。
        # `/vehicle_pose` 在 map 帧，`/odom_gt` 在 Gazebo 世界帧，两者相差一个
        # 出生点平移；直接相减得到的是「出生点距离」，不是定位误差。
        # 实测旧逻辑给出的 `纯里程计 ATE RMSE = 9.552 m` 就是出生点偏移，
        # 而首帧对齐后同一条数据是 0.000 m —— 也就是说那一份日志里里程计其实
        # 完全跟得上，SLAM 那条却是 NaN（`/vehicle_pose` 一帧都没收到）。
        est_ate_a = ate_aligned(1, 2)
        odom_ate_a = ate_aligned(3, 4)
        n_est = sum(1 for r in self.rows if not math.isnan(r[1]))
        summary = (f'采样 {len(self.rows)} 帧（其中 /vehicle_pose 有效 {n_est} 帧）\n'
                   f'SLAM 定位 ATE RMSE（首帧对齐）= {est_ate_a:.3f} m\n'
                   f'纯里程计 ATE RMSE（首帧对齐）= {odom_ate_a:.3f} m\n'
                   f'（未对齐的原始值仅供参考：SLAM {est_ate:.3f} m / '
                   f'里程计 {odom_ate:.3f} m）\n')
        if est_ate_a == est_ate_a and odom_ate_a == odom_ate_a and odom_ate_a > 1e-6:
            summary += f'相对改善           = {100.0 * (1 - est_ate_a / odom_ate_a):.1f}%\n'
        if n_est == 0:
            summary += '警告：全程没有收到 /vehicle_pose，SLAM 那条支线没数据\n'
        txt = os.path.splitext(self.csv_path)[0] + '_summary.txt'
        with open(txt, 'w') as f:
            f.write(summary)
        self.get_logger().info('定位评估结果：\n' + summary)
        print(summary)


def main(args=None):
    rclpy.init(args=args)
    node = SlamEval()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.report()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
