#!/usr/bin/env python3
r"""台架转向标定节点：把「执行器欠转比 = 实际曲率 / 指令曲率」量准。

为什么需要它
------------
`PathFollower.steer_gain` 的取值完全取决于这个比值。历史上它是**从真机日志里
单帧差分 yaw 反推**的（20 Hz 下每帧航向只变 0.004 rad，被定位噪声淹没），
因此标出了 0.65~0.80 这种量级，进而把 `steer_gain` 抬到 1.55 —— 车实际多转
49%，入位圆弧半径从规划的 6.39 m 缩到 5.15 m，车尾扫进邻车、停在车位口。
台架用**真值里程计**重测的同一条曲线是 0.96。

做法
----
空世界（无任何障碍）+ 自车 + ros_gz_bridge，直接往 `/cmd_vel` 打恒定
`(v, ω=v·κ)`，用 `/odom_gt` 采样后按「累加弧长 + 展开航向增量」反算实际曲率：

    κ_meas = Σ wrap(yaw[i] − yaw[i−1]) / Σ |p[i] − p[i−1]|

比单帧差分稳得多，而且不吃定位噪声。

用法
----
    bash scripts/bench_steer.sh            # 自带仿真启动，一步到位
    source /opt/ros/jazzy/setup.bash && source ~/apa_ws/install/setup.bash
    python3 scripts/bench_steer.py         # 仿真已经起好的话，自己单独跑

输出：每个 (v, κ) 组合一行 `ratio = κ_meas/κ_cmd`，最后一条「弧长 → 累积曲率»
轨迹用来看欠转到底是**稳态**还是**过渡**。
"""
import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node

L = 2.70        # 轴距，与 model.sdf 的 <wheel_base> 一致

# (v, [κ...])：v 取入位速度、巡航速度、倒车入位速度
CASES = (
    (0.45, (0.10, 0.14, 0.175, 0.20, 0.23)),
    (1.20, (0.10, 0.14, 0.175, 0.20, 0.23)),
    (-0.45, (0.10, 0.14, 0.175, 0.20, 0.23)),
)


def unwrap_series(yaws):
    out = [yaws[0]]
    for y in yaws[1:]:
        d = y - out[-1]
        while d > math.pi:
            d -= 2.0 * math.pi
        while d < -math.pi:
            d += 2.0 * math.pi
        out.append(out[-1] + d)
    return out


class Cal(Node):
    def __init__(self):
        super().__init__('bench_steer')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.samples = []
        self.create_subscription(Odometry, '/odom_gt', self.on_odom, 100)

    def on_odom(self, m):
        p = m.pose.pose.position
        q = m.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                         1 - 2 * (q.y * q.y + q.z * q.z))
        self.samples.append((time.time(), p.x, p.y, yaw))

    def spin(self, sec):
        end = time.time() + sec
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.005)

    def run(self, v, kappa, sec=6.0, settle=2.5):
        t = Twist()
        t.linear.x = float(v)
        t.angular.z = float(v * kappa)
        self.samples.clear()
        self.spin(0.4)
        self.samples.clear()
        end = time.time() + sec
        while time.time() < end:
            self.pub.publish(t)
            rclpy.spin_once(self, timeout_sec=0.005)
        self.pub.publish(Twist())
        self.spin(0.3)
        s = list(self.samples)
        if len(s) < 20:
            print(f'  v={v:+.2f} kappa={kappa:+.3f}: 采样不足 ({len(s)})')
            return
        t0 = s[0][0]
        ss = [q for q in s if q[0] >= t0 + settle] or s
        ys = unwrap_series([q[3] for q in ss])
        arc = 0.0
        for a, b in zip(ss[:-1], ss[1:]):
            arc += math.hypot(b[1] - a[1], b[2] - a[2])
        dt = ss[-1][0] - ss[0][0]
        if arc < 0.2 or dt < 1e-3:
            print(f'  v={v:+.2f} kappa={kappa:+.3f}: 位移太小 arc={arc:.2f}')
            return
        v_meas = arc / dt
        k_meas = (ys[-1] - ys[0]) / arc
        ratio = k_meas / kappa
        delta = math.degrees(math.atan(k_meas * L))
        # [速度口径判定] gz AckermannSteering 对 (v,ω) 有两种可能的执行语义：
        #   A. ω 按绝对偏航率执行 → yaw_rate_meas ≈ ω_cmd，实际曲率 = ω_cmd/v_meas，
        #      v_meas < v_cmd 时曲率超发（0.45 命令 / 0.39 实测 → +15%）；
        #   B. 阿克曼几何（δ = atan(ω·L/v_cmd)，轮速 = v_cmd）→ yaw_rate_meas ≈
        #      v_meas·κ_cmd，曲率按命令口径执行，不超发。
        # 判定量：r_yaw = 稳态 yaw_rate / ω_cmd（≈1 → 口径 A），v_track = v_meas/v_cmd。
        w_cmd = v * kappa
        w_meas = (ys[-1] - ys[0]) / dt
        r_yaw = (w_meas / w_cmd) if abs(w_cmd) > 1e-9 else float('nan')
        v_track = v_meas / v if abs(v) > 1e-9 else float('nan')
        print(f'  v_cmd={v:+.2f} kappa_cmd={kappa:+.4f} -> v_meas={v_meas:+.3f} '
              f'kappa_meas={k_meas:+.4f} ratio={ratio:.3f} delta={delta:+.1f}deg | '
              f'w_cmd={w_cmd:+.4f} w_meas={w_meas:+.4f} r_yaw={r_yaw:.3f} '
              f'v_track={v_track:.3f}')

    def trace(self, v, kappa, sec=5.0):
        """打印「弧长 → 累积曲率」，用来区分稳态欠转和过渡滞后。"""
        t = Twist()
        t.linear.x = float(v)
        t.angular.z = float(v * kappa)
        self.samples.clear()
        self.spin(0.3)
        self.samples.clear()
        end = time.time() + sec
        while time.time() < end:
            self.pub.publish(t)
            rclpy.spin_once(self, timeout_sec=0.005)
        self.pub.publish(Twist())
        self.spin(0.3)
        s = list(self.samples)
        if len(s) < 10:
            return
        ys = unwrap_series([q[3] for q in s])
        marks = [0.5, 1.0, 1.5, 2.0, 3.0, 4.0, 5.0, 6.0]
        acc, k = 0.0, 0
        print(f'  收敛轨迹 v={v:+.2f} kappa_cmd={kappa:+.4f}')
        for i in range(1, len(s)):
            acc += math.hypot(s[i][1] - s[i - 1][1], s[i][2] - s[i - 1][2])
            while k < len(marks) and acc >= marks[k]:
                inst = (ys[i] - ys[0]) / acc if acc > 1e-6 else 0.0
                print(f'    s={marks[k]:.1f} m  kappa_avg={inst:+.4f} '
                      f'ratio={inst / kappa:.3f}')
                k += 1


def main():
    rclpy.init()
    n = Cal()
    n.spin(15.0)
    if not n.samples:
        print('收不到 /odom_gt —— 台架没起起来？先跑 scripts/bench_steer.sh')
        rclpy.shutdown()
        return 1
    print(f'起始位姿 {n.samples[-1][1:]}')
    for v, kappas in CASES:
        print(f'=== v={v:+.2f} m/s ===')
        for kap in kappas:
            n.run(v, kap)
    print('=== 稳态/过渡判定 ===')
    n.trace(0.45, 0.175)
    n.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
