#!/usr/bin/env python3
"""多目标检测跟踪节点。

订阅 /scan (LaserScan)，聚类得到目标，跟踪估计速度，发布 /tracks。

流程：
1. 极坐标 -> 笛卡尔坐标
2. 按相邻扫描点间距做贪心聚类（break-point detector）
3. 过滤点数过少的簇
4. 最近邻关联维护目标 id，估计 (x, y, vx, vy)
5. 连续稳定跟踪 CONFIRM_FRAMES 帧后才发布（track confirmation，区分噪声与真实静止目标）
6. 按簇尺寸/速度粗分类型 pedestrian / vehicle
"""

import math
from dataclasses import dataclass

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

from prediction_interfaces.msg import TrackedObject, TrackedObjects

MIN_CLUSTER_POINTS = 3   # 一个目标至少包含的扫描点数
BREAK_DIST = 0.4         # 相邻点聚类断开距离（米），可调
ASSOC_DIST = 0.8         # 跟踪关联最大距离（米）
MAX_TRACK_MISS = 5       # 丢失多少帧后删除轨迹
MAX_RANGE = 15.0         # 检测有效范围上限（米）：泊车走廊约 7m、横穿行人最远约 9m，
                         # 15m 覆盖全部相关目标，排除 26m 外无关的 parked_e 排车与远场噪声
CONFIRM_FRAMES = 3       # 目标需连续稳定跟踪多少帧才发布（track confirmation）。
                         # 噪声簇是间歇性的（出现 1~2 帧就消失），真实目标（含静止行人/车辆）
                         # 持续稳定出现——用时间一致性区分噪声与真实静止目标，而不是速度阈值
PEDESTRIAN_CORRECTION = 0.08   # 质心→物体中心校正量（米），行人：2D LiDAR 只打到近端表面，
                               # 圆柱行人(R=0.25m)实测质心径向偏置约 0.08m，沿视线回推消除系统偏置
VEHICLE_CORRECTION = 1.0       # 车辆：近端面质心偏置先验（车深 2~4.5m，侧视≈1m / 端视≈2.25m，
                               # 取保守 1.0m），待真车实测标定
ALPHA_BETA_A = 0.4           # alpha-beta 恒速滤波位置增益（平滑质心抖动）
ALPHA_BETA_B = 0.1           # alpha-beta 恒速滤波速度增益


@dataclass
class Track:
    id: int
    x: float
    y: float
    vx: float
    vy: float
    type: str
    miss: int
    frames_seen: int


def laser_to_points(scan):
    """极坐标转笛卡尔坐标，跳过无效点与超出有效范围的点。"""
    pts = []
    for i, r in enumerate(scan.ranges):
        if r < scan.range_min or r > scan.range_max:
            continue
        if r > MAX_RANGE:
            continue
        a = scan.angle_min + i * scan.angle_increment
        pts.append((r * math.cos(a), r * math.sin(a)))
    return pts


def cluster_points(pts):
    """按相邻点间距贪心聚类，返回簇列表。"""
    clusters = []
    cur = []
    prev = None
    for p in pts:
        if prev is None:
            cur = [p]
        else:
            d = math.hypot(p[0] - prev[0], p[1] - prev[1])
            if d < BREAK_DIST:
                cur.append(p)
            else:
                if len(cur) >= MIN_CLUSTER_POINTS:
                    clusters.append(cur)
                cur = [p]
        prev = p
    if len(cur) >= MIN_CLUSTER_POINTS:
        clusters.append(cur)
    return clusters


def cluster_stat(cluster):
    """返回簇质心 (cx, cy) 与尺寸（外接盒对角近似）。"""
    xs = [p[0] for p in cluster]
    ys = [p[1] for p in cluster]
    cx = sum(xs) / len(xs)
    cy = sum(ys) / len(ys)
    return cx, cy, math.hypot(max(xs) - min(xs), max(ys) - min(ys))


class DetectionNode(Node):
    def __init__(self):
        super().__init__('detection_node')
        self.sub = self.create_subscription(LaserScan, '/scan', self.on_scan, 10)
        self.pub = self.create_publisher(TrackedObjects, '/tracks', 10)
        self.tracks = {}      # id -> Track
        self.next_id = 0
        self.last_time = None
        self.get_logger().info('检测跟踪节点已启动')

    def on_scan(self, msg):
        now = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        dt = None
        if self.last_time is not None:
            dt = now - self.last_time
        self.last_time = now

        pts = laser_to_points(msg)
        clusters = cluster_points(pts)

        # 最近邻关联（贪心）
        matched = set()
        for cl in clusters:
            cx, cy, size = cluster_stat(cl)

            # 质心 → 物体中心校正：2D LiDAR 只打到目标近端表面，沿视线方向回推消除系统偏置。
            # 校正量与目标类型相关（行人≈0.08m，车辆≈1.0m），用簇尺寸粗分（与下方 type 判断同阈值）。
            corr = VEHICLE_CORRECTION if size > 1.0 else PEDESTRIAN_CORRECTION
            r = math.hypot(cx, cy)
            if r > 1e-3:
                cx += cx / r * corr
                cy += cy / r * corr

            best_id, best_d = None, ASSOC_DIST
            for tid, tr in self.tracks.items():
                if tid in matched:
                    continue
                d = math.hypot(cx - tr.x, cy - tr.y)
                if d < best_d:
                    best_d, best_id = d, tid

            if best_id is not None:
                tr = self.tracks[best_id]
                if dt and dt > 0.0:
                    # 恒速 alpha-beta 滤波：先按上帧速度预测，再用当前测量修正，
                    # 平滑质心抖动、估计速度（替代原单帧差分 + 指数平滑）。
                    px = tr.x + tr.vx * dt
                    py = tr.y + tr.vy * dt
                    rx = cx - px
                    ry = cy - py
                    tr.x = px + ALPHA_BETA_A * rx
                    tr.y = py + ALPHA_BETA_A * ry
                    tr.vx += (ALPHA_BETA_B / dt) * rx
                    tr.vy += (ALPHA_BETA_B / dt) * ry
                else:
                    tr.x, tr.y = cx, cy
                speed = math.hypot(tr.vx, tr.vy)
                tr.type = 'vehicle' if (size > 1.0 or speed > 2.5) else 'pedestrian'
                tr.miss = 0
                tr.frames_seen += 1
                matched.add(best_id)
            else:
                tid = self.next_id
                self.tracks[tid] = Track(
                    tid, cx, cy, 0.0, 0.0,
                    'vehicle' if size > 1.0 else 'pedestrian', 0, 1)
                self.next_id += 1
                matched.add(tid)

        # 未匹配轨迹计数，超时删除
        for tid in list(self.tracks.keys()):
            if tid not in matched:
                self.tracks[tid].miss += 1
                if self.tracks[tid].miss > MAX_TRACK_MISS:
                    del self.tracks[tid]

        out = TrackedObjects()
        out.header = msg.header
        for tr in self.tracks.values():
            if tr.miss > 0:
                continue
            if tr.frames_seen < CONFIRM_FRAMES:
                continue  # 尚未确认：连续稳定跟踪不足 CONFIRM_FRAMES 帧，视为疑似噪声
            obj = TrackedObject()
            obj.id = tr.id
            obj.type = tr.type
            obj.x = float(tr.x)
            obj.y = float(tr.y)
            obj.vx = float(tr.vx)
            obj.vy = float(tr.vy)
            out.objects.append(obj)
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = DetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
