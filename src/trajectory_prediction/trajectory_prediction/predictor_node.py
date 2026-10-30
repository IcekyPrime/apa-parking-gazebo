#!/usr/bin/env python3
"""Transformer 轨迹预测节点。

订阅 /tracks (TrackedObjects)，维护每个目标过去 8 帧历史，推理未来 5 步轨迹，
发布 /predicted_trajectories。

要点：
1. /tracks 通常以激光帧率（如 10Hz）发布，而模型按 dt=0.4s 训练，
   这里按消息时间戳降采样，距上次采样满 0.4s 才往历史缓冲里追加一帧。
2. 自车运动补偿：/tracks 是"车体坐标"，自车运动会让静止目标在车体系里
   "看起来在动"。这里订阅 /vehicle_pose、把每一帧车体坐标重投影到"当前自车
   位姿下冻结的车体系"（消除自车平动+转动），使模型输入与训练分布（对象世界
   运动）一致——静止目标在世界里不动，补偿后其在车体系里的历史也应不动。
"""

import math
import os
from collections import deque

import numpy as np
import torch
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

from prediction_interfaces.msg import (
    PredictedTrajectory,
    PredictedTrajectories,
    TrackedObjects,
)
from trajectory_prediction.model import TrajectoryTransformer

DT = 0.4            # 采样/预测步长（秒）
HISTORY_LEN = 8     # 过去帧数


class PredictorNode(Node):
    def __init__(self):
        super().__init__('predictor_node')

        # 默认权重位置与 `training/train.py` 的输出、以及 launch 里的
        # model_path 保持一致；可用参数 model_path 覆盖。
        default_model = os.path.expanduser(
            '~/apa_ws/training/checkpoints/trajectory_transformer.pt')
        self.model_path = self.declare_parameter('model_path', default_model).value

        self.model = TrajectoryTransformer()
        if self.model_path and os.path.exists(self.model_path):
            ckpt = torch.load(self.model_path, map_location='cpu', weights_only=True)
            self.model.load_state_dict(ckpt['state_dict'])
            self.model.eval()
            self.get_logger().info(f'已加载模型: {self.model_path}')
        else:
            self.get_logger().warn('未找到模型权重，节点将不输出预测。请先运行 training/train.py')

        # id -> {'buf': deque(maxlen=8, (t, x, y)), 'last_t': 上次采样时间}
        self.history = {}
        self.sub = self.create_subscription(TrackedObjects, '/tracks', self.on_tracks, 10)
        self.pub = self.create_publisher(PredictedTrajectories, '/predicted_trajectories', 10)

        # 自车位姿历史 (t, x, y, yaw)，用于自车运动补偿
        self.pose_topic = self.declare_parameter('pose_topic', '/vehicle_pose').value
        self.sub_pose = self.create_subscription(PoseStamped, self.pose_topic, self.on_pose, 10)
        self._ego_hist = deque(maxlen=200)   # (t, x, y, yaw)

    def on_pose(self, msg):
        q = msg.pose.orientation
        yaw = 2.0 * math.atan2(q.z, q.w)
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self._ego_hist.append((t, msg.pose.position.x, msg.pose.position.y, yaw))

    def _ego_pose_at(self, t):
        """线性插值求 t 时刻自车位姿 (x, y, yaw)；历史不足返回 None。"""
        hist = self._ego_hist
        if not hist:
            return None
        if t <= hist[0][0]:
            return hist[0][1:]
        if t >= hist[-1][0]:
            return hist[-1][1:]
        lo, hi = 0, len(hist) - 1
        while hi - lo > 1:
            mid = (lo + hi) // 2
            if hist[mid][0] < t:
                lo = mid
            else:
                hi = mid
        t0, x0, y0, yaw0 = hist[lo]
        t1, x1, y1, yaw1 = hist[hi]
        if t1 <= t0:
            return (x0, y0, yaw0)
        w = (t - t0) / (t1 - t0)
        x = x0 + w * (x1 - x0)
        y = y0 + w * (y1 - y0)
        dyaw = yaw1 - yaw0
        dyaw = (dyaw + math.pi) % (2.0 * math.pi) - math.pi
        return (x, y, yaw0 + w * dyaw)

    def _compensate(self, buf):
        """把过去 8 帧车体坐标重投影到"当前自车冻结车体系"，返回 (8, 2) 或 None。

        车体坐标 B(t) = R(t)^T (W(t) - T(t))，随自车运动而变化；重投影到当前帧
        车体系 B_frozen(t) = R(t0)^T (W(t) - T(t0)) 后，只剩对象自身世界运动。
        """
        cur = self._ego_pose_at(buf[-1][0])
        if cur is None:
            return None
        cx, cy, cyaw = cur
        cc, cs = math.cos(cyaw), math.sin(cyaw)
        out = np.empty((len(buf), 2))
        for i, (t, bx, by) in enumerate(buf):
            p = self._ego_pose_at(t)
            if p is None:
                return None
            ex, ey, yaw = p
            c, s = math.cos(yaw), math.sin(yaw)
            wx = ex + c * bx - s * by              # 车体 -> 世界
            wy = ey + s * bx + c * by
            out[i, 0] = cc * (wx - cx) + cs * (wy - cy)   # 世界 -> 当前冻结车体系
            out[i, 1] = -cs * (wx - cx) + cc * (wy - cy)
        return out

    def on_tracks(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        out = PredictedTrajectories()
        out.header = msg.header

        for obj in msg.objects:
            st = self.history.setdefault(
                obj.id, {'buf': deque(maxlen=HISTORY_LEN), 'last_t': None})

            if st['last_t'] is None or (t - st['last_t']) >= DT:
                st['buf'].append((t, obj.x, obj.y))
                st['last_t'] = t

            buf = st['buf']
            if len(buf) < HISTORY_LEN:
                continue

            ref = np.array(buf[-1][1:3], dtype=np.float32)   # 当前帧车体坐标
            body = self._compensate(buf)
            if body is None:
                body = np.array([b[1:3] for b in buf], dtype=np.float64)

            past = np.zeros((HISTORY_LEN, 4), dtype=np.float32)
            past[:, 0:2] = body - ref                        # 坐标相对当前帧归一化
            # 用位置向后差分重算速度，保证与训练数据速度-位置一致
            vel = np.empty_like(past[:, 2:4])
            vel[1:] = (past[1:, 0:2] - past[:-1, 0:2]) / DT
            vel[0] = vel[1]
            past[:, 2:4] = vel
            type_id = 0 if obj.type == 'pedestrian' else 1

            with torch.no_grad():
                future = self.model(
                    torch.tensor(past[None]), torch.tensor([type_id]))   # (1, 5, 2)
            future = future[0].numpy() + ref            # 还原为当前冻结车体系的绝对坐标

            traj = PredictedTrajectory()
            traj.id = obj.id
            traj.type = obj.type
            traj.dt = DT
            traj.future_x = future[:, 0].tolist()
            traj.future_y = future[:, 1].tolist()
            out.trajectories.append(traj)

        if out.trajectories:
            self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = PredictorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
