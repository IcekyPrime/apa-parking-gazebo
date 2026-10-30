#!/usr/bin/env python3
"""训练 Transformer 轨迹预测模型，评估 ADE（平均位移误差）。

用法：
    python3 train.py                # 用默认参数训练
    python3 train.py --epochs 60    # 更多轮次

验收标准：验证集 5 步轨迹 ADE < 0.4 m。
"""

import argparse
import sys
from pathlib import Path

import numpy as np
import torch
from torch.utils.data import DataLoader, TensorDataset

# 让训练脚本能 import ROS 包里的模型定义（避免两份 model.py 不一致）
SRC = Path(__file__).resolve().parent.parent / 'src' / 'trajectory_prediction'
sys.path.insert(0, str(SRC))
from trajectory_prediction.model import TrajectoryTransformer  # noqa: E402


def ade(pred, gt):
    """平均位移误差（Average Displacement Error），单位米。"""
    return torch.norm(pred - gt, dim=-1).mean().item()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--data', type=Path,
                        default=Path(__file__).parent / 'trajectory_dataset.npz')
    parser.add_argument('--epochs', type=int, default=40)
    parser.add_argument('--batch-size', type=int, default=128)
    parser.add_argument('--lr', type=float, default=1e-3)
    parser.add_argument('--out', type=Path,
                        default=Path(__file__).parent / 'checkpoints' / 'trajectory_transformer.pt')
    args = parser.parse_args()

    data = np.load(args.data)
    past = torch.tensor(data['past'], dtype=torch.float32)       # (N, 8, 4)
    future = torch.tensor(data['future'], dtype=torch.float32)   # (N, 5, 2)
    type_ids = torch.tensor(data['type'], dtype=torch.long)      # (N,)

    # 划分训练/验证集（90/10）
    rng = np.random.default_rng(0)
    idx = rng.permutation(len(past))
    n_val = int(0.1 * len(past))
    val_idx, train_idx = idx[:n_val], idx[n_val:]

    train_ds = TensorDataset(past[train_idx], type_ids[train_idx], future[train_idx])
    val_ds = TensorDataset(past[val_idx], type_ids[val_idx], future[val_idx])
    train_dl = DataLoader(train_ds, batch_size=args.batch_size, shuffle=True)
    val_dl = DataLoader(val_ds, batch_size=args.batch_size)

    model = TrajectoryTransformer()
    opt = torch.optim.Adam(model.parameters(), lr=args.lr)
    sched = torch.optim.lr_scheduler.StepLR(opt, step_size=15, gamma=0.5)
    loss_fn = torch.nn.MSELoss()

    val_ade = float('inf')
    for epoch in range(1, args.epochs + 1):
        model.train()
        total_loss = 0.0
        for p, t, f in train_dl:
            opt.zero_grad()
            pred = model(p, t)
            loss = loss_fn(pred, f)
            loss.backward()
            opt.step()
            total_loss += loss.item() * p.shape[0]
        sched.step()

        model.eval()
        val_ade, val_loss = 0.0, 0.0
        with torch.no_grad():
            for p, t, f in val_dl:
                pred = model(p, t)
                val_ade += ade(pred, f) * p.shape[0]
                val_loss += loss_fn(pred, f).item() * p.shape[0]
        val_ade /= len(val_ds)
        val_loss /= len(val_ds)

        if epoch % 5 == 0 or epoch == 1:
            print(f'epoch {epoch:3d}  train_loss {total_loss / len(train_ds):.4f}  '
                  f'val_loss {val_loss:.4f}  val_ADE {val_ade:.4f} m')

    args.out.parent.mkdir(parents=True, exist_ok=True)
    torch.save({'state_dict': model.state_dict(),
                'config': dict(d_model=64, nhead=4, num_encoder_layers=2,
                               input_dim=4, num_types=2, past_len=8, future_len=5)},
               args.out)
    print(f'\n最终 val_ADE = {val_ade:.4f} m（验收标准 < 0.4 m）')
    print(f'模型已保存 -> {args.out}')


if __name__ == '__main__':
    main()
