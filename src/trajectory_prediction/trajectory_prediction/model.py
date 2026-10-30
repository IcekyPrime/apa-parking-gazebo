"""轻量 Transformer 轨迹预测模型。

按 Instruction.md 设计：
- 两层 Transformer Encoder（d_model=64，4 头注意力）
- 输入：过去 8 帧 [x, y, vx, vy] + 目标类型嵌入
- 输出：未来 5 步位置序列 (x, y)

坐标约定：
- 输入 x/y 为相对当前帧（最后一帧过去）的坐标，vx/vy 为绝对速度
- 输出未来位置同样相对当前帧，推理时需加回当前帧绝对坐标
"""

import torch
import torch.nn as nn


class TrajectoryTransformer(nn.Module):
    """Transformer 编码器 + MLP 解码头。"""

    def __init__(
        self,
        d_model: int = 64,
        nhead: int = 4,
        num_encoder_layers: int = 2,
        dim_feedforward: int = 128,
        input_dim: int = 4,     # x, y, vx, vy
        num_types: int = 2,     # pedestrian, vehicle
        past_len: int = 8,
        future_len: int = 5,
        dropout: float = 0.1,
    ):
        super().__init__()
        self.d_model = d_model
        self.input_dim = input_dim
        self.past_len = past_len
        self.future_len = future_len

        # 输入嵌入：每帧 4 维特征升到 d_model
        self.input_proj = nn.Linear(input_dim, d_model)
        # 目标类型嵌入（加到每个时间步）
        self.type_embedding = nn.Embedding(num_types, d_model)
        # 可学习位置编码
        self.pos_embedding = nn.Parameter(torch.randn(1, past_len, d_model) * 0.02)

        encoder_layer = nn.TransformerEncoderLayer(
            d_model=d_model,
            nhead=nhead,
            dim_feedforward=dim_feedforward,
            dropout=dropout,
            batch_first=True,
        )
        self.encoder = nn.TransformerEncoder(encoder_layer, num_layers=num_encoder_layers)

        # MLP 解码头：序列表示 -> 未来 future_len 步 (x, y)
        self.head = nn.Sequential(
            nn.Linear(d_model, d_model),
            nn.ReLU(),
            nn.Linear(d_model, future_len * 2),
        )

    def forward(self, past, type_ids):
        """前向传播。

        Args:
            past: (B, past_len, input_dim) 过去轨迹，x/y 相对当前帧归一化
            type_ids: (B,) 目标类型索引，0=pedestrian, 1=vehicle

        Returns:
            (B, future_len, 2) 未来位置（相对当前帧）
        """
        B = past.shape[0]

        # 输入嵌入 + 类型嵌入 + 位置编码
        x = self.input_proj(past)                                  # (B, L, d_model)
        x = x + self.type_embedding(type_ids).unsqueeze(1)         # (B, L, d_model)
        x = x + self.pos_embedding[:, : self.past_len]             # (B, L, d_model)

        # Transformer Encoder
        enc = self.encoder(x)                                      # (B, L, d_model)

        # 取最后一个时间步的输出作为序列表示
        rep = enc[:, -1, :]                                        # (B, d_model)

        # 解码未来位置
        out = self.head(rep)                                       # (B, future_len * 2)
        return out.view(B, self.future_len, 2)
