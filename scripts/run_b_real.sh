#!/bin/bash
# B 排空位真机回归：B1/B3/B5（B0/B2/B4 被 parked_b0/1/2 占据）
# 平行位超时按记忆中的经验放宽到 900 s
export LIST="B1 B3 B5"
export TMO=900
exec bash "$HOME/apa_ws/scripts/batch_real.sh"
