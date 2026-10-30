#!/bin/bash

# 项目根目录（由本脚本位置推出，克隆到任意路径都可用）
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# 分别离线验证 B3 / B5（verify_park_sim.py 一次只接受一个车位名）
WS="$HOME/apa_ws"
for d in /opt/ros/*/setup.bash; do source "$d"; break; done
source "$WS/install/setup.bash" 2>/dev/null

diff -q "$WS/scripts/verify_park_sim.py" \
  "$ROOT/scripts/verify_park_sim.py" \
  && echo "[sync] WSL 与 Windows 的 verify_park_sim.py 一致" \
  || echo "[warn] verify_park_sim.py 两侧不一致，以 WSL 侧为准"

for st in B3 B5; do
  echo "===== $st ====="
  python3 "$WS/scripts/verify_park_sim.py" "$st" > "/tmp/verify_${st}.log" 2>&1
  echo "退出码 $?, 日志 /tmp/verify_${st}.log"
  grep -E '^B[0-9]|达标|不达标|剐|间隙|误差' "/tmp/verify_${st}.log" | tail -6
done
