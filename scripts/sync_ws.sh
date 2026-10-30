#!/bin/bash
# 把仓库里的源码同步到构建工作区 `~/apa_ws/src`，然后只重编 parking_planner。
#
# 为什么要这一步：`~/apa_ws/build/parking_planner/parking_planner` 是指向
# `~/apa_ws/src/parking_planner/parking_planner` 的**符号链接**，所以运行中的
# 节点读的是 `~/apa_ws/src` 那一份；而本仓库（`Project/src`）是交付/版本管理的
# 那一份。两份必须保持一致，否则「改了代码、真机没变化」——
# 本轮就踩过：改了 controller.py 后离线回归结果一字不变，最后查到是
# `~/apa_ws/src/.../controller.py` 还是旧文件（Oct 25 22:50）。
#
# 用法：bash scripts/sync_ws.sh [工作区路径，默认 ~/apa_ws]
WS=${1:-$HOME/apa_ws}
HERE=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
SRC="$HERE/src"
echo "[sync] $SRC -> $WS/src"
for pkg in "$SRC"/*/; do
  name=$(basename "$pkg")
  [ "$name" = "training" ] && continue        # 训练脚本不进工作区
  mkdir -p "$WS/src/$name"
  cp -a "$pkg." "$WS/src/$name/"
done
echo "[sync] 对比结果（应无输出）："
diff -rq -x __pycache__ -x training "$WS/src" "$SRC" || true
