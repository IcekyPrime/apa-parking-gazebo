#!/bin/bash
# W9 拼接解失败模式的逐拍追踪（临时脚本）
source /opt/ros/jazzy/setup.bash
source ~/apa_ws/install/setup.bash
cd "$(dirname "$0")"
HANDOFF_BACK=0.3 HANDOFF_LAT=0.15 TRACE_CSV=/tmp/w9_trace.csv \
  python3 verify_park_sim.py W9 2>&1 | grep -E "^W9|达标"
echo "---- trace head (first 40 lines) ----"
head -40 /tmp/w9_trace.csv
echo "---- around shift (gear changes) ----"
awk -F, 'NR==1 || $8!=p {print} {p=$8}' /tmp/w9_trace.csv | head -20
echo "---- rows where |phi|>0.15 or |e|>0.3 ----"
awk -F, 'NR>1 && (sqrt($6*$6)>0.15 || sqrt($5*$5)>0.3)' /tmp/w9_trace.csv | head -30
