#!/usr/bin/env bash
# 强制把导航相关节点拉到 active，并做检查（解决 status=6 / 车不动）
set -e
source /opt/ros/jazzy/setup.bash

bring_up() {
  local n="$1"
  local st
  st=$(ros2 lifecycle get "$n" 2>/dev/null | head -1 || echo "missing")
  if [[ "$st" == *missing* ]] || [[ -z "$st" ]]; then
    echo "[MISS] $n"
    return 0
  fi
  if [[ "$st" == *unconfigured* ]]; then
    echo "[CFG ] $n"
    ros2 lifecycle set "$n" configure || true
    sleep 0.3
  fi
  st=$(ros2 lifecycle get "$n" 2>/dev/null | head -1 || true)
  if [[ "$st" == *inactive* ]]; then
    echo "[ACT ] $n"
    ros2 lifecycle set "$n" activate || true
    sleep 0.3
  fi
  st=$(ros2 lifecycle get "$n" 2>/dev/null | head -1 || true)
  echo "[NOW ] $n -> $st"
}

echo "===== 定位 ====="
bring_up /map_server
bring_up /amcl

echo "===== 导航 ====="
for n in \
  /controller_server \
  /smoother_server \
  /planner_server \
  /behavior_server \
  /bt_navigator \
  /velocity_smoother \
  /waypoint_follower
do
  bring_up "$n"
done

echo
echo "===== 必须全部为 active，否则会 status=6 车不动 ====="
ok=1
for n in /map_server /amcl /controller_server /planner_server /bt_navigator; do
  st=$(ros2 lifecycle get "$n" 2>/dev/null | head -1 || echo missing)
  echo "$n: $st"
  if [[ "$st" != *active* ]]; then
    ok=0
  fi
done

if [[ "$ok" -ne 1 ]]; then
  echo
  echo "仍有节点未 active。请 Ctrl+C 停掉仿真后完整重启，再按 README 顺序执行。"
  exit 1
fi

echo
echo "节点就绪。请在仓库根目录再执行："
echo "  bash ./00_set_initial_pose.sh"
echo "  python3 ./02_multi_goals.py"
