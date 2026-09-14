#!/usr/bin/env bash
# 路线 A 调参对比：改最大速度后观察车变快/变慢（需仿真已在跑）
# 用法：
#   bash ./03_set_max_vel.sh 0.26
#   bash ./03_set_max_vel.sh 0.10
set -e
source /opt/ros/jazzy/setup.bash

VEL=${1:-0.26}

# FollowPath 控制器常用参数名（Jazzy Nav2 默认）
ros2 param set /controller_server FollowPath.desired_linear_vel "$VEL" || \
ros2 param set /controller_server desired_linear_vel "$VEL" || true

echo "已尝试设置线速度上限为 ${VEL} m/s"
echo "请再跑一次 02_multi_goals.py 或点 Nav2 Goal，对比快慢并录屏。"
echo "当前相关参数："
ros2 param list /controller_server 2>/dev/null | grep -iE 'vel|speed' | head -30 || true
