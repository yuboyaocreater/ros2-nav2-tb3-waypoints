#!/usr/bin/env bash
# 设置 AMCL 初始位姿（默认与 tb3_simulation_launch 刷车位置一致）
set -e
source /opt/ros/jazzy/setup.bash

X=${1:--2.0}
Y=${2:--0.5}

ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped "{
  header: {frame_id: 'map'},
  pose: {
    pose: {
      position: {x: ${X}, y: ${Y}, z: 0.0},
      orientation: {w: 1.0}
    },
    covariance: [0.25,0,0,0,0,0, 0,0.25,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0.068]
  }
}"

echo "已发布 initialpose: x=${X}, y=${Y}, yaw=0"
