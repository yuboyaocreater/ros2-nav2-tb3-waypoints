#!/usr/bin/env python3
"""路线 A：依次发送多个 NavigateToPose 目标，完成多点巡航。

用法（仿真已启动，且已设初始位姿、导航节点 active）：
  source /opt/ros/jazzy/setup.bash
  python3 ./02_multi_goals.py
"""

import math
import subprocess
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node


# 绕开地图中间柱子，走外围空地，路径更长，方便录屏。
# (x, y, yaw_rad) 均在 map 坐标系。
WAYPOINTS = [
    (-1.8, -1.2, 0.0),
    (1.2, -1.2, 1.57),
    (1.2, 1.2, 3.14),
    (-1.8, 1.2, -1.57),
    (-1.8, -0.5, 0.0),
]

# 线速度上限 (m/s)。默认大约 0.5，录屏建议 0.12~0.18。
SLOW_VX_MAX = 0.12


def yaw_to_quat(yaw: float):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def set_slow_speed(vx_max: float) -> None:
    """调慢控制器与速度平滑器，拉长运动时间。失败不致命。"""
    cmds = [
        ["ros2", "param", "set", "/controller_server", "FollowPath.vx_max", str(vx_max)],
        [
            "ros2",
            "param",
            "set",
            "/velocity_smoother",
            "max_velocity",
            f"[{vx_max}, 0.0, 1.0]",
        ],
    ]
    for cmd in cmds:
        try:
            r = subprocess.run(cmd, capture_output=True, text=True, timeout=10)
            print(f"[调速] {' '.join(cmd[3:])}: {r.stdout.strip() or r.stderr.strip()}")
        except Exception as e:
            print(f"[调速] 失败(可忽略): {e}")


class MultiGoalNavigator(Node):
    def __init__(self):
        super().__init__("route_a_multi_goal")
        self._client = ActionClient(self, NavigateToPose, "navigate_to_pose")

    def wait_server(self, timeout_sec: float = 30.0) -> bool:
        self.get_logger().info("等待 /navigate_to_pose ...")
        return self._client.wait_for_server(timeout_sec=timeout_sec)

    def go_to(self, x: float, y: float, yaw: float = 0.0, timeout_sec: float = 300.0) -> bool:
        qx, qy, qz, qw = yaw_to_quat(yaw)
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        goal = NavigateToPose.Goal()
        goal.pose = pose

        self.get_logger().info(f"发送目标: x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}")
        send_future = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=30.0)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("目标被拒绝（检查导航节点是否 active）")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=timeout_sec)
        wrapped = result_future.result()
        if wrapped is None:
            self.get_logger().error("等待结果超时")
            return False

        status = wrapped.status
        # 4 = STATUS_SUCCEEDED, 6 = STATUS_ABORTED
        ok = status == 4
        err_code = getattr(wrapped.result, "error_code", None)
        err_msg = getattr(wrapped.result, "error_msg", "")
        self.get_logger().info(
            f"到达结果 status={status} ({'成功' if ok else '失败'}) "
            f"error_code={err_code} error_msg={err_msg!r}"
        )
        return ok


def main():
    set_slow_speed(SLOW_VX_MAX)

    rclpy.init()
    node = MultiGoalNavigator()
    try:
        if not node.wait_server():
            node.get_logger().error("Action server 不可用，先跑 01_activate_nav.sh")
            return

        success = 0
        for i, (x, y, yaw) in enumerate(WAYPOINTS, 1):
            node.get_logger().info(f"===== 第 {i}/{len(WAYPOINTS)} 个点 =====")
            if node.go_to(x, y, yaw):
                success += 1
            else:
                node.get_logger().warn(
                    "本点失败(常见原因: 点太靠近柱子/墙、定位偏了、瞬时规划失败)，继续下一点"
                )
            time.sleep(2.0)

        node.get_logger().info(f"完成：成功 {success}/{len(WAYPOINTS)}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
