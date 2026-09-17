#!/usr/bin/env python3
"""外侧绕圈。行人一直来回；用路程/速度对齐，在南、北口各停一次。"""

import math
import subprocess
import time

import rclpy
from geometry_msgs.msg import PoseArray, PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from tf2_ros import Buffer, TransformException, TransformListener

# 行人往南经过 y=1.35 再发车（人还在北边，车有足够时间赶到南口）
# 南口/北口先开到交叉口西/东侧停下，等人走进车头再走，避免人先到已折返
START_XY = (-2.00, -0.50)
START_YAW = -1.57
WAYPOINTS = [
    (-2.00, -1.48, -1.57),
    (-0.20, -1.48, 0.0),
    (1.40, -1.48, 0.0),
    (1.40, 1.48, 1.57),
    (1.05, 1.48, 3.14),
    (-2.00, 1.40, 3.14),
    (-2.00, -0.50, -1.57),
]
WAIT_FOR_PED = {1, 4}

VX_MAX = 0.70
STOP_X = 0.75
LANE_Y = 0.42
CANCEL_COOLDOWN = 0.5
PED_GO_SOUTH_Y = 1.35


def yaw_to_quat(yaw: float):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def quat_yaw(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def set_speed(vx_max: float) -> None:
    for args in (
        ["ros2", "param", "set", "/controller_server", "FollowPath.vx_max", str(vx_max)],
        [
            "ros2",
            "param",
            "set",
            "/velocity_smoother",
            "max_velocity",
            f"[{vx_max}, 0.0, 2.5]",
        ],
    ):
        try:
            subprocess.run(args, capture_output=True, text=True, timeout=3)
        except Exception:
            pass


def gz_set_robot(x: float, y: float, yaw: float) -> None:
    z, w = math.sin(yaw / 2.0), math.cos(yaw / 2.0)
    try:
        subprocess.run(
            [
                "gz",
                "service",
                "-s",
                "/world/default/set_pose",
                "--reqtype",
                "gz.msgs.Pose",
                "--reptype",
                "gz.msgs.Boolean",
                "--timeout",
                "400",
                "--req",
                f'name: "turtlebot3_waffle" position {{ x: {x:.4f} y: {y:.4f} z: 0.08 }} '
                f"orientation {{ z: {z:.5f} w: {w:.5f} }}",
            ],
            capture_output=True,
            text=True,
            timeout=1.0,
        )
    except Exception:
        pass


class MultiGoalNavigator(Node):
    def __init__(self):
        super().__init__(
            "route_a_multi_goal",
            parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)],
        )
        self._client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self._walkers: list[tuple[float, float]] = []
        self._last_cancel = 0.0
        self._tf = Buffer()
        self._tf_listener = TransformListener(self._tf, self)
        self.create_subscription(PoseArray, "/demo_walkers", self._on_walkers, 10)
        self._init_pub = self.create_publisher(PoseWithCovarianceStamped, "initialpose", 10)

    def _on_walkers(self, msg: PoseArray) -> None:
        self._walkers = [(p.position.x, p.position.y) for p in msg.poses]

    def _robot_xy_yaw(self) -> tuple[float, float, float] | None:
        for frame in ("base_footprint", "base_link"):
            try:
                tf = self._tf.lookup_transform("map", frame, rclpy.time.Time())
                t = tf.transform.translation
                return t.x, t.y, quat_yaw(tf.transform.rotation)
            except TransformException:
                continue
        return None

    def walker_ahead(self) -> float | None:
        if not self._walkers:
            return None
        robot = self._robot_xy_yaw()
        if robot is None:
            return None
        rx, ry, yaw = robot
        best = None
        for wx, wy in self._walkers:
            dx, dy = wx - rx, wy - ry
            fx = math.cos(yaw) * dx + math.sin(yaw) * dy
            fy = -math.sin(yaw) * dx + math.cos(yaw) * dy
            if 0.08 < fx < STOP_X and abs(fy) < LANE_Y:
                best = fx if best is None else min(best, fx)
        return best

    def wait_server(self, timeout_sec: float = 20.0) -> bool:
        t0 = time.time()
        while rclpy.ok() and time.time() - t0 < timeout_sec:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._client.server_is_ready():
                return True
        self.get_logger().error("navigate_to_pose 不可用，请再跑 bash ./01_activate_nav.sh")
        return False

    def wait_ped_going_south(self, timeout_sec: float = 8.0) -> bool:
        self.get_logger().info("等行人往南走再发车，最多 8 秒")
        t0 = time.time()
        last_y = None
        last_log = 0.0
        seen_at = None
        while rclpy.ok() and time.time() - t0 < timeout_sec:
            rclpy.spin_once(self, timeout_sec=0.1)
            now = time.time()
            if not self._walkers:
                if now - last_log > 2.0:
                    self.get_logger().info("还没收到 /demo_walkers，请确认 04 在跑")
                    last_log = now
                continue
            if seen_at is None:
                seen_at = now
            y = self._walkers[0][1]
            going_south = last_y is not None and y < last_y - 0.002
            if now - last_log > 2.0:
                self.get_logger().info(f"行人 y={y:.2f} 往南={going_south}")
                last_log = now
            if going_south and y < 1.45:
                self.get_logger().info(f"行人 y={y:.2f} 往南，发车")
                return True
            if seen_at is not None and now - seen_at >= 8.0:
                self.get_logger().info("已收到行人位置，发车")
                return True
            last_y = y
        self.get_logger().warn("未对齐也发车，避免小车一直停着")
        return True

    def restore_robot(self) -> None:
        x, y, yaw = START_XY[0], START_XY[1], START_YAW
        gz_set_robot(x, y, yaw)
        q = yaw_to_quat(yaw)
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation.z = q[2]
        msg.pose.pose.orientation.w = q[3]
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.068
        for _ in range(3):
            self._init_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.05)
        time.sleep(0.4)

    def wait_for_clear(self) -> None:
        """停车后：行人往回走，并走过路口那排白柱，车才起步。"""
        robot = self._robot_xy_yaw()
        ry = 0.0 if robot is None else robot[1]
        south_gate = ry < 0.2
        t0 = time.time()
        last_y = None
        going_back = False
        while rclpy.ok() and time.time() - t0 < 5.0:
            rclpy.spin_once(self, timeout_sec=0.06)
            if not self._walkers:
                continue
            y = self._walkers[0][1]
            if last_y is not None:
                if south_gate and y > last_y + 0.004:
                    going_back = True
                if (not south_gate) and y < last_y - 0.004:
                    going_back = True
            last_y = y
            if time.time() - t0 < 0.3:
                continue
            if south_gate and going_back and y > -0.85:
                self.get_logger().info(f"[wait] 行人北返已过南柱 y={y:.2f}，起步")
                return
            if (not south_gate) and going_back and y < 0.85:
                self.get_logger().info(f"[wait] 行人南返已过北柱 y={y:.2f}，起步")
                return
        self.get_logger().info("[wait] 让行结束，起步")

    def wait_until_ahead(self, timeout_sec: float = 12.0) -> bool:
        self.get_logger().info("[wait] 停在交叉口，等行人走进车头")
        t0 = time.time()
        while rclpy.ok() and time.time() - t0 < timeout_sec:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.walker_ahead() is not None:
                self.wait_for_clear()
                return True
        self.get_logger().warn("[wait] 这轮行人没走进车头")
        return False

    def go_to(
        self,
        x: float,
        y: float,
        yaw: float = 0.0,
        timeout_sec: float = 180.0,
        wait_for_ped: bool = False,
    ) -> bool:
        deadline = time.time() + timeout_sec
        while rclpy.ok() and time.time() < deadline:
            if self.walker_ahead() is not None:
                self.get_logger().info("[wait] 前方有行人，停下等通过")
                self.wait_for_clear()

            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            q = yaw_to_quat(yaw)
            pose.pose.orientation.x, pose.pose.orientation.y = q[0], q[1]
            pose.pose.orientation.z, pose.pose.orientation.w = q[2], q[3]
            goal = NavigateToPose.Goal()
            goal.pose = pose

            self.get_logger().info(f"发送目标: x={x:.2f}, y={y:.2f}")
            send_future = self._client.send_goal_async(goal)
            rclpy.spin_until_future_complete(self, send_future, timeout_sec=15.0)
            goal_handle = send_future.result()
            if goal_handle is None or not goal_handle.accepted:
                self.get_logger().error("目标被拒绝")
                return False

            sent_at = time.time()
            result_future = goal_handle.get_result_async()
            canceled = False
            while rclpy.ok() and time.time() < deadline:
                rclpy.spin_once(self, timeout_sec=0.08)
                if (
                    self.walker_ahead() is not None
                    and time.time() - sent_at > 0.3
                    and time.time() - self._last_cancel > CANCEL_COOLDOWN
                ):
                    self.get_logger().info("[wait] 行驶中遇行人，停等")
                    self._last_cancel = time.time()
                    rclpy.spin_until_future_complete(
                        self, goal_handle.cancel_goal_async(), timeout_sec=3.0
                    )
                    self.wait_for_clear()
                    canceled = True
                    break
                if result_future.done():
                    break
                rclpy.spin_until_future_complete(self, result_future, timeout_sec=0.12)

            if canceled:
                if wait_for_ped:
                    return True
                continue
            wrapped = result_future.result() if result_future.done() else None
            if wrapped is None:
                return False
            ok = wrapped.status == 4
            self.get_logger().info(f"到达结果 status={wrapped.status} ({'成功' if ok else '失败'})")
            if ok and wait_for_ped:
                self.wait_until_ahead()
            return ok
        return False


def main():
    set_speed(VX_MAX)
    rclpy.init()
    node = MultiGoalNavigator()
    try:
        if not node.wait_server():
            return
        node.restore_robot()
        if not node.wait_ped_going_south():
            return
        node.restore_robot()
        success = 0
        for i, (x, y, yaw) in enumerate(WAYPOINTS, 1):
            node.get_logger().info(f"===== 第 {i}/{len(WAYPOINTS)} 个点 =====")
            if node.go_to(x, y, yaw, wait_for_ped=(i - 1) in WAIT_FOR_PED):
                success += 1
        node.get_logger().info(f"完成：成功 {success}/{len(WAYPOINTS)}")
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
