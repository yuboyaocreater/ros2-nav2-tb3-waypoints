#!/usr/bin/env python3
"""行人沿红框南北一直来回，不停车等车。"""

from __future__ import annotations

import subprocess
import threading
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Pose, PoseArray
from rclpy.node import Node
from rclpy.parameter import Parameter

ROOT = Path(__file__).resolve().parent
SDF = ROOT / "models" / "person.sdf"
CREATE = "/opt/ros/jazzy/lib/ros_gz_sim/create"
NAME = "demo_ped"

SOUTH = (0.55, -1.48, 0.85)
NORTH = (0.55, 1.48, 0.85)
# 按仿真时钟走，和 Nav2 同一套时间；略慢于车，车先到交叉口等人走进车头
SPEED = 0.28
DT = 0.04


def gz_req(service: str, reqtype: str, req: str, timeout_ms: int = 250) -> None:
    try:
        subprocess.run(
            [
                "gz",
                "service",
                "-s",
                service,
                "--reqtype",
                reqtype,
                "--reptype",
                "gz.msgs.Boolean",
                "--timeout",
                str(timeout_ms),
                "--req",
                req,
            ],
            capture_output=True,
            text=True,
            timeout=0.4,
        )
    except (subprocess.TimeoutExpired, FileNotFoundError):
        pass


def spawn() -> None:
    for name in (NAME, "walker1", "walker2"):
        gz_req(
            "/world/default/remove",
            "gz.msgs.Entity",
            f'name: "{name}" type: MODEL',
            timeout_ms=800,
        )
        time.sleep(0.2)
    x, y, z = NORTH
    try:
        r = subprocess.run(
            [
                CREATE,
                "-world",
                "default",
                "-file",
                str(SDF),
                "-name",
                NAME,
                "-x",
                str(x),
                "-y",
                str(y),
                "-z",
                str(z),
            ],
            capture_output=True,
            text=True,
            timeout=12,
        )
        print("[spawn]", ((r.stdout or "") + (r.stderr or "")).strip()[:180])
    except subprocess.TimeoutExpired:
        print("[spawn] 超时，后面仍按脚本坐标发布行人位置")


def main() -> None:
    spawn()
    rclpy.init()
    node = Node(
        "demo_walkers",
        parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)],
    )
    pub = node.create_publisher(PoseArray, "/demo_walkers", 10)
    xyz = list(NORTH)
    going_south = True
    pose_lock = threading.Lock()
    stop = threading.Event()

    def gz_loop() -> None:
        last = None
        while not stop.is_set():
            with pose_lock:
                cur = tuple(xyz)
            if cur != last:
                x, y, z = cur
                gz_req(
                    "/world/default/set_pose",
                    "gz.msgs.Pose",
                    f'name: "{NAME}" position {{ x: {x:.4f} y: {y:.4f} z: {z:.4f} }}',
                    timeout_ms=200,
                )
                last = cur
            time.sleep(0.03)

    threading.Thread(target=gz_loop, daemon=True).start()
    print("[walk] 南北一直来回（仿真时间）。Ctrl+C 停止。")
    n = 0
    prev = node.get_clock().now()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.02)
            now = node.get_clock().now()
            dt = (now - prev).nanoseconds * 1e-9
            prev = now
            if dt <= 0.0:
                time.sleep(0.02)
                continue
            if dt > 0.25:
                dt = DT
            step = SPEED * dt
            target = SOUTH if going_south else NORTH
            dy = target[1] - xyz[1]
            if abs(dy) <= step:
                xyz[1] = target[1]
                going_south = not going_south
            else:
                xyz[1] += step if dy > 0 else -step
            with pose_lock:
                x, y, z = xyz[0], xyz[1], xyz[2]
            arr = PoseArray()
            arr.header.stamp = now.to_msg()
            arr.header.frame_id = "map"
            p = Pose()
            p.position.x, p.position.y, p.position.z = x, y, z
            p.orientation.w = 1.0
            arr.poses.append(p)
            pub.publish(arr)
            if n % 25 == 0:
                print(f"[walk] ped=({x:.2f},{y:.2f})")
            n += 1
            time.sleep(0.02)
    except KeyboardInterrupt:
        print("\n[walk] 已停止")
    finally:
        stop.set()
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
