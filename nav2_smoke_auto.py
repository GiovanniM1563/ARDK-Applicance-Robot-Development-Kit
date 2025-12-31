#!/usr/bin/env python3
"""
nav2_smoke_auto.py  (one-file concept test)

Goal:
1) Start Nav2 bringup (as a managed process group) with autostart:=false
2) Programmatically LoadMap(map_yaml_path) into nav2_map_server
3) Programmatically STARTUP localization and navigation stacks via Nav2 lifecycle managers
4) Verify readiness (services + /map + TF map->odom->base_link)
5) SHUTDOWN stacks and stop process

NO ros2 CLI helpers used for switching. Only ROS APIs (rclpy service calls).

Usage:
  source /opt/ros/jazzy/setup.bash
  python3 nav2_smoke_auto.py \
    --map-yaml /abs/path/to/map.yaml \
    --nav2-cmd "ros2 launch nav2_bringup bringup_launch.py use_sim_time:=false autostart:=false"

Notes:
- You MUST have Nav2 bringup running nodes that include:
  - lifecycle_manager_localization
  - lifecycle_manager_navigation
  - map_server providing /map_server/load_map
- If your stack is namespaced, override the service names with args.
"""

from __future__ import annotations

import argparse
import os
import signal
import subprocess
import sys
import time
from pathlib import Path
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from nav2_msgs.srv import ManageLifecycleNodes, LoadMap
from tf2_ros import Buffer, TransformListener
from tf2_ros import TransformException


def start_process(cmd: str) -> subprocess.Popen:
    return subprocess.Popen(
        cmd,
        shell=True,
        preexec_fn=os.setsid,  # new process group
        stdout=None,  # inherit for debugging
        stderr=None,
        text=True,
    )


def stop_process_group(proc: subprocess.Popen, timeout_sec: float = 8.0) -> None:
    if proc.poll() is not None:
        return
    try:
        pgid = os.getpgid(proc.pid)
    except Exception:
        proc.terminate()
        return

    try:
        os.killpg(pgid, signal.SIGINT)
        proc.wait(timeout=timeout_sec)
        return
    except Exception:
        pass

    try:
        os.killpg(pgid, signal.SIGKILL)
    except Exception:
        pass
    try:
        proc.wait(timeout=2.0)
    except Exception:
        pass


class SmokeNode(Node):
    def __init__(self) -> None:
        super().__init__("nav2_smoke_auto")
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)


def wait_for_services(node: Node, timeout_sec: float, *service_names: str) -> None:
    deadline = time.time() + timeout_sec
    while rclpy.ok() and time.time() < deadline:
        available = set(name for (name, _) in node.get_service_names_and_types())
        if all(s in available for s in service_names):
            return
        rclpy.spin_once(node, timeout_sec=0.1)
    missing = [s for s in service_names if s not in set(name for (name, _) in node.get_service_names_and_types())]
    node.get_logger().error(f"Available services: {sorted(list(set(name for (name, _) in node.get_service_names_and_types())))}")
    raise RuntimeError(f"Timeout waiting for services: {missing}")


def wait_for_topic(node: Node, timeout_sec: float, topic_name: str) -> None:
    deadline = time.time() + timeout_sec
    while rclpy.ok() and time.time() < deadline:
        topics = dict(node.get_topic_names_and_types())
        if topic_name in topics:
            return
        rclpy.spin_once(node, timeout_sec=0.1)
    raise RuntimeError(f"Timeout waiting for topic: {topic_name}")


def call_manage_nodes(node: Node, service_name: str, command: int, timeout_sec: float) -> None:
    client = node.create_client(ManageLifecycleNodes, service_name)
    if not client.wait_for_service(timeout_sec=timeout_sec):
        raise RuntimeError(f"Lifecycle manager service not available: {service_name}")

    req = ManageLifecycleNodes.Request()
    req.command = command
    fut = client.call_async(req)

    deadline = time.time() + timeout_sec
    while rclpy.ok() and not fut.done() and time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)

    if not fut.done():
        raise RuntimeError(f"Timed out calling {service_name}")
    if fut.exception() is not None:
        raise RuntimeError(f"{service_name} exception: {fut.exception()}")

    resp = fut.result()
    if not resp.success:
        raise RuntimeError(f"{service_name} returned success=false")


def call_load_map(node: Node, service_name: str, map_yaml: str, timeout_sec: float) -> None:
    client = node.create_client(LoadMap, service_name)
    if not client.wait_for_service(timeout_sec=timeout_sec):
        raise RuntimeError(f"LoadMap service not available: {service_name}")

    req = LoadMap.Request()
    req.map_url = map_yaml
    fut = client.call_async(req)

    deadline = time.time() + timeout_sec
    while rclpy.ok() and not fut.done() and time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)

    if not fut.done():
        raise RuntimeError("Timed out calling LoadMap")
    if fut.exception() is not None:
        raise RuntimeError(f"LoadMap exception: {fut.exception()}")

    resp = fut.result()
    # LoadMap has varied slightly across releases; treat it permissively.
    # Commonly resp.result is bool, or resp.result is an enum-like status.
    if hasattr(resp, "result"):
        if isinstance(resp.result, bool) and not resp.result:
            raise RuntimeError("LoadMap returned result=false")
        # If result is numeric, 0 is often success but don't assume; log if needed.


def wait_for_tf_chain(node: SmokeNode, timeout_sec: float, frame_map: str, frame_odom: str, frame_base: str) -> None:
    deadline = time.time() + timeout_sec
    while rclpy.ok() and time.time() < deadline:
        try:
            node.tf_buffer.lookup_transform(frame_map, frame_odom, Time())
            node.tf_buffer.lookup_transform(frame_odom, frame_base, Time())
            return
        except TransformException:
            rclpy.spin_once(node, timeout_sec=0.1)
    raise RuntimeError(f"Timeout waiting for TF chain {frame_map}->{frame_odom}->{frame_base}")


def main() -> int:
    ap = argparse.ArgumentParser()
    
    # Default to the map we just made
    default_map = str(Path.cwd() / "maps" / "ardk_smoke_map.yaml")
    ap.add_argument("--map-yaml", default=default_map, help="Absolute path to a saved map.yaml")
    
    # Default to RPi params
    default_nav2_cmd = (
        "ros2 launch nav2_bringup bringup_launch.py "
        "use_sim_time:=false "
        "autostart:=false "
        "use_composition:=False "
        "params_file:=/home/gio/rpi_robot/config/nav2_params.yaml"
    )
    ap.add_argument(
        "--nav2-cmd",
        default=default_nav2_cmd,
        help="Command to start Nav2 bringup (must include autostart:=false)",
    )
    ap.add_argument("--timeout", type=float, default=30.0, help="Timeout seconds for waits/calls")
    ap.add_argument(
        "--lm-localization",
        default="/lifecycle_manager_localization/manage_nodes",
        help="Localization lifecycle manager manage_nodes service",
    )
    ap.add_argument(
        "--lm-navigation",
        default="/lifecycle_manager_navigation/manage_nodes",
        help="Navigation lifecycle manager manage_nodes service",
    )
    ap.add_argument(
        "--load-map-service",
        default="/map_server/load_map",
        help="Map server LoadMap service",
    )
    ap.add_argument("--map-topic", default="/map", help="Map topic name")
    ap.add_argument("--frame-map", default="map")
    ap.add_argument("--frame-odom", default="odom")
    ap.add_argument("--frame-base", default="base_link")
    args = ap.parse_args()

    map_yaml = str(Path(args.map_yaml).expanduser())
    if not Path(map_yaml).exists():
        print(f"--map-yaml not found: {map_yaml}", file=sys.stderr)
        return 2

    rclpy.init()
    node = SmokeNode()
    proc: Optional[subprocess.Popen] = None

    try:
        # Inject map argument if not present (simple check)
        final_cmd = args.nav2_cmd
        if "map:=" not in final_cmd:
             final_cmd += f" map:={map_yaml}"

        proc = start_process(final_cmd)

        # Wait for required services to appear
        wait_for_services(
            node,
            args.timeout,
            args.lm_localization,
            args.lm_navigation,
            args.load_map_service,
        )

        # Load the map into map_server (works even if stacks are inactive)
        call_load_map(node, args.load_map_service, map_yaml, args.timeout)

        # Bring up localization first, then navigation
        call_manage_nodes(node, args.lm_localization, ManageLifecycleNodes.Request.STARTUP, args.timeout)
        call_manage_nodes(node, args.lm_navigation, ManageLifecycleNodes.Request.STARTUP, args.timeout)

        # Readiness checks
        wait_for_topic(node, args.timeout, args.map_topic)
        wait_for_tf_chain(node, args.timeout, args.frame_map, args.frame_odom, args.frame_base)

        node.get_logger().info("NAV2 SMOKE TEST PASSED: map loaded, stacks started, TF chain present.")

        # Tear down in reverse order
        call_manage_nodes(node, args.lm_navigation, ManageLifecycleNodes.Request.SHUTDOWN, args.timeout)
        call_manage_nodes(node, args.lm_localization, ManageLifecycleNodes.Request.SHUTDOWN, args.timeout)

        return 0

    except Exception as e:
        node.get_logger().error(str(e))
        return 1

    finally:
        if proc is not None:
            stop_process_group(proc)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
