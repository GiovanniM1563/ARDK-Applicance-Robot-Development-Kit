import rclpy
from rclpy.node import Node
from nav2_msgs.srv import ManageLifecycleNodes, LoadMap
from ardk.core.process_group import start_process, stop_process_group
import subprocess
import time

# Service Names
SRV_LOC = "/lifecycle_manager_localization/manage_nodes"
SRV_NAV = "/lifecycle_manager_navigation/manage_nodes"
SRV_LOAD_MAP = "/map_server/load_map"


def start(nav2_cmd: str) -> subprocess.Popen:
    """
    Launch Nav2 stack (bringup).
    Must contain 'autostart:=true' if you want it to come up fully, 
    OR 'autostart:=false' if you want manual control via startup_loc_nav.
    """
    return start_process(nav2_cmd, stdout=None, stderr=None)


def stop(handle: subprocess.Popen) -> None:
    """
    Stop the Nav2 process group.
    """
    stop_process_group(handle)


def load_map(node: Node, map_yaml: str, timeout_sec: float = 5.0, client=None) -> None:
    """
    Call map_server/load_map.
    """
    if client is None:
        client = node.create_client(LoadMap, SRV_LOAD_MAP)
        
    if not client.wait_for_service(timeout_sec=timeout_sec):
        raise RuntimeError(f"Service not available: {SRV_LOAD_MAP}")
    
    req = LoadMap.Request()
    req.map_url = map_yaml
    fut = client.call_async(req)
    if node.executor:
        node.get_logger().info(f"DEBUG: Using executor loop for LoadMap")
        start_time = time.time()
        while not fut.done():
            if time.time() - start_time > timeout_sec:
                break
            time.sleep(0.1)
    else:
        node.get_logger().info(f"DEBUG: Using spin_until for LoadMap")
        rclpy.spin_until_future_complete(node, fut, timeout_sec=timeout_sec)
    
    if not fut.done():
        raise RuntimeError("Timed out calling LoadMap")
    
    resp = fut.result()
    if hasattr(resp, "result"):
        if isinstance(resp.result, bool) and not resp.result:
            raise RuntimeError("LoadMap failed (bool=False)")
        if isinstance(resp.result, int) and resp.result != 0:
            raise RuntimeError(f"LoadMap failed with code {resp.result}")


def startup_localization(node: Node, timeout_sec: float = 20.0, client=None) -> None:
    _manage_lifecycle(node, SRV_LOC, ManageLifecycleNodes.Request.STARTUP, timeout_sec, client)


def shutdown_localization(node: Node, timeout_sec: float = 20.0, client=None) -> None:
    _manage_lifecycle(node, SRV_LOC, ManageLifecycleNodes.Request.SHUTDOWN, timeout_sec, client)


def startup_navigation(node: Node, timeout_sec: float = None, client=None) -> None:
    _manage_lifecycle(node, SRV_NAV, ManageLifecycleNodes.Request.STARTUP, timeout_sec, client)


def shutdown_navigation(node: Node, timeout_sec: float = 20.0, client=None) -> None:
    _manage_lifecycle(node, SRV_NAV, ManageLifecycleNodes.Request.SHUTDOWN, timeout_sec, client)


def startup_loc_nav(node: Node, timeout_sec: float = 10.0, client_loc=None, client_nav=None) -> None:
    """
    Start localization then navigation lifecycle managers.
    """
    startup_localization(node, timeout_sec, client=client_loc)
    startup_navigation(node, timeout_sec, client=client_nav)


def shutdown_loc_nav(node: Node, timeout_sec: float = 10.0, client_loc=None, client_nav=None) -> None:
    """
    Shutdown navigation then localization lifecycle managers.
    """
    shutdown_navigation(node, timeout_sec, client=client_nav)
    shutdown_localization(node, timeout_sec, client=client_loc)


def _manage_lifecycle(node: Node, service_name: str, command: int, timeout_sec: float, client=None) -> None:
    if client is None:
        client = node.create_client(ManageLifecycleNodes, service_name)
    
    if not client.wait_for_service(timeout_sec=timeout_sec):
        raise RuntimeError(f"Lifecycle service not available: {service_name}")
        
    req = ManageLifecycleNodes.Request()
    req.command = command
    
    start_time = time.time()
    node.get_logger().info(f"Calling {service_name} (Deterministic Wait)...")
    fut = client.call_async(req)
    
    # Wait loop with logging
    while not fut.done():
        if node.executor:
            time.sleep(1.0)
        else:
            rclpy.spin_until_future_complete(node, fut, timeout_sec=1.0)
        
        elapsed = time.time() - start_time
        if elapsed > 10.0 and int(elapsed) % 5 == 0:
            node.get_logger().info(f"Waiting for Lifecycle Transition ({str(service_name)})... Elapsed: {int(elapsed)}s")

    resp = fut.result()
    if not resp.success:
        raise RuntimeError(f"{service_name} returned success=False")
