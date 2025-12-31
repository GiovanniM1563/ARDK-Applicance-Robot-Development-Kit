import time
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import TransformException

def wait_for_services(node: Node, timeout_sec: float, *service_names: str) -> None:
    """
    Wait until all services in 'service_names' appear in the graph.
    """
    deadline = time.time() + timeout_sec
    while rclpy.ok() and time.time() < deadline:
        available = set(name for (name, _) in node.get_service_names_and_types())
        if all(s in available for s in service_names):
            return
        rclpy.spin_once(node, timeout_sec=0.1)
    
    available = set(name for (name, _) in node.get_service_names_and_types())
    missing = [s for s in service_names if s not in available]
    node.get_logger().error(f"Timeout waiting for services: {missing}. Available: {sorted(list(available))}")
    raise RuntimeError(f"Timeout waiting for services: {missing}")

def wait_for_topic(node: Node, timeout_sec: float, topic_name: str) -> None:
    """
    Wait until 'topic_name' appears in the graph.
    """
    deadline = time.time() + timeout_sec
    while rclpy.ok() and time.time() < deadline:
        topics = dict(node.get_topic_names_and_types())
        if topic_name in topics:
            return
        rclpy.spin_once(node, timeout_sec=0.1)
    raise RuntimeError(f"Timeout waiting for topic: {topic_name}")

def wait_for_tf_chain(node: Node, tf_buffer, timeout_sec: float, frame_map: str, frame_odom: str, frame_base: str) -> None:
    """
    Wait for map->odom AND odom->base_link transforms to be available.
    Requires the node to hold a Buffer/Listener.
    """
    deadline = time.time() + timeout_sec
    while rclpy.ok() and time.time() < deadline:
        try:
            tf_buffer.lookup_transform(frame_map, frame_odom, Time())
            tf_buffer.lookup_transform(frame_odom, frame_base, Time())
            return
        except TransformException:
            rclpy.spin_once(node, timeout_sec=0.1)
    raise RuntimeError(f"Timeout waiting for TF chain {frame_map}->{frame_odom}->{frame_base}")
