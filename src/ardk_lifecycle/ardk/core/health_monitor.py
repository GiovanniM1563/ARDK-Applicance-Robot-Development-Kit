"""
Health monitoring utilities for ARDK.
Provides continuous health checks for TF chain validity and service presence.
"""

import time
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import TransformException


def check_tf_freshness(node: Node, tf_buffer, max_age_sec: float = 2.0) -> bool:
    """
    Check if the TF chain (map→odom→base_link) is valid and fresh.
    Returns True if transforms exist and are < max_age_sec old.
    """
    try:
        now = node.get_clock().now()
        
        # Check map→odom (when in NAV mode, AMCL publishes this)
        try:
            t1 = tf_buffer.lookup_transform('map', 'odom', Time())
            stamp1 = Time.from_msg(t1.header.stamp)
            age1 = (now - stamp1).nanoseconds / 1e9
            if age1 > max_age_sec:
                return False
        except TransformException:
            # map→odom may not exist in MAPPING or IDLE mode, that's OK
            pass
        
        # Check odom→base_link (always required when hardware is running)
        t2 = tf_buffer.lookup_transform('odom', 'base_link', Time())
        stamp2 = Time.from_msg(t2.header.stamp)
        age2 = (now - stamp2).nanoseconds / 1e9
        if age2 > max_age_sec:
            return False
            
        return True
        
    except TransformException:
        return False
    except Exception:
        return False


def check_required_services(node: Node, mode: int) -> bool:
    """
    Check if required services are available based on current mode.
    Mode 0 = IDLE, 1 = MAPPING, 2 = NAVIGATION
    """
    available = set(name for (name, _) in node.get_service_names_and_types())
    
    # Core service always required
    core_services = ['/set_mode']
    
    if mode == 1:  # MAPPING
        required = core_services + ['/slam_toolbox/save_map']
    elif mode == 2:  # NAVIGATION
        required = core_services + [
            '/lifecycle_manager_localization/manage_nodes',
            '/lifecycle_manager_navigation/manage_nodes',
            '/map_server/load_map'
        ]
    else:  # IDLE
        required = core_services
    
    return all(s in available for s in required)
