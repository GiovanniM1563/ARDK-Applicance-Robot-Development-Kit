#!/usr/bin/env python3

import sys
import time
import rclpy
from rclpy.node import Node
from ardk_lifecycle.srv import SetMode
from ardk_lifecycle.msg import ARDKStatus

class CycleTester(Node):
    def __init__(self):
        super().__init__('cycle_tester')
        self.cli = self.create_client(SetMode, 'set_mode')
        self.sub = self.create_subscription(ARDKStatus, 'ardk_status', self.status_cb, 10)
        self.current_mode = -1
        self.mode_names = {0: "IDLE", 1: "MAPPING", 2: "NAVIGATION"}

    def status_cb(self, msg):
        self.current_mode = msg.mode
        # self.get_logger().info(f"Status update: Mode={self.mode_names.get(msg.mode, 'UNKNOWN')} Step={msg.transition_step}")

    def call_mode(self, mode):
        self.get_logger().info(f"Requesting {self.mode_names[mode]}...")
        req = SetMode.Request()
        req.target_mode = mode
        
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting...')
            
        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def run_cycle(self):
        # 1. Check start
        time.sleep(2)
        if self.current_mode != 0:
            self.get_logger().warn("Not in IDLE at start. Resetting...")
            self.call_mode(0)
            time.sleep(2)

        # 2. IDLE -> MAPPING
        self.get_logger().info("=== TEST 1: IDLE -> MAPPING ===")
        res = self.call_mode(1)
        if not res.success:
            self.get_logger().error(f"Failed: {res.message}")
            return
        self.get_logger().info("Success. Waiting 5s...")
        time.sleep(5)

        # 3. MAPPING -> NAVIGATION
        self.get_logger().info("=== TEST 2: MAPPING -> NAVIGATION ===")
        res = self.call_mode(2)
        if res.success:
             self.get_logger().info("Transition accepted. Waiting for completion...")
             # Wait for status to become 2
             for _ in range(20):
                 rclpy.spin_once(self, timeout_sec=1)
                 if self.current_mode == 2:
                     self.get_logger().info("Reached NAVIGATION mode verified by status topic.")
                     break
                 time.sleep(0.5)
        else:
             self.get_logger().error(f"Failed: {res.message}")
             return
             
        self.get_logger().info("Sitting in NAV for 5s...")
        time.sleep(5)

        # 4. NAVIGATION -> IDLE
        self.get_logger().info("=== TEST 3: NAVIGATION -> IDLE ===")
        res = self.call_mode(0)
        if not res.success:
            self.get_logger().error(f"Failed: {res.message}")
            return
        
        # Verify
        time.sleep(2)
        rclpy.spin_once(self)
        if self.current_mode == 0:
             self.get_logger().info("Returned to IDLE successfully.")
             self.get_logger().info("=== CYCLE TEST PASSED ===")
        else:
             self.get_logger().error("Failed to return to IDLE status.")

def main():
    rclpy.init()
    tester = CycleTester()
    try:
        tester.run_cycle()
    finally:
        tester.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
