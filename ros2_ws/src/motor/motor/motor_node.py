#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Float64
from motor_control import DynamixelMXController
from utils import rpm_to_linear_velocity_mps
import utils as utils
import time

import sys

class MotorPublisherNode(Node):
    """ROS2 Node for controlling and monitoring a Dynamixel motor."""
    
    def __init__(self, controller):
        super().__init__('motor_publisher')
        self.controller = controller

        # Publishers for position, velocity, and goal positions
        self.position_pub = self.create_publisher(Int32, 'motor/present_position', 10)
        self.velocity_pub = self.create_publisher(Float64, 'motor/present_velocity', 10)
        self.goal_pub = self.create_publisher(Int32, 'motor/goal_position', 10)
        
        time.sleep(0.2)

        self.current_index = 0
        self.cleanup_called = False  # Prevent multiple cleanup calls

        # Timer for handling goal transitions (e.g., every 0.05 sec)
        self.move_timer = self.create_timer(0.1, self.auto_move_and_publish)

        # Dedicated timer for publishing present velocity as fast as possible (e.g., every 0.005 sec)
        self.velocity_timer = self.create_timer(0.001, self.publish_velocity_callback)

    def publish_present_position(self, present_position, goal_position):
        """Publishes the current and goal positions."""
        self.position_pub.publish(Int32(data=int(present_position)))
        self.goal_pub.publish(Int32(data=int(goal_position)))

    def publish_present_velocity(self, present_velocity):
        """Publishes the current velocity, ensuring it is an integer."""
        out_msg = Float64()
        out_msg.data = present_velocity
        self.velocity_pub.publish(out_msg)

    def publish_velocity_callback(self):
        """Callback to read and publish the motor's present velocity quickly."""
        raw_velocity = self.controller.get_present_velocity()
        rpm = utils.raw_to_rpm(raw_velocity)
        linear_velocity = rpm_to_linear_velocity_mps(rpm)
        self.publish_present_velocity(linear_velocity)

    def auto_move_and_publish(self):
        """Handles movement to the next goal position and publishes position updates."""
        if self.current_index >= len(self.controller.goal_positions):
            self.get_logger().info("✅ All goal positions reached. Shutting down...")
            time.sleep(0.5)
            self.destroy_timer(self.move_timer)
            self.get_logger().info("Closed first timer")
            self.destroy_timer(self.velocity_timer)
            self.get_logger().info("Closed second timer")
            self.controller.disable_torque()
            try:
                self.controller.close_port()
            except Exception as e:
                self.get_logger().warn(f"⚠️ [WARNING] Failed to close port: {e}")
            # self.cleanup_and_exit()
            self.destroy_node()       # Destroy this node.
            rclpy.shutdown()          # Shutdown the ROS context.
            sys.exit(0) 
            return

        goal_position = self.controller.goal_positions[self.current_index]
        # self.get_logger().info(f"🔹 Moving to goal position: {goal_position}")
        self.controller.set_goal_position(goal_position)

        present_position = self.controller.get_present_position()
        self.publish_present_position(present_position, goal_position)
        # self.get_logger().info(f"🟢 [ID:{self.controller.motor_id}] GoalPos: {goal_position}, PresPos: {present_position}")

        if abs(present_position - goal_position) <= self.controller.MOVING_STATUS_THRESHOLD:
            self.get_logger().info(f"✅ Reached Goal Position: {goal_position}")
            self.current_index += 1  # Move to the next position

    def cleanup_and_exit(self):
        """Handles shutdown: disables torque, closes the port, stops ROS2."""
        if self.cleanup_called:
            return
        self.cleanup_called = True

        self.get_logger().info("🔻 Disabling torque and closing the port...")
        self.controller.disable_torque()
        
        try:
            self.controller.close_port()
        except Exception as e:
            self.get_logger().warn(f"⚠️ [WARNING] Failed to close port: {e}")

        self.get_logger().info("🔻 Destroying ROS2 node and shutting down...")
        self.destroy_node()
        rclpy.shutdown()
        self.get_logger().info("🔴 Exiting program now...")
        sys.exit(0)

def main(args=None):
    rclpy.init(args=args)

    # Define default goal positions (modify as needed)
    # goal_positions = [2000, 500, 2500, 1000, 2500, 0]
    goal_positions = [750, 2000, 200, 1000, 2500, 0]
    # goal_positions = [0]  # Single goal position
    distances = utils.calc_goal_differences_in_m(goal_positions)
    total_distance = utils.calc_total_distance_in_m(goal_positions)

    print("Distances between goals (m):", distances)
    print("Total distance traveled (m):", total_distance)

    # Validate positions
    valid_positions = [pos for pos in goal_positions if 0 <= pos <= 3000]
    if not valid_positions:
        print("❌ No valid goal positions provided. Exiting...")
        return

    # Initialize motor controller
    controller = DynamixelMXController(goal_positions=valid_positions)
    controller.open_port()
    controller.set_position_limits()
    controller.set_vel_and_accel()
    controller.enable_torque()
    time.sleep(1)

    try:
        motor_node = MotorPublisherNode(controller)
        rclpy.spin(motor_node)
    except KeyboardInterrupt:
        print("\n🛑 [CTRL+C] Interrupt received. Cleaning up...")
        motor_node.cleanup_and_exit()
        

if __name__ == '__main__':
    main()
