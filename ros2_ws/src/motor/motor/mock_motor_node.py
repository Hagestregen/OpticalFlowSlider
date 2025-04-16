#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Float64, Float64MultiArray
from sensor_msgs.msg import Imu
from motor_control import DynamixelMXController
from utils import rpm_to_linear_velocity_mps
import utils
import sys
from rclpy.time import Time
import csv  # Added for CSV writing

class MotorPublisherNode(Node):
    def __init__(self, controller):
        super().__init__('motor_publisher')
        self.controller = controller
        self.position_pub = self.create_publisher(Int32, 'motor/present_position', 10)
        self.velocity_pub = self.create_publisher(Float64MultiArray, 'motor/present_velocity', 10)
        self.goal_pub = self.create_publisher(Int32, 'motor/goal_position', 10)
        self.current_index = 0
        self.pausing = False
        self.pause_start_time = None
        self.imu_sub = self.create_subscription(
            Imu,
            '/inertialsense/imu',
            self.imu_callback,
            10
        )
        self.waiting_for_motion = False
        self.t_command = None
        self.t_velocity_change = None
        self.accel_threshold = 0.07  # Adjust this after testing (e.g., 0.1 m/s²)
        self.velocity_threshold = 0.01
        self.move_timer = self.create_timer(0.1, self.auto_move_and_publish)
        self.velocity_timer = self.create_timer(0.001, self.publish_velocity_callback)
        
        # Added for synchronized logging
        self.latest_velocity = 0.0
        self.latest_accel_x = 0.0
        self.data_log = []  # List to store (timestamp, velocity, accel_x)
        self.log_timer = self.create_timer(0.01, self.log_data_callback)  # 100 Hz logging

    def publish_present_position(self, present_position, goal_position):
        self.position_pub.publish(Int32(data=int(present_position)))
        self.goal_pub.publish(Int32(data=int(goal_position)))

    def publish_present_velocity(self, present_velocity):
        current_time = self.get_clock().now().nanoseconds / 1e9
        out_msg = Float64MultiArray()
        out_msg.data = [float(present_velocity), current_time]
        self.velocity_pub.publish(out_msg)
        # Update latest velocity
        self.latest_velocity = present_velocity
        if self.waiting_for_motion and self.t_velocity_change is None:
            if abs(present_velocity) > self.velocity_threshold:
                self.t_velocity_change = self.get_clock().now()
                delay_command_to_velocity = (self.t_velocity_change - self.t_command).nanoseconds / 1e9
                self.get_logger().info(
                    f"Velocity change detected at {self.t_velocity_change.nanoseconds / 1e9}, "
                    f"delay from command: {delay_command_to_velocity} seconds"
                )

    def publish_velocity_callback(self):
        raw_velocity = self.controller.get_present_velocity()
        rpm = utils.raw_to_rpm(raw_velocity)
        linear_velocity = rpm_to_linear_velocity_mps(rpm)
        self.publish_present_velocity(linear_velocity)

    def auto_move_and_publish(self):
        if self.current_index >= len(self.controller.goal_positions):
            self.get_logger().info("✅ All goal positions reached. Shutting down...")
            # Write synchronized data to CSV
            self.data_log.sort(key=lambda x: x[0])  # Sort by timestamp
            with open('synchronized_data.csv', 'w', newline='') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow(['timestamp', 'velocity', 'accel_x'])
                for entry in self.data_log:
                    writer.writerow(entry)
            self.get_logger().info("Data saved to 'synchronized_data.csv'")
            self.destroy_timer(self.move_timer)
            self.destroy_timer(self.velocity_timer)
            self.destroy_timer(self.log_timer)  # Destroy logging timer
            self.controller.disable_torque()
            self.controller.close_port()
            self.destroy_node()
            rclpy.shutdown()
            sys.exit(0)
            return
        if self.pausing:
            current_time = self.get_clock().now()
            elapsed = (current_time - self.pause_start_time).nanoseconds / 1e9
            if elapsed >= 0.5:
                self.pausing = False
                goal_position = self.controller.goal_positions[self.current_index]
                self.t_command = self.get_clock().now()
                self.waiting_for_motion = True
                self.t_velocity_change = None
                self.controller.set_goal_position(goal_position)
                self.get_logger().info(
                    f"Moving to goal position {goal_position} after pause at time "
                    f"{self.t_command.nanoseconds / 1e9}"
                )
        else:
            if self.current_index == 2:
                self.controller.set_vel_and_accel(25, 10)
                self.get_logger().info("Set velocity and acceleration to: 25 and 10")
                self.pausing = True
                self.pause_start_time = self.get_clock().now()
            elif self.current_index == 3:
                self.controller.set_vel_and_accel(150, 35)
                self.get_logger().info("Set velocity and acceleration to: 150 and 35")
                self.pausing = True
                self.pause_start_time = self.get_clock().now()
            elif self.current_index == 5:
                self.controller.set_vel_and_accel(350, 50)
                self.get_logger().info("Set velocity and acceleration to: 350 and 50")
                self.pausing = True
                self.pause_start_time = self.get_clock().now()
            elif self.current_index == 8:
                self.controller.set_vel_and_accel(25, 10)
                self.get_logger().info("Set velocity and acceleration to: 25 and 10")
                self.pausing = True
                self.pause_start_time = self.get_clock().now()
            else:
                goal_position = self.controller.goal_positions[self.current_index]
                self.t_command = self.get_clock().now()
                self.waiting_for_motion = True
                self.t_velocity_change = None
                self.controller.set_goal_position(goal_position)
                self.get_logger().info(
                    f"🔹 Moving to goal position {goal_position} at time "
                    f"{self.t_command.nanoseconds / 1e9}"
                )
        present_position = self.controller.get_present_position()
        goal_position = self.controller.goal_positions[self.current_index]
        self.publish_present_position(present_position, goal_position)
        if not self.pausing and abs(present_position - goal_position) <= self.controller.MOVING_STATUS_THRESHOLD:
            self.get_logger().info(f"Reached Goal Position: {goal_position}")
            self.current_index += 1

    def imu_callback(self, msg):
        t_motion = self.get_clock().now()
        accel_x = msg.linear_acceleration.x
        # Update latest IMU data
        self.latest_accel_x = accel_x
        if self.waiting_for_motion and self.t_velocity_change is not None:
            self.get_logger().info(f"t_motion: {t_motion.nanoseconds / 1e9}, t_velocity_change: {self.t_velocity_change.nanoseconds / 1e9}")
            if t_motion > self.t_velocity_change:
                if abs(accel_x) > self.accel_threshold:
                    delay = (t_motion - self.t_velocity_change).nanoseconds / 1e9
                    self.get_logger().info(f"IMU motion detected, delay: {delay} s")
                    self.waiting_for_motion = False
                else:
                    self.get_logger().info(f"accel_x: {accel_x} below threshold")
            else:
                self.get_logger().info("Timestamp too early")
        else:
            self.get_logger().info("Not waiting or no velocity change")

    def log_data_callback(self):
        """Callback to log velocity and IMU data at a fixed timestep."""
        current_time = self.get_clock().now().nanoseconds / 1e9
        self.data_log.append((current_time, self.latest_velocity, self.latest_accel_x))

def main(args=None):
    rclpy.init(args=args)
    goal_positions = [750, 0]
    controller = DynamixelMXController(goal_positions=goal_positions)
    controller.open_port()
    controller.set_position_limits()
    controller.set_vel_and_accel(0, 0)
    controller.enable_torque()
    motor_node = MotorPublisherNode(controller)
    rclpy.spin(motor_node)

if __name__ == '__main__':
    main()