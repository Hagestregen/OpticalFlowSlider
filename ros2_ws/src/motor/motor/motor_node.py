#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Float64,Float64MultiArray
from motor_control import DynamixelMXController
from utils import rpm_to_linear_velocity_mps
import utils
import sys

class MotorPublisherNode(Node):
    """
    A ROS2 Node for controlling and monitoring a Dynamixel motor.
    
    This node moves the motor through a sequence of goal positions, changes speed at specific points,
    pauses for 0.5 seconds without moving when speed changes, and continuously publishes the motor's
    position and velocity, even during pauses.
    """
    
    def __init__(self, controller):
        """
        Initialize the MotorPublisherNode.
        
        Args:
            controller (DynamixelMXController): The motor controller instance.
        """
        super().__init__('motor_publisher')
        self.controller = controller

        # Publishers for motor state
        self.position_pub = self.create_publisher(Int32, 'motor/present_position', 10)
        self.velocity_pub = self.create_publisher(Float64MultiArray, 'motor/present_velocity', 10)
        self.goal_pub = self.create_publisher(Int32, 'motor/goal_position', 10)
        
        # State variables
        self.current_index = 0  # Index of the current goal position
        self.pausing = False    # Flag indicating if the motor is pausing
        self.pause_start_time = None  # Timestamp when pause starts

        # Timers
        self.move_timer = self.create_timer(0.1, self.auto_move_and_publish)  # 10 Hz for movement
        self.velocity_timer = self.create_timer(0.001, self.publish_velocity_callback)  # 1000 Hz for velocity

    def publish_present_position(self, present_position, goal_position):
        """
        Publish the current and goal positions of the motor.
        
        Args:
            present_position (int): The motor's current position.
            goal_position (int): The motor's current goal position.
        """
        self.position_pub.publish(Int32(data=int(present_position)))
        self.goal_pub.publish(Int32(data=int(goal_position)))

    def publish_present_velocity(self, present_velocity):
        """
        Publish the motor's current velocity.
        
        Args:
            present_velocity (float): The motor's velocity in meters per second.
        """
        
        current_time = self.get_clock().now().nanoseconds / 1e9
        # out_msg = Float64()
        out_msg = Float64MultiArray()
        out_msg.data = [float(present_velocity), current_time]
        # out_msg.data = float(present_velocity)
        self.velocity_pub.publish(out_msg)

    def publish_velocity_callback(self):
        """Callback to publish the motor's velocity at a high rate."""
        raw_velocity = self.controller.get_present_velocity()
        rpm = utils.raw_to_rpm(raw_velocity)
        linear_velocity = rpm_to_linear_velocity_mps(rpm)
        self.publish_present_velocity(linear_velocity)

    def auto_move_and_publish(self):
        """
        Main callback to control motor movement and publish its state.
        
        - Moves the motor to the next goal position.
        - Pauses for 0.5 seconds at the previous goal when changing speed (at indices 5 and 8).
        - Continuously publishes position and goal data, even during pauses.
        - Shuts down the node after all goals are reached.
        """
        # Check if all goal positions have been reached
        if self.current_index >= len(self.controller.goal_positions):
            self.get_logger().info("✅ All goal positions reached. Shutting down...")
            self.destroy_timer(self.move_timer)
            self.destroy_timer(self.velocity_timer)
            self.controller.disable_torque()
            self.controller.close_port()
            self.destroy_node()
            rclpy.shutdown()
            sys.exit(0)
            return

        # Handle pausing state
        if self.pausing:
            current_time = self.get_clock().now()
            elapsed = (current_time - self.pause_start_time).nanoseconds / 1e9  # Convert to seconds
            if elapsed >= 0.5:
                self.pausing = False
                goal_position = self.controller.goal_positions[self.current_index]
                self.controller.set_goal_position(goal_position)
                # self.get_logger().info(f"🔹 Moving to goal position: {goal_position}")
        else:
            # Handle speed changes and initiate pause before moving to specific goals
            if self.current_index == 2:
                self.controller.set_vel_and_accel(25, 10)
                self.get_logger().info("🔹 Set velocity and acceleration to: 25 and 10")
                self.pausing = True
                self.pause_start_time = self.get_clock().now()
            if self.current_index == 3:
                self.controller.set_vel_and_accel(150, 35)
                self.get_logger().info("🔹 Set velocity and acceleration to: 150 and 35")
                self.pausing = True
                self.pause_start_time = self.get_clock().now()
            elif self.current_index == 5:
                self.controller.set_vel_and_accel(350, 50)
                self.get_logger().info("🔹 Set velocity and acceleration to: 350 and 50")
                self.pausing = True
                self.pause_start_time = self.get_clock().now()
            elif self.current_index == 8:
                self.controller.set_vel_and_accel(25, 10)
                self.get_logger().info("🔹 Set velocity and acceleration to: 25 and 10")
                self.pausing = True
                self.pause_start_time = self.get_clock().now()
            else:
                goal_position = self.controller.goal_positions[self.current_index]
                self.controller.set_goal_position(goal_position)
                # self.get_logger().info(f" Moving to goal position: {goal_position}")

        # Publish current state
        present_position = self.controller.get_present_position()
        # Note: Assumes get_goal_position() exists; if not, use the last set goal_position
        # goal_position = self.controller.get_goal_position()
        goal_position = self.controller.goal_positions[self.current_index]
        
        self.publish_present_position(present_position, goal_position)

        # Check if the current goal is reached (only when not pausing)
        if not self.pausing and abs(present_position - goal_position) <= self.controller.MOVING_STATUS_THRESHOLD:
            self.get_logger().info(f"✅ Reached Goal Position: {goal_position}")
            self.current_index += 1  # Move to the next position

def main(args=None):
    """Main function to initialize and run the ROS2 node."""
    rclpy.init(args=args)

    # Define the sequence of goal positions
    goal_positions = [750, 2000, 200, 2000, 500, 2400, 500, 1000, 0]
    
    # Initialize the motor controller
    controller = DynamixelMXController(goal_positions=goal_positions)
    controller.open_port()
    controller.set_position_limits()
    controller.set_vel_and_accel()  # Set default velocity and acceleration
    controller.enable_torque()

    # Create and spin the node
    motor_node = MotorPublisherNode(controller)
    rclpy.spin(motor_node)

if __name__ == '__main__':
    main()

# #!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import Int32, Float64
# from motor_control import DynamixelMXController
# from utils import rpm_to_linear_velocity_mps
# import utils as utils
# import time

# import sys

# class MotorPublisherNode(Node):
#     """ROS2 Node for controlling and monitoring a Dynamixel motor."""
    
#     def __init__(self, controller):
#         super().__init__('motor_publisher')
#         self.controller = controller

#         # Publishers for position, velocity, and goal positions
#         self.position_pub = self.create_publisher(Int32, 'motor/present_position', 10)
#         self.velocity_pub = self.create_publisher(Float64, 'motor/present_velocity', 10)
#         self.goal_pub = self.create_publisher(Int32, 'motor/goal_position', 10)
        
#         time.sleep(0.5)

#         self.current_index = 0
#         self.cleanup_called = False  # Prevent multiple cleanup calls

#         # Timer for handling goal transitions (e.g., every 0.05 sec)
#         self.move_timer = self.create_timer(0.1, self.auto_move_and_publish)

#         # Dedicated timer for publishing present velocity as fast as possible (e.g., every 0.005 sec)
#         self.velocity_timer = self.create_timer(0.001, self.publish_velocity_callback)

#     def publish_present_position(self, present_position, goal_position):
#         """Publishes the current and goal positions."""
#         self.position_pub.publish(Int32(data=int(present_position)))
#         self.goal_pub.publish(Int32(data=int(goal_position)))

#     def publish_present_velocity(self, present_velocity):
#         """Publishes the current velocity, ensuring it is an integer."""
#         out_msg = Float64()
#         out_msg.data = present_velocity
#         self.velocity_pub.publish(out_msg)

#     def publish_velocity_callback(self):
#         """Callback to read and publish the motor's present velocity quickly."""
#         raw_velocity = self.controller.get_present_velocity()
#         rpm = utils.raw_to_rpm(raw_velocity)
#         linear_velocity = rpm_to_linear_velocity_mps(rpm)
#         self.publish_present_velocity(linear_velocity)

#     def auto_move_and_publish(self):
#         """Handles movement to the next goal position and publishes position updates."""
#         if self.current_index >= len(self.controller.goal_positions):
#             self.get_logger().info("✅ All goal positions reached. Shutting down...")
#             time.sleep(0.5)
#             self.destroy_timer(self.move_timer)
#             self.get_logger().info("Closed first timer")
#             self.destroy_timer(self.velocity_timer)
#             self.get_logger().info("Closed second timer")
#             self.controller.disable_torque()
#             try:
#                 self.controller.close_port()
#             except Exception as e:
#                 self.get_logger().warn(f"⚠️ [WARNING] Failed to close port: {e}")
#             # self.cleanup_and_exit()
#             self.destroy_node()       # Destroy this node.
#             rclpy.shutdown()          # Shutdown the ROS context.
#             sys.exit(0) 
#             return

#         goal_position = self.controller.goal_positions[self.current_index]
#         # self.get_logger().info(f"🔹 Moving to goal position: {goal_position}")
#         if self.current_index == 5:
#                 self.controller.set_vel_and_accel(350, 50)
#                 self.get_logger().info("🔹 Set velocity and acceleration to: 400 and 50")
#                 time.sleep(0.5)
#         elif self.current_index == 8:
#                 self.controller.set_vel_and_accel(50, 20)
#                 self.get_logger().info("🔹 Set velocity and acceleration to: 50 and 20")
#                 time.sleep(0.5)
#         self.controller.set_goal_position(goal_position)

#         present_position = self.controller.get_present_position()
#         self.publish_present_position(present_position, goal_position)
#         # self.get_logger().info(f"🟢 [ID:{self.controller.motor_id}] GoalPos: {goal_position}, PresPos: {present_position}")

#         if abs(present_position - goal_position) <= self.controller.MOVING_STATUS_THRESHOLD:
#             self.get_logger().info(f"✅ Reached Goal Position: {goal_position}")
#             self.current_index += 1  # Move to the next position

#     def cleanup_and_exit(self):
#         """Handles shutdown: disables torque, closes the port, stops ROS2."""
#         if self.cleanup_called:
#             return
#         self.cleanup_called = True

#         self.get_logger().info("🔻 Disabling torque and closing the port...")
#         self.controller.disable_torque()
        
#         try:
#             self.controller.close_port()
#         except Exception as e:
#             self.get_logger().warn(f"⚠️ [WARNING] Failed to close port: {e}")

#         self.get_logger().info("🔻 Destroying ROS2 node and shutting down...")
#         self.destroy_node()
#         rclpy.shutdown()
#         self.get_logger().info("🔴 Exiting program now...")
#         sys.exit(0)

# def main(args=None):
#     rclpy.init(args=args)

#     # Define default goal positions (modify as needed)
#     # goal_positions = [2000, 500, 2500, 1000, 2500, 0]
#     goal_positions = [750, 2000, 200, 2000, 500, 2500, 500, 1000, 0]
#     # goal_positions = [0]  # Single goal position
#     # distances = utils.calc_goal_differences_in_m(goal_positions)
#     # total_distance = utils.calc_total_distance_in_m(goal_positions)

#     # print("Distances between goals (m):", distances)
#     # print("Total distance traveled (m):", total_distance)

#     # Validate positions
#     valid_positions = [pos for pos in goal_positions if 0 <= pos <= 3000]
#     if not valid_positions:
#         print("❌ No valid goal positions provided. Exiting...")
#         return

#     # Initialize motor controller
#     controller = DynamixelMXController(goal_positions=valid_positions)
#     controller.open_port()
#     controller.set_position_limits()
#     controller.set_vel_and_accel()
#     controller.enable_torque()
#     time.sleep(0.5)

#     try:
#         motor_node = MotorPublisherNode(controller)
#         rclpy.spin(motor_node)
#     except KeyboardInterrupt:
#         print("\n🛑 [CTRL+C] Interrupt received. Cleaning up...")
#         motor_node.cleanup_and_exit()
        

# if __name__ == '__main__':
#     main()
