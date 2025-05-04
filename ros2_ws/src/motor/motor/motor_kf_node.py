#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Float64, Float64MultiArray
from geometry_msgs.msg import TwistStamped
from motor_control import DynamixelMXController
from utils import rpm_to_linear_velocity_mps
import utils
import sys
import time

class MotorPublisherNode(Node):
    """
    A ROS2 Node for controlling and monitoring a Dynamixel motor.
    
    This node executes a sequence of actions (move, set_speed, pause), publishes motor state,
    and handles pauses without moving while continuously publishing data.
    """
    
    def __init__(self, controller, sequence):
        """
        Initialize the MotorPublisherNode.
        
        Args:
            controller (DynamixelMXController): The motor controller instance.
            sequence (list): List of actions to perform.
        """
        super().__init__('motor_publisher')
        self.controller = controller
        self.sequence = sequence  # Store the sequence of actions

        # Publishers for motor state
        self.position_pub = self.create_publisher(Int32, 'motor/present_position', 10)
        # self.velocity_pub = self.create_publisher(TwistStamped, 'motor/present_velocity', 10)
        self.goal_pub = self.create_publisher(Int32, 'motor/goal_position', 10)
        
        self.vel_pub = self.create_publisher(TwistStamped, 'motor/present_velocity', 10)
        
        # State variables
        self.current_index = 0  # Index of the current action in the sequence
        self.pausing = False    # Flag indicating if the motor is pausing
        self.pause_start_time = None  # Timestamp when pause starts
        self.moving = False     # Flag indicating if moving to a position

        # Timers
        self.start_time = time.perf_counter()
        self.move_timer = self.create_timer(0.1, self.auto_move_and_publish)  # 10 Hz for movement
        self.vel_timer=self.create_timer(0.001, self.publish_velocity)  # 1000 Hz for velocity
        
        # Sequence‐done flag
        self.sequence_complete = False
        self.get_logger().info("Motor Publisher Node Initialized")

    def publish_present_position(self, present_position, goal_position):
        """
        Publish the current and goal positions of the motor.
        
        Args:
            present_position (int): The motor's current position.
            goal_position (int): The motor's current goal position.
        """
        self.position_pub.publish(Int32(data=int(present_position)))
        self.goal_pub.publish(Int32(data=int(goal_position)))

    def publish_velocity(self):
        """
        Publish the motor's current velocity.
        """
        raw_vel = self.controller.get_present_velocity()
        rpm = utils.raw_to_rpm(raw_vel)
        lin_vel = utils.rpm_to_linear_velocity_mps(rpm)
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist.linear.x = lin_vel
        self.vel_pub.publish(msg)

    def auto_move_and_publish(self):
        """
        Main callback to control motor movement and publish its state based on the sequence.
        
        - Processes actions: 'move' to a position, 'set_speed' to change velocity/acceleration,
          and 'pause' for a specified duration.
        - Continuously publishes position and velocity, even during pauses.
        - Shuts down after all actions are completed.
        """
        if self.pausing:
            # Handle pause
            current_time = self.get_clock().now()
            elapsed = (current_time - self.pause_start_time).nanoseconds / 1e9  # Convert to seconds
            if elapsed >= self.pause_duration:
                self.pausing = False
                self.current_index += 1
        else:
            # Check if all actions have been completed
            if self.current_index >= len(self.sequence):
                self.get_logger().info("All goal positions reached. Shutting down...")
                end_time = time.perf_counter()
                self.get_logger().info(f"Time taken to reach goal position: {end_time - self.start_time:.2f} seconds")
                self.move_timer.cancel()
                self.vel_timer.cancel()
                self.sequence_complete = True
                return

            # Process the current action
            action = self.sequence[self.current_index]
            if action['type'] == 'move':
                if not self.moving:
                    self.controller.set_goal_position(action['position'])
                    self.moving = True
                present_position = self.controller.get_present_position()
                if abs(present_position - action['position']) <= self.controller.MOVING_STATUS_THRESHOLD:
                    self.get_logger().info(f"Reached Goal Position: {action['position']}")
                    self.moving = False
                    self.current_index += 1
            elif action['type'] == 'set_speed':
                self.controller.set_vel_and_accel(action['speed'], action['accel'])
                self.get_logger().info(f"Set velocity and acceleration to: {action['speed']} and {action['accel']}")
                self.current_index += 1
            elif action['type'] == 'pause':
                self.pausing = True
                self.pause_start_time = self.get_clock().now()
                self.pause_duration = action['duration']

        # Publish current state
        present_position = self.controller.get_present_position()
        # Find the next 'move' position as the goal position
        next_move_index = next((i for i in range(self.current_index, len(self.sequence)) 
                               if self.sequence[i]['type'] == 'move'), len(self.sequence))
        if next_move_index < len(self.sequence):
            goal_position = self.sequence[next_move_index]['position']
        else:
            goal_position = 0  # Default when no more moves
        self.publish_present_position(present_position, goal_position)

def main(args=None):
    """Main function to initialize and run the ROS2 node."""
    rclpy.init(args=args)

    # Define the sequence of actions (replicates original behavior)
    sequence = [
        {'type': 'set_speed', 'speed': 100, 'accel': 10},
        {'type': 'move', 'position': 1500},
        {'type': 'set_speed', 'speed': 150, 'accel': 35},
        {'type': 'pause', 'duration': 1.0},
        {'type': 'move', 'position': 2000},
        {'type': 'set_speed', 'speed': 30, 'accel': 10},
        {'type': 'pause', 'duration': 2.0},
        {'type': 'move', 'position': 200},
        {'type': 'move', 'position': 2000},
        {'type': 'set_speed', 'speed': 75, 'accel': 0},
        {'type': 'move', 'position': 1500},
        {'type': 'set_speed', 'speed': 25, 'accel': 50},
        {'type': 'move', 'position': 200},
        {'type': 'set_speed', 'speed': 100, 'accel': 0},
        {'type': 'move', 'position': 1700},
        {'type': 'pause', 'duration': 3.0},
        {'type': 'set_speed', 'speed': 25, 'accel': 10},
        {'type': 'move', 'position': 500},
        {'type': 'set_speed', 'speed': 100, 'accel': 0},
        {'type': 'move', 'position': 1700},
        {'type': 'move', 'position': 1300},
        {'type': 'move', 'position': 1700},
        {'type': 'move', 'position': 1300},
        {'type': 'move', 'position': 1700},
        {'type': 'move', 'position': 1300},
        {'type': 'move', 'position': 1700},
        {'type': 'move', 'position': 1500},
        {'type': 'set_speed', 'speed': 75, 'accel': 50},
        {'type': 'move', 'position': 2000},
        {'type': 'move', 'position': 1700},
        {'type': 'move', 'position': 2000},
        {'type': 'set_speed', 'speed': 350, 'accel': 100},
        {'type': 'pause', 'duration': 5.0},
        {'type': 'move', 'position': 2400},
        {'type': 'pause', 'duration': 1.0},
        {'type': 'set_speed', 'speed': 200, 'accel': 10},
        {'type': 'move', 'position': 1000},
        {'type': 'set_speed', 'speed': 200, 'accel': 5},
        {'type': 'move', 'position': 2300},
        {'type': 'pause', 'duration': 4.0},
        {'type': 'set_speed', 'speed': 250, 'accel': 50},
        {'type': 'move', 'position': 100},
        {'type': 'set_speed', 'speed': 250, 'accel': 10},
        {'type': 'move', 'position': 2300},
        {'type': 'set_speed', 'speed': 75, 'accel': 50},
        {'type': 'move', 'position': 1800},
        {'type': 'set_speed', 'speed': 150, 'accel': 100},
        {'type': 'move', 'position': 1000},
        {'type': 'pause', 'duration': 3.0},
        {'type': 'set_speed', 'speed': 25, 'accel': 5},
        {'type': 'move', 'position': 0},
        {'type': 'pause', 'duration': 1.0},
    ]
    
    sequence2RepeaterSlow = [
    {'type': 'set_speed', 'speed': 50, 'accel': 10},
    {'type': 'move', 'position': 2300},
    {'type': 'move', 'position': 0},
    {'type': 'move', 'position': 2300},
    {'type': 'move', 'position': 0},
    {'type': 'move', 'position': 2300},
    {'type': 'move', 'position': 0},
    {'type': 'move', 'position': 2300},
    {'type': 'move', 'position': 0},
    {'type': 'move', 'position': 2300},
    {'type': 'move', 'position': 0},
    {'type': 'move', 'position': 2300},
    {'type': 'move', 'position': 0},
    {'type': 'move', 'position': 2300},
    {'type': 'move', 'position': 0},
    {'type': 'move', 'position': 2300},
    {'type': 'move', 'position': 0},
    {'type': 'move', 'position': 2300},
    {'type': 'move', 'position': 0},
    # {'type': 'pause', 'duration': 1.0},

    ]
    
    sequenceRepeaterFast = [
    {'type': 'set_speed', 'speed': 250, 'accel': 75},
    {'type': 'move', 'position': 2300},
    {'type': 'move', 'position': 0},
    {'type': 'move', 'position': 2300},
    {'type': 'move', 'position': 0},
    {'type': 'move', 'position': 2300},
    {'type': 'move', 'position': 0},
    {'type': 'move', 'position': 2300},
    {'type': 'move', 'position': 0},
    {'type': 'move', 'position': 2300},
    {'type': 'move', 'position': 0},
    {'type': 'move', 'position': 2300},
    {'type': 'move', 'position': 0},
    {'type': 'move', 'position': 2300},
    {'type': 'move', 'position': 0},
    {'type': 'move', 'position': 2300},
    {'type': 'move', 'position': 0},
    {'type': 'move', 'position': 2300},
    {'type': 'move', 'position': 0},
    # {'type': 'pause', 'duration': 1.0},

    ]
    
    sequenceStationary = [
    {'type': 'set_speed', 'speed': 40, 'accel': 25},
    {'type': 'move', 'position': 1500},
    {'type': 'pause', 'duration': 58.0},
    {'type': 'move', 'position': 0},
    # {'type': 'pause', 'duration': 1.0},

    ]

    sequencePD = [
    {'type': 'set_speed', 'speed': 40, 'accel': 25},
    {'type': 'move', 'position': 1200},
    {'type': 'pause', 'duration': 1.0},
    # {'type': 'pause', 'duration': 1.0},
    ]
    
    sequenceHome = [
    {'type': 'set_speed', 'speed': 40, 'accel': 25},
    {'type': 'move', 'position': 0},
    {'type': 'pause', 'duration': 1.0},
    # {'type': 'pause', 'duration': 1.0},

    ]
    
    # Initialize the motor controller
    controller = DynamixelMXController()  # Removed goal_positions as it's now in sequence
    controller.open_port()
    controller.set_position_limits()
    controller.set_vel_and_accel()  # Set default velocity and acceleration
    controller.enable_torque()

    # Create and spin the node
    node = MotorPublisherNode(controller, sequencePD)
    
    while rclpy.ok() and not node.sequence_complete:
        rclpy.spin_once(node, timeout_sec=0.5)
    # Clean up exactly once
    node.get_logger().info("Shutting down motor node...")
    controller.disable_torque()
    controller.close_port()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()