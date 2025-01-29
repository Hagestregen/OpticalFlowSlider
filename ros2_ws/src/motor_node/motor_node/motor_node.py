import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from motor_control import DynamixelMXController
import sys

class MotorPublisherNode(Node):
    """ROS2 Node for controlling and monitoring a Dynamixel motor."""
    
    def __init__(self, controller):
        super().__init__('motor_publisher')
        self.controller = controller
        self.position_pub = self.create_publisher(Int32, 'motor/present_position', 10)
        self.goal_pub = self.create_publisher(Int32, 'motor/goal_position', 10)
        self.current_index = 0
        self.cleanup_called = False  # Prevent multiple cleanup calls
        self.timer = self.create_timer(0.1, self.auto_move_and_publish)  # Timer for automatic movement

    def publish_present_position(self, present_position, goal_position):
        """Publishes the current and goal positions."""
        self.position_pub.publish(Int32(data=present_position))
        self.goal_pub.publish(Int32(data=goal_position))

    def auto_move_and_publish(self):
        """Moves to the next goal position automatically and publishes position updates."""
        if self.current_index >= len(self.controller.goal_positions):  
            print("✅ All goal positions reached. Shutting down...")
            self.destroy_timer(self.timer)
            self.cleanup_and_exit()
            return

        goal_position = self.controller.goal_positions[self.current_index]
        print(f"🔹 Moving to goal position: {goal_position}")
        self.controller.set_goal_position(goal_position)

        present_position = self.controller.get_present_position()
        self.publish_present_position(present_position, goal_position)
        print(f"🟢 [ID:{self.controller.motor_id}] GoalPos: {goal_position}, PresPos: {present_position}")

        if abs(present_position - goal_position) <= self.controller.MOVING_STATUS_THRESHOLD:
            print(f"✅ Reached Goal Position: {goal_position}")
            self.current_index += 1  # Move to the next position

    def cleanup_and_exit(self):
        """Handles shutdown: disables torque, closes the port, stops ROS2."""
        if self.cleanup_called:
            return  
        self.cleanup_called = True  

        print("🔻 Disabling torque and closing the port...")
        self.controller.disable_torque()
        
        try:
            self.controller.close_port()
        except Exception as e:
            print(f"⚠️ [WARNING] Failed to close port: {e}")

        print("🔻 Destroying ROS2 node and shutting down...")
        self.destroy_node()
        rclpy.shutdown()

        print("🔴 Exiting program now...")
        sys.exit(0)  


def main(args=None):
    """Main entry point for the ROS2 motor control node."""
    rclpy.init(args=args)

    # Define default goal positions (modify as needed)
    goal_positions = [2000, 1000, 2500, 1500, 3000]

    # Validate positions
    valid_positions = [pos for pos in goal_positions if 0 <= pos <= 3000]
    if not valid_positions:
        print("❌ No valid goal positions provided. Exiting...")
        return

    # Initialize motor controller
    controller = DynamixelMXController(goal_positions=valid_positions)
    controller.open_port()
    controller.set_position_limits()
    controller.enable_torque()

    try:
        motor_node = MotorPublisherNode(controller)
        rclpy.spin(motor_node)  

    except KeyboardInterrupt:
        print("\n🛑 [CTRL+C] Interrupt received. Cleaning up...")
        motor_node.cleanup_and_exit()

    except Exception as e:
        print(f"❌ [ERROR] {str(e)}")

    finally:
        if not motor_node.cleanup_called:
            print("🔻 Final cleanup process...")
            motor_node.cleanup_and_exit()


if __name__ == '__main__':
    main()
