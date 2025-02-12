import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import time
import random

class MockMotorNode(Node):
    def __init__(self):
        super().__init__('mock_motor_node')
        self.position_pub = self.create_publisher(Int32, 'motor/present_position', 10)
        self.goal_pub = self.create_publisher(Int32, 'motor/goal_position', 10)

        self.goal_positions = [2000, 1000, 2500, 1500, 3000]  # Mock goal positions
        self.current_index = 0
        self.current_position = self.goal_positions[0]  # Start at the first goal

        self.timer = self.create_timer(0.1, self.mock_motor_movement)

    def mock_motor_movement(self):
        """Simulates motor movement and publishes present position and goal position."""
        if self.current_index >= len(self.goal_positions):  
            self.get_logger().info("✅ Finished all goal positions. Exiting mock node...")
            rclpy.shutdown()
            return

        goal_position = self.goal_positions[self.current_index]

        # Simulate gradual movement towards the goal position
        step_size = random.randint(30, 70)  # Simulate movement speed variability
        if self.current_position < goal_position:
            self.current_position = min(self.current_position + step_size, goal_position)
        elif self.current_position > goal_position:
            self.current_position = max(self.current_position - step_size, goal_position)

        # Publish current position and goal position
        self.position_pub.publish(Int32(data=self.current_position))
        self.goal_pub.publish(Int32(data=goal_position))

        self.get_logger().info(f"🟢 [MOCK] GoalPos: {goal_position}, PresPos: {self.current_position}")

        # Check if the position is close enough to the goal to consider it reached
        if abs(self.current_position - goal_position) <= 10:
            self.get_logger().info(f"✅ [MOCK] Reached Goal Position: {goal_position}")
            self.current_index += 1  # Move to the next goal

def main(args=None):
    rclpy.init(args=args)
    mock_motor_node = MockMotorNode()
    rclpy.spin(mock_motor_node)

if __name__ == '__main__':
    main()
