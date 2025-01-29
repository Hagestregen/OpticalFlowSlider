import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from motor_control import DynamixelMXController

class MotorPublisherNode(Node):
    def __init__(self, controller):
        super().__init__('motor_publisher')
        self.controller = controller
        self.position_pub = self.create_publisher(Int32, 'motor/present_position', 10)
        self.goal_pub = self.create_publisher(Int32, 'motor/goal_position', 10)

    def publish_position(self, present_position, goal_position):
        self.position_pub.publish(Int32(data=present_position))
        self.goal_pub.publish(Int32(data=goal_position))

    def move_and_publish(self):
        goal_position = self.controller.goal_positions[self.controller.current_goal_index]
        self.controller.set_goal_position(goal_position)

        while self.controller.is_moving():
            present_position = self.controller.get_present_position()
            print(f"[ID:{self.controller.motor_id}] GoalPos:{goal_position}  PresPos:{present_position}")
            self.publish_position(present_position, goal_position)

        self.controller.current_goal_index = 1 - self.controller.current_goal_index  # Toggle

def main(args=None):
    rclpy.init(args=args)

    goal_positions = [1500, 3000]  # Default values
    print("Enter goal positions as comma-separated values (e.g., 1500,3000):")
    user_input = input()
    if user_input.strip():
        goal_positions = [int(pos) for pos in user_input.split(',') if pos.isdigit()]

    controller = DynamixelMXController(goal_positions=goal_positions)
    controller.open_port()
    controller.set_position_limits()
    controller.enable_torque()

    try:
        motor_node = MotorPublisherNode(controller)
        while rclpy.ok():
            print("Press any key to move to the next goal! (ESC to quit)")
            if controller.getch() == chr(0x1b):  # ESC key
                controller.disable_torque()
                break
            motor_node.move_and_publish()

    finally:
        controller.disable_torque()
        controller.close_port()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
