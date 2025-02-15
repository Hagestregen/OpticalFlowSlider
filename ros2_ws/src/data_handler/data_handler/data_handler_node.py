import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import sys

class DataHandlerNode(Node):
    def __init__(self):
        super().__init__('data_handler_node')
        self.motor_pos_sub = self.create_subscription(
            Int32, 
            '/motor/present_position', 
            self.motor_callback, 
            10)
        
        self.goal_pos_sub = self.create_subscription(
            Int32, 
            '/motor/goal_position', 
            self.goal_callback, 
            10)
        
        self.prev_motor_pos = None
        self.prev_time = None
        self.velocity_estimate = None
        
    
    def motor_callback(self, msg):
        """ Process motor position updates """
        current_time = self.get_clock().now()
        if self.prev_motor_pos is not None and self.prev_time is not None:
            dt = (current_time - self.prev_time).nanoseconds / 1e9  # Convert to seconds
            velocity = (msg.data - self.prev_motor_pos) / dt
            self.get_logger().info(f"Motor velocity: {velocity:.4f} units/s")
            # self.publish_state_estimate(velocity)

        self.prev_motor_pos = msg.data
        self.prev_time = current_time

        

    def goal_callback(self, msg):
        self.get_logger().info(f"🔴 Received goal position: {msg.data}")


    



    def cleanup_and_exit(self):
        print("🔻 Destroying ROS2 node and shutting down...")
        self.destroy_node()
        rclpy.shutdown()

        print("Exiting program now...")
        sys.exit(0)  

if __name__ == '__main__':
    rclpy.init()
    

    try:
        node = DataHandlerNode()
        rclpy.spin(node) 

    except KeyboardInterrupt:
        print("\n🛑 [CTRL+C] Interrupt received. Cleaning up...")
        node.cleanup_and_exit()