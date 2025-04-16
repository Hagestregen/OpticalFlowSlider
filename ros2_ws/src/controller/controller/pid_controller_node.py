import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Float32MultiArray, Float64MultiArray
import numpy as np

class PIDControllerNode(Node):
    def __init__(self):
        super().__init__('pid_controller_node')
        # # PID gains
        # self.Kp = 0.8  # Proportional gain
        # self.Ki = 0.0  # Integral gain (small to start)
        # self.Kd = 0.2   # Derivative gain (for damping)
        # self.setpoint = 0.0
        # self.filtered_position = 0.0
        # self.current_velocity = 0.0
        # self.alpha = 0.3  # Adjusted for faster response
        # self.dt = 0.01
        # self.integral = 0.0
        # self.prev_error = 0.0
        # self.integral_limit = 10.0  # Prevent windup
        # self.deadband = 0.01  # Ignore small errors
        # Declare parameters with default values
        self.declare_parameter('Kp', 0.8)           # Proportional gain
        self.declare_parameter('Ki', 0.0)           # Integral gain
        self.declare_parameter('Kd', 0.2)           # Derivative gain
        self.declare_parameter('setpoint', 0.0)     # Desired position
        self.declare_parameter('alpha', 0.3)        # Smoothing factor for position filter
        self.declare_parameter('dt', 0.01)          # Time step for control loop
        self.declare_parameter('integral_limit', 10.0)  # Limit for integral windup prevention
        self.declare_parameter('deadband', 0.001)    # Error threshold below which no correction is made
        
        # Retrieve parameter values
        self.Kp = self.get_parameter('Kp').get_parameter_value().double_value
        self.Ki = self.get_parameter('Ki').get_parameter_value().double_value
        self.Kd = self.get_parameter('Kd').get_parameter_value().double_value
        self.setpoint = self.get_parameter('setpoint').get_parameter_value().double_value
        self.alpha = self.get_parameter('alpha').get_parameter_value().double_value
        self.dt = self.get_parameter('dt').get_parameter_value().double_value
        self.integral_limit = self.get_parameter('integral_limit').get_parameter_value().double_value
        self.deadband = self.get_parameter('deadband').get_parameter_value().double_value
        
        # Log parameter values for verification
        self.get_logger().info(
            f'PID parameters: Kp={self.Kp}, Ki={self.Ki}, Kd={self.Kd}, '
            f'setpoint={self.setpoint}, alpha={self.alpha}, dt={self.dt}, '
            f'integral_limit={self.integral_limit}, deadband={self.deadband}'
        )
        
        # Initialize other variables
        self.filtered_position = 0.0
        self.current_velocity = 0.0
        self.integral = 0.0
        self.prev_error = 0.0

        # Subscriber to Kalman filter state
        self.sub = self.create_subscription(
            Float32MultiArray,
            '/kalman_filter/state',
            self.state_callback,
            10
        )
        # Publisher for motor control input
        self.pub = self.create_publisher(
            Float64,
            '/motor/control_input',
            10
        )
        
        self.pos_pub = self.create_publisher(
            Float64,
            '/slider/current_position',
            10
        )
        
        self.pid_pub = self.create_publisher(
            Float64MultiArray,
            '/pid/output',
            10
        )
        self.create_timer(self.dt, self.control_loop)

    def state_callback(self, msg):
        self.current_position = msg.data[0]
        self.current_velocity = msg.data[1]
        self.filtered_position = (self.alpha * self.current_position +
                                  (1 - self.alpha) * self.filtered_position)
        new_filtered_position = Float64()
        new_filtered_position.data = self.filtered_position
        self.pos_pub.publish(new_filtered_position)
        # self.get_logger().info(f'Filtered position: {self.filtered_position:.2f} m, Velocity: {self.current_velocity:.2f} m/s')

    def control_loop(self):
        error = self.setpoint - self.filtered_position
        if abs(error) < self.deadband:
            error = 0.0
        # Integral term with windup protection
        self.integral += error * self.dt
        self.integral = max(min(self.integral, self.integral_limit), -self.integral_limit)
        # Derivative term
        # derivative = (error - self.prev_error) / self.dt
        derivative = -self.current_velocity
        # Control signal (positive error → positive velocity)
        u = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        self.prev_error = error
        u = max(min(u, 1.0), -1.0)  # Limit to [-1.0, 1.0] m/s
        
        p_term = self.Kp * error
        i_term = self.Ki * self.integral
        d_term = self.Kd * derivative
        
        arr = np.array([p_term, i_term, d_term]).astype(np.float64)
        
        
        pid_output = Float64MultiArray()
        pid_output.data = arr.tolist()
        # pid_output.data = [p_term, i_term, d_term]
        self.pid_pub.publish(pid_output)
        
        msg = Float64()
        msg.data = u
        self.pub.publish(msg)
        # self.get_logger().info(f'Control signal: {u:.2f} m/s, Error: {error:.2f} m')

def main(args=None):
    rclpy.init(args=args)
    node = PIDControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()