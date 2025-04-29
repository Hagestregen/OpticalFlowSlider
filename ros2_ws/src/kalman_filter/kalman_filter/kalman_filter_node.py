#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Float32MultiArray
from geometry_msgs.msg import Vector3Stamped
from sensor_msgs.msg import Imu
from kalman_filter import KalmanFilter
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import numpy as np
from scipy.signal import lfilter, butter

class KalmanFilterNode(Node):
    def __init__(self):
        super().__init__('kalman_filter_node')
        
        # Declare parameters for Kalman filter initialization
        self.declare_parameter('dt', 0.01)
        self.declare_parameter('sigma_a', 0.3)
        self.declare_parameter('sigma_flow', 0.01)
        self.declare_parameter('sigma_b', 0.005)
        
        # Parameters for IMU filtering
        self.declare_parameter('imu_cutoff_freq', 1)
        self.declare_parameter('imu_sampling_freq', 71)
        self.declare_parameter('imu_filter_order', 2)
        
        # Get parameter values
        dt = self.get_parameter('dt').value
        sigma_a = self.get_parameter('sigma_a').value
        sigma_flow = self.get_parameter('sigma_flow').value
        sigma_b = self.get_parameter('sigma_b').value
        self.imu_cutoff = self.get_parameter('imu_cutoff_freq').value
        self.imu_fs = self.get_parameter('imu_sampling_freq').value
        self.imu_order = int(self.get_parameter('imu_filter_order').value)
        
        # QoS profile for optical flow messages
        qos_profile = QoSProfile(
            depth=50,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST
        )
        
        # Initialize the Kalman Filter with parameters
        self.kf = KalmanFilter(dt=dt, sigma_a=sigma_a, sigma_flow=sigma_flow, sigma_b=sigma_b)
        
        # Timing variables
        self.last_imu_time = None
        
        # Filter state for incremental filtering of acceleration
        self.filter_state = np.zeros(self.imu_order) # Adjust based on filter order
        
        # Subscriptions
        self.imu_sub = self.create_subscription(
            Imu,
            '/inertialsense/imu',
            self.imu_callback,
            10
        )
        
        self.flow_sub = self.create_subscription(
            Vector3Stamped,
            '/optical_flow/LFN3_velocity',
            self.flow_callback,
            qos_profile
        )
        
        # Publishers
        self.state_pub = self.create_publisher(Vector3Stamped, '/kalman_filter/new_state', 10)
        self.state_vel_pub = self.create_publisher(Vector3Stamped, '/kalman_filter/new_velocity', 10)
        self.position_pub = self.create_publisher(Vector3Stamped, '/kalman_filter/new_position', 10)
        self.imu_filt_pub = self.create_publisher(Vector3Stamped, '/kalman_filter/new_imu_filtered', 10)
        # self.state_pub = self.create_publisher(Float32MultiArray, '/kalman_filter/state', 10)
        # self.state_vel_pub = self.create_publisher(Float64, '/kalman_filter/velocity', 10)
        # self.position_pub = self.create_publisher(Float64, '/kalman_filter/position', 10)
        # self.imu_filt_pub = self.create_publisher(Float64, '/kalman_filter/imu_filtered', 10)
        
        # Timer
        self.timer = self.create_timer(0.01, self.timer_callback)
        
        self.get_logger().info('Kalman Filter Node is up and running!')

    def low_pass_filter(self, new_value):
        """Apply low-pass filter incrementally to a new acceleration value."""
        nyquist = self.imu_fs / 2
        normal_cutoff = self.imu_cutoff / nyquist
        b, a = butter(self.imu_order, normal_cutoff, btype='low', analog=False)
        output, self.filter_state = lfilter(b, a, [new_value], zi=self.filter_state)
        return output[0]

    def imu_callback(self, msg):
        """Process IMU data, filter acceleration, and predict Kalman filter state."""
        acceleration = msg.linear_acceleration.x
        imu_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        
        dt = self.kf.dt_default if self.last_imu_time is None else max(imu_time - self.last_imu_time, 1e-3)
        if self.last_imu_time is None:
            self.last_imu_time = imu_time
            return
        
        # Apply low-pass filter to acceleration
        filtered_acceleration = self.low_pass_filter(acceleration)
        # Subtract the estimated bias from the filtered acceleration
        bias = self.kf.x_hat[2]  # Bias is the third element in the state vector
        corrected_acceleration = filtered_acceleration - bias
        
        
        # Publish filtered acceleration with timestamp
        imu_msg = Vector3Stamped()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = 'imu_link'  # Adjust frame_id as needed
        imu_msg.vector.x = float(corrected_acceleration)
        imu_msg.vector.y = 0.0
        imu_msg.vector.z = 0.0
        self.imu_filt_pub.publish(imu_msg)
        
        
        # Predict Kalman filter state with filtered acceleration
        # self.kf.predict(corrected_acceleration, dt=dt)
        # self.kf.predict(corrected_acceleration, dt=self.kf.dt_default)
        self.kf.predict(acceleration, dt=self.kf.dt_default)
        
        # Update the latest time
        self.last_imu_time = imu_time
        
        # Publish the state after prediction
        self.publish_state()

    def flow_callback(self, msg):
        """Directly update the Kalman filter with aligned optical flow velocity."""
        velocity = msg.vector.x
        # Assuming the optical flow measurement is aligned with the current state
        self.kf.update(velocity)
        # Publish the state after update
        self.publish_state()

    def timer_callback(self):
        """Publish the current state at regular intervals."""
        self.publish_state()

    def publish_state(self):
        """Publish the current state, velocity, and position with timestamps."""
        state = self.kf.get_state()  # [position, velocity, bias]
        timestamp = self.get_clock().now().to_msg()
        
        # State (position, velocity, bias)
        state_msg = Vector3Stamped()
        state_msg.header.stamp = timestamp
        state_msg.header.frame_id = 'world'  # Adjust frame_id as needed
        state_msg.vector.x = float(state[0])  # Position
        state_msg.vector.y = float(state[1])  # Velocity
        state_msg.vector.z = float(state[2])  # Bias
        self.state_pub.publish(state_msg)
        
        # Velocity
        velocity_msg = Vector3Stamped()
        velocity_msg.header.stamp = timestamp
        velocity_msg.header.frame_id = 'world'
        velocity_msg.vector.x = float(state[1])  # Velocity
        velocity_msg.vector.y = 0.0
        velocity_msg.vector.z = 0.0
        self.state_vel_pub.publish(velocity_msg)
        
        # Position
        position_msg = Vector3Stamped()
        position_msg.header.stamp = timestamp
        position_msg.header.frame_id = 'world'
        position_msg.vector.x = float(state[0])  # Position
        position_msg.vector.y = 0.0
        position_msg.vector.z = 0.0
        self.position_pub.publish(position_msg)

def main(args=None):
    rclpy.init(args=args)
    node = KalmanFilterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()