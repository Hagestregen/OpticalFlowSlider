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
        self.declare_parameter('sigma_a', 0.1)
        self.declare_parameter('sigma_flow', 0.05)
        self.declare_parameter('sigma_b', 0.001)
        
        # Parameters for IMU filtering
        self.declare_parameter('imu_cutoff_freq', 0.1)
        self.declare_parameter('imu_sampling_freq', 100)
        self.declare_parameter('imu_filter_order', 2)
        
        # Get parameter values
        dt = self.get_parameter('dt').value
        sigma_a = self.get_parameter('sigma_a').value
        sigma_flow = self.get_parameter('sigma_flow').value
        sigma_b = self.get_parameter('sigma_b').value
        self.imu_cutoff = self.get_parameter('imu_cutoff_freq').value
        self.imu_fs = self.get_parameter('imu_sampling_freq').value
        self.imu_order = self.get_parameter('imu_filter_order').value
        
        # QoS profile for optical flow messages
        qos_profile = QoSProfile(
            depth=50,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST
        )
        
        # Initialize the Kalman Filter with parameters
        self.kf = KalmanFilter(dt=dt, sigma_a=sigma_a, sigma_flow=sigma_flow, sigma_b=sigma_b)
        
        # State and timing variables
        self.state_history = []
        self.last_imu_time = None
        
        # Filter state for incremental filtering of acceleration
        self.filter_state = np.zeros(max(3, self.imu_order) - 1)  # Adjust based on filter order
        
        # Subscriptions
        self.imu_sub = self.create_subscription(
            Imu,
            '/inertialsense/imu',
            self.imu_callback,
            10
        )
        
        self.flow_sub = self.create_subscription(
            Vector3Stamped,
            '/optical_flow/PWC_velocity',
            self.flow_callback,
            qos_profile
        )
        
        self.flow_sub = self.create_subscription(
            Vector3Stamped,
            '/optical_flow/PWC_smooth_velocity',
            self.flow_callback,
            qos_profile
        )
        
        # Publishers
        self.state_pub = self.create_publisher(Vector3Stamped, '/kalman_filter/state', 10)
        # self.state_vel_pub = self.create_publisher(Vector3Stamped, '/kalman_filter/velocity', 10)
        # self.position_pub = self.create_publisher(Float64, '/kalman_filter/position', 10)
        self.imu_filt_pub = self.create_publisher(Float64, '/kalman_filter/imu_filtered', 10)
        
        # Timer
        self.timer = self.create_timer(0.01, self.timer_callback)
        
        self.get_logger().info('Kalman Filter Node (with OOSM) is up and running!')

    def low_pass_filter(self, new_value):
        """Apply low-pass filter incrementally to a new acceleration value."""
        nyquist = self.imu_fs / 2
        normal_cutoff = self.imu_cutoff / nyquist
        b, a = butter(self.imu_order, normal_cutoff, btype='low', analog=False)
        output, self.filter_state = lfilter(b, a, [new_value], zi=self.filter_state)
        return output[0]

    def imu_callback(self, msg):
        """Process IMU data, filter acceleration, and update Kalman filter."""
        acceleration = msg.linear_acceleration.x
        imu_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        
        dt = self.kf.dt_default if self.last_imu_time is None else max(imu_time - self.last_imu_time, 1e-3)
        if self.last_imu_time is None:
            self.last_imu_time = imu_time
            return
        
        # Apply low-pass filter to acceleration
        filtered_acceleration = self.low_pass_filter(acceleration)
        
        # bias = float(self.kf.x_hat[2])
        # corrected_acceleration = filtered_acceleration - bias
        
        self.imu_filt_pub.publish(Float64(data=filtered_acceleration))
        # Update Kalman filter with filtered acceleration
        self.kf.predict(filtered_acceleration, dt=dt)  # Kalman filter expects acceleration
        # self.kf.predict(acceleration, dt=dt)
        # Update the latest time
        self.last_imu_time = imu_time
        
        # Buffer the current state (store copies to avoid mutation)
        self.state_history.append({
            'time': imu_time,
            'x_hat': self.kf.x_hat.copy(),
            'P': self.kf.P.copy(),
            'accel': filtered_acceleration,
            # 'accel': acceleration  # Store raw acceleration for OOSM replay
        })
        
        # Prune old history (e.g., older than 1 second) to avoid unbounded growth
        current_time = imu_time
        self.state_history = [entry for entry in self.state_history if current_time - entry['time'] < 1.0]
        
        # self.get_logger().info(f"IMU time: {imu_time:.6f}, dt: {dt:.6f}, Raw accel: {acceleration:.3f}, Filtered accel: {filtered_acceleration:.3f} m/s²")

    def flow_callback(self, msg):
        """
        When an optical flow measurement arrives, incorporate it as an out-of-sequence measurement.
        
        Steps:
          1. Extract the measurement timestamp from the optical flow message.
          2. Find the buffered state that was valid at or just before that time.
          3. Reset the filter to that past state.
          4. Apply the optical flow measurement update.
          5. Replay (predict) all IMU measurements from that past state to the current time.
        """
        velocity = msg.vector.x
        flow_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        # Ensure we have history
        if not self.state_history:
            self.get_logger().warn("No buffered IMU states available. Skipping OOSM update.")
            return

        # Find the most recent state before (or equal to) the flow measurement time
        oosm_index = None
        for i, entry in enumerate(self.state_history):
            if entry['time'] > flow_time:
                break
            oosm_index = i

        if oosm_index is None:
            self.get_logger().warn("No buffered state is older than the optical flow measurement time.")
            return

        # Backup the current (latest) filter state
        backup_state = {
            'x_hat': self.kf.x_hat.copy(),
            'P': self.kf.P.copy()
        }
        
        # Roll back the filter to the state at the measurement time
        state_entry = self.state_history[oosm_index]
        self.kf.x_hat = state_entry['x_hat'].copy()
        self.kf.P = state_entry['P'].copy()
        rollback_time = state_entry['time']

        # Apply the measurement update at the time of the optical flow
        self.kf.update(velocity)

        # Replay all IMU predictions from rollback_time to current time
        replay_entries = [entry for entry in self.state_history if entry['time'] > rollback_time]
        replay_entries.sort(key=lambda e: e['time'])
        previous_time = rollback_time
        for entry in replay_entries:
            dt = entry['time'] - previous_time
            # Use the stored raw acceleration for replay (not filtered, to maintain consistency)
            self.kf.predict(entry['accel'], dt=dt)
            previous_time = entry['time']

        # Prune and publish state
        self.prune_state_history()
        self.publish_state()

    def timer_callback(self):
        self.publish_state()
        
    def prune_state_history(self):
        """Remove old entries from the state history that are older than a given threshold."""
        if self.last_imu_time is None:
            return

        threshold = 2.0  # seconds; reduced for efficiency
        self.state_history = [
            entry for entry in self.state_history
            if self.last_imu_time - entry['time'] < threshold
        ]

    def publish_state(self):
        state = self.kf.get_state()
        timestamp = self.get_clock().now().to_msg()
        
        # State (position, velocity, bias)
        state_msg = Vector3Stamped()
        state_msg.header.stamp = timestamp
        state_msg.header.frame_id = 'world'  # Adjust frame_id as needed
        state_msg.vector.x = float(state[0])  # Position
        state_msg.vector.y = float(state[1])  # Velocity
        state_msg.vector.z = float(state[2])  # Bias
        self.state_pub.publish(state_msg)
        
        # state_msg = Float32MultiArray()
        # state_msg.header.stamp = timestamp
        # state_msg.header.frame_id = 'world'  # Adjust frame_id as needed
        # state_msg.data = state.tolist() 
        # self.state_pub.publish(state_msg)
        
        
        # velocity_msg = Float64()
        # velocity_msg.data = state[1]  # Velocity
        # self.state_vel_pub.publish(velocity_msg)
        
        # Velocity
        # velocity_msg = Vector3Stamped()
        # velocity_msg.header.stamp = timestamp
        # velocity_msg.header.frame_id = 'world'
        # velocity_msg.vector.x = float(state[1])  # Velocity
        # velocity_msg.vector.y = 0.0
        # velocity_msg.vector.z = 0.0
        # self.state_vel_pub.publish(velocity_msg)
        
        # position_msg = Float64()
        # position_msg.data = state[0]  # Position
        # self.position_pub.publish(position_msg)
        # self.get_logger().info(f"Published position: {position_msg.data}, and velocity: {velocity_msg.data}")

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