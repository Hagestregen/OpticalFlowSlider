#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Float64MultiArray
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
        self.declare_parameter('dt',                0.01)
        self.declare_parameter('sigma_a',           0.1)
        self.declare_parameter('sigma_flow',        0.05)
        self.declare_parameter('sigma_b',           0.001)
        
        # Parameters for IMU filtering
        self.declare_parameter('imu_cutoff_freq',   0.1)
        self.declare_parameter('imu_sampling_freq', 100)
        self.declare_parameter('imu_filter_order',  2)
        
        # OOSM
        self.declare_parameter('enable_oosm',       False)
        
        # Get parameter values
        dt                  = self.get_parameter('dt').value
        sigma_a             = self.get_parameter('sigma_a').value
        sigma_flow          = self.get_parameter('sigma_flow').value
        sigma_b             = self.get_parameter('sigma_b').value
        self.imu_cutoff     = self.get_parameter('imu_cutoff_freq').value
        self.imu_fs         = self.get_parameter('imu_sampling_freq').value
        self.imu_order      = self.get_parameter('imu_filter_order').value
        self.enable_oosm    = self.get_parameter('enable_oosm').value
        
        self.calib_duration_s   = 1.0
        self.calib_samples      = []
        self.calib_target_count = int(self.calib_duration_s / dt)
        self.initial_bias       = 0.0
        self.calibrated         = False
        
        # noise_density_g   = 60e-6       # 60 μg/√Hz
        # bias_stability_g  = 19e-6       # 19 μg
        # g = 9.81                         # m/s² per g
        # sigma_a_new = noise_density_g * g * (1.0 / dt)**0.5
        # sigma_b_new = bias_stability_g * g
        
        self.velocity_x = 0.0 
        
        # QoS profile for optical flow messages
        qos_profile = QoSProfile(
            depth=50,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST
        )
        
        # Initialize the Kalman Filter with parameters
        self.kf = KalmanFilter(
            dt=dt, 
            sigma_a=sigma_a, 
            sigma_flow=sigma_flow, 
            sigma_b=sigma_b)
        
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
            '/optical_flow/LFN3_velocity',
            self.flow_callback,
            qos_profile
        )
        
        # self.flow_sub = self.create_subscription(
        #     Vector3Stamped,
        #     '/optical_flow/LFN3_smooth_velocity',
        #     self.flow_callback,
        #     qos_profile
        # )
        
        # Publishers
        self.p_publish = self.create_publisher(Float64MultiArray, 'kalman/p_matrix', 10)
        self.state_pub = self.create_publisher(Vector3Stamped, '/kalman_filter/state', 10)
        # self.state_vel_pub = self.create_publisher(Vector3Stamped, '/kalman_filter/velocity', 10)
        # self.position_pub = self.create_publisher(Float64, '/kalman_filter/position', 10)
        self.imu_filt_pub = self.create_publisher(Float64, '/kalman_filter/imu_filtered', 10)
        
        # Timer
        self.timer = self.create_timer(dt, self.timer_callback)
        
        
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
        
        # Calibrate IMU Bias
        if not self.calibrated:
            self.calib_samples.append(acceleration)
            self.last_imu_time = imu_time
            if len(self.calib_samples) >= self.calib_target_count:
                self.initial_bias = float(np.mean(self.calib_samples))
                self.kf.x_hat[2, 0] = self.initial_bias
                self.calibrated = True
                self.get_logger().info(
                    f"IMU bias calibrated: {self.initial_bias:.4f} m/s²"
                )
                filt0 = self.low_pass_filter(acceleration)
                corr0 = filt0 - self.initial_bias
                self.state_history.append({
                    'time':  imu_time,
                    'x_hat': self.kf.x_hat.copy(),
                    'P':     self.kf.P.copy(),
                    'accel': corr0,
                })
            return
        
        dt = self.kf.dt_default if self.last_imu_time is None else max(imu_time - self.last_imu_time, 1e-3)
        
        self.last_imu_time = imu_time
        
        # Apply low-pass filter to acceleration
        filtered_acceleration = self.low_pass_filter(acceleration)
        
        # subtract fixed startup bias
        corr_accel = filtered_acceleration - self.initial_bias
        
        # bias = float(self.kf.x_hat[2])
        # corrected_acceleration = filtered_acceleration - bias
        
        self.velocity_x += corr_accel * dt
        
        self.imu_filt_pub.publish(Float64(data=self.velocity_x))
        
        # Update Kalman filter with filtered acceleration
        self.kf.predict(corr_accel, dt=dt)  # Kalman filter expects acceleration

        
        # Buffer the current state (store copies to avoid mutation)
        if self.enable_oosm:
            current_time = imu_time
            self.state_history.append({
                'time': imu_time,
                'x_hat': self.kf.x_hat.copy(),
                'P': self.kf.P.copy(),
                'accel': corr_accel,
                
            })
            self.state_history = [
                entry for entry in self.state_history if current_time - entry['time'] < 2.0
                ]


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
        
        if not self.enable_oosm:
            self.kf.update(velocity)
            self.publish_state()
            return

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
            self.kf.update(velocity)
            self.publish_state()
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
        
        
        p_msg = Float64MultiArray()
        p_msg.data = self.kf.P.flatten().tolist()
        self.p_publish.publish(p_msg)
        
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