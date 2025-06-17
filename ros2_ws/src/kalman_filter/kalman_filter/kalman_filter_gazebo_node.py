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
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

class KalmanFilterNode(Node):
    def __init__(self):
        super().__init__('kalman_filter_gazebo_node')
        
        # Create separate callback groups
        self.imu_cb_group   = ReentrantCallbackGroup()
        self.flow_cb_group  = ReentrantCallbackGroup()
        self.timer_cb_group = ReentrantCallbackGroup()
        self.gain_cb_group  = ReentrantCallbackGroup()
        
        # Declare parameters for Kalman filter initialization
        self.declare_parameter('dt',                0.014)
        self.declare_parameter('sigma_a',           0.5)
        self.declare_parameter('sigma_flow',        0.01)
        self.declare_parameter('sigma_b',           0.001)
        
        # Parameters for IMU filtering
        self.declare_parameter('imu_cutoff_freq',   2.5)
        self.declare_parameter('imu_sampling_freq', 71)
        self.declare_parameter('imu_filter_order',  2)
        
        # OOSM
        self.declare_parameter('enable_oosm',       True)
        
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
        self.calibrated         = True
        # self.initial_bias       = 0.0625
        
        self.velocity_x         = 0.0 
        self.velocity_unfiltered_x = 0.0
        self.dt_sum             = 0.0
        self.dt_count           = 0
        
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
        
        self.get_logger().info(
            f"Kalman Filter initialized with dt={dt}, sigma_a={sigma_a}, "
            f"sigma_flow={sigma_flow}, sigma_b={sigma_b}"
        )
        
        # State and timing variables
        self.state_history = []
        self.last_imu_time = None
        
        # Filter state for incremental filtering of acceleration
        self.filter_state = np.zeros(self.imu_order)
        
        # Subscriptions
        self.imu_sub = self.create_subscription(
            Imu,
            '/inertialsense/imu',
            self.imu_callback,
            10,
            callback_group=self.imu_cb_group
        )
        
        self.flow_sub = self.create_subscription(
            Vector3Stamped,
            '/optical_flow/LFN3_smooth_velocity',
            self.flow_callback,
            qos_profile,
            callback_group=self.flow_cb_group
        )
        
        # self.flow_sub = self.create_subscription(
        #     Vector3Stamped,
        #     '/optical_flow/LK_velocity',
        #     self.flow_callback,
        #     qos_profile,
        #     callback_group=self.flow_cb_group
        # )
        
        # Publishers
        self.p_publish = self.create_publisher(Float64MultiArray, 'kalman/p_matrix', 10)
        self.state_pub = self.create_publisher(Vector3Stamped, '/kalman_filter/state', 10)
        self.imu_filt_pub = self.create_publisher(Float64, '/kalman_filter/imu_filtered', 10)
        self.imu_un_filt_pub = self.create_publisher(Float64, '/kalman_filter/imu_unfiltered', 10)
        
        # Timer
        self.publish_timer = self.create_timer(dt, self.timer_callback, callback_group=self.timer_cb_group)
        self.gain_timer = self.create_timer(1.0, self.kalman_gain_callback, callback_group=self.gain_cb_group)
        
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
        
        if self.last_imu_time is None:
            dt = self.kf.dt_default
        else:
            dt = imu_time - self.last_imu_time
            if dt <= 0:
                self.get_logger().warn("Non-positive dt detected, using default")
                dt = self.kf.dt_default
        
        self.dt_sum += dt
        self.dt_count += 1
        
        self.last_imu_time = imu_time
        
        # Apply low-pass filter to acceleration
        filtered_acceleration = self.low_pass_filter(acceleration)
        
        # # Subtract fixed startup bias
        # corr_accel = filtered_acceleration - self.initial_bias
        
        # self.velocity_x += corr_accel * dt
        
        self.velocity_x = filtered_acceleration * dt
        
        self.imu_filt_pub.publish(Float64(data=self.velocity_x))
        
        self.velocity_unfiltered_x += filtered_acceleration * dt
        self.imu_un_filt_pub.publish(Float64(data=self.velocity_unfiltered_x))
        
        # Update Kalman filter with filtered acceleration
        self.kf.predict(filtered_acceleration, dt=dt)
        
        # Buffer the current state
        if self.enable_oosm:
            self.state_history.append({
                'time': imu_time,
                'x_hat': self.kf.x_hat.copy(),
                'P': self.kf.P.copy(),
                'accel': filtered_acceleration,
            })
            self.prune_state_history()

    def flow_callback(self, msg):
        """Handle optical flow measurements with OOSM."""
        velocity = msg.vector.x
        flow_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        
        if abs(velocity) > 8.0:
            self.get_logger().warn(f"Skipped velocity update: {velocity} exceeds threshold")
            return

        if not self.enable_oosm:
            self.kf.update(np.array([velocity]))
            self.publish_state()
            return

        if not self.state_history:
            self.get_logger().warn("No buffered IMU states available. Skipping OOSM update.")
            return

        # Find the last entry ≤ flow_time
        oosm_index = None
        for i, entry in enumerate(self.state_history):
            if entry['time'] > flow_time:
                break
            oosm_index = i

        if oosm_index is None:
            self.get_logger().warn("No buffered state older than flow time. Using standard update.")
            self.kf.update(np.array([velocity]))
            self.publish_state()
            return

        # Roll back to the buffered state
        state_entry = self.state_history[oosm_index]
        self.kf.x_hat = state_entry['x_hat'].copy()
        self.kf.P = state_entry['P'].copy()
        rollback_time = state_entry['time']

        # Predict to flow_time
        dt1 = flow_time - rollback_time
        if dt1 > 1e-9:
            self.kf.predict(state_entry['accel'], dt=dt1)

        # Update with the measurement
        self.kf.update(np.array([velocity]))

        # Finish the IMU interval if next state exists
        if oosm_index + 1 < len(self.state_history):
            next_entry = self.state_history[oosm_index + 1]
            dt2 = next_entry['time'] - flow_time
            if dt2 > 1e-9:
                self.kf.predict(next_entry['accel'], dt=dt2)

            # Replay subsequent IMU steps
            previous_time = next_entry['time']
            for entry in self.state_history[oosm_index + 2:]:
                dt = entry['time'] - previous_time
                self.kf.predict(entry['accel'], dt=dt)
                previous_time = entry['time']
        else:
            self.get_logger().info("No subsequent states to replay.")

        # Prune and publish
        self.prune_state_history()
        self.publish_state()

    def timer_callback(self):
        self.publish_state()
        
    def kalman_gain_callback(self):
        if self.kf.K_1 is not None:
            self.get_logger().info(f"Kalman gain for velocity: {self.kf.K_1[1]}")
        else:
            self.get_logger().info("Kalman gain not available yet")
        
    def prune_state_history(self):
        """Remove old entries from state history."""
        if self.last_imu_time is None:
            return
        threshold = 2.0  # Increased to handle larger delays
        self.state_history = [
            entry for entry in self.state_history
            if self.last_imu_time - entry['time'] < threshold
        ]

    def publish_state(self):
        state = self.kf.get_state()
        timestamp = self.get_clock().now().to_msg()
        
        state_msg = Vector3Stamped()
        state_msg.header.stamp = timestamp
        state_msg.header.frame_id = 'world'
        state_msg.vector.x = float(state[0])  # Position
        state_msg.vector.y = float(state[1])  # Velocity
        state_msg.vector.z = float(state[2])  # Bias
        self.state_pub.publish(state_msg)
        
        p_msg = Float64MultiArray()
        p_msg.data = self.kf.P.flatten().tolist()
        self.p_publish.publish(p_msg)

def main(args=None):
    rclpy.init(args=args)
    node = KalmanFilterNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()