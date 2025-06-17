
# # import rclpy
# # from rclpy.node import Node
# # from std_msgs.msg import Float64, Float64MultiArray
# # from geometry_msgs.msg import Vector3Stamped
# # from sensor_msgs.msg import Imu
# # from kalman_filter import KalmanFilter
# # from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
# # import numpy as np
# # from scipy.signal import lfilter, butter
# # from rclpy.executors import MultiThreadedExecutor
# # from rclpy.callback_groups import ReentrantCallbackGroup

# # class KalmanFilterNode(Node):
# #     def __init__(self):
# #         super().__init__('kalman_filter_node')
        
# #         # Create separate callback groups
# #         self.imu_cb_group   = ReentrantCallbackGroup()
# #         self.flow_cb_group  = ReentrantCallbackGroup()
# #         self.timer_cb_group = ReentrantCallbackGroup()
# #         self.gain_cb_group  = ReentrantCallbackGroup()
        
# #         # Declare parameters for Kalman filter initialization
# #         self.declare_parameter('dt',                0.014)
# #         self.declare_parameter('sigma_a',           0.5)
# #         self.declare_parameter('sigma_flow',        0.01)
# #         self.declare_parameter('sigma_b',           0.001)
        
# #         # Parameters for IMU filtering
# #         self.declare_parameter('imu_cutoff_freq',   2.5)
# #         self.declare_parameter('imu_sampling_freq', 71)
# #         self.declare_parameter('imu_filter_order',  2)
        
# #         # OOSM
# #         self.declare_parameter('enable_oosm',       True)
        
# #         # Get parameter values
# #         dt                  = self.get_parameter('dt').value
# #         sigma_a             = self.get_parameter('sigma_a').value
# #         sigma_flow          = self.get_parameter('sigma_flow').value
# #         sigma_b             = self.get_parameter('sigma_b').value
# #         self.imu_cutoff     = self.get_parameter('imu_cutoff_freq').value
# #         self.imu_fs         = self.get_parameter('imu_sampling_freq').value
# #         self.imu_order      = self.get_parameter('imu_filter_order').value
# #         self.enable_oosm    = self.get_parameter('enable_oosm').value
        
# #         self.calib_duration_s   = 1.0
# #         self.calib_samples      = []
# #         self.calib_target_count = int(self.calib_duration_s / dt)
# #         self.initial_bias       = 0.0
# #         self.calibrated         = False
# #         self.initial_bias       = 0.0625
        
# #         self.velocity_x         = 0.0 
# #         self.velocity_unfiltered_x = 0.0
# #         self.dt_sum             = 0.0
# #         self.dt_count           = 0
        
# #         # QoS profile for optical flow messages
# #         qos_profile = QoSProfile(
# #             depth=50,
# #             reliability=QoSReliabilityPolicy.BEST_EFFORT,
# #             history=QoSHistoryPolicy.KEEP_LAST
# #         )
        
# #         # Initialize the Kalman Filter with parameters
# #         self.kf = KalmanFilter(
# #             dt=dt, 
# #             sigma_a=sigma_a, 
# #             sigma_flow=sigma_flow, 
# #             sigma_b=sigma_b)
        
# #         self.get_logger().info(
# #             f"Kalman Filter initialized with dt={dt}, sigma_a={sigma_a}, "
# #             f"sigma_flow={sigma_flow}, sigma_b={sigma_b}"
# #         )
        
# #         # State and timing variables
# #         self.state_history = []
# #         self.last_imu_time = None
        
# #         # Filter state for incremental filtering of acceleration
# #         self.filter_state = np.zeros(self.imu_order)
        
# #         # Subscriptions
# #         self.imu_sub = self.create_subscription(
# #             Imu,
# #             '/inertialsense/imu',
# #             self.imu_callback,
# #             10,
# #             callback_group=self.imu_cb_group
# #         )
        
# #         self.flow_sub = self.create_subscription(
# #             Vector3Stamped,
# #             '/optical_flow/LFN3_velocity',
# #             self.flow_callback,
# #             qos_profile,
# #             callback_group=self.flow_cb_group
# #         )
        
# #         # self.flow_sub = self.create_subscription(
# #         #     Vector3Stamped,
# #         #     '/optical_flow/LK_smooth_velocity',
# #         #     self.flow_callback,
# #         #     qos_profile,
# #         #     callback_group=self.flow_cb_group
# #         # )
        
# #         # self.flow_sub = self.create_subscription(
# #         #     Vector3Stamped,
# #         #     '/optical_flow/raft_large_smooth_velocity',
# #         #     self.flow_callback,
# #         #     qos_profile,
# #         #     callback_group=self.flow_cb_group
# #         # )
        
# #         # Publishers
# #         self.p_publish = self.create_publisher(Float64MultiArray, 'kalman/p_matrix', 10)
# #         self.state_pub = self.create_publisher(Vector3Stamped, '/kalman_filter/state', 10)
# #         self.imu_filt_pub = self.create_publisher(Float64, '/kalman_filter/imu_filtered', 10)
# #         self.imu_un_filt_pub = self.create_publisher(Float64, '/kalman_filter/imu_unfiltered', 10)
        
# #         # Timer
# #         self.publish_timer = self.create_timer(dt, self.timer_callback, callback_group=self.timer_cb_group)
# #         self.gain_timer = self.create_timer(1.0, self.kalman_gain_callback, callback_group=self.gain_cb_group)
        
# #         self.get_logger().info('Kalman Filter Node (with OOSM) is up and running!')

# #     def low_pass_filter(self, new_value):
# #         """Apply low-pass filter incrementally to a new acceleration value."""
# #         nyquist = self.imu_fs / 2
# #         normal_cutoff = self.imu_cutoff / nyquist
# #         b, a = butter(self.imu_order, normal_cutoff, btype='low', analog=False)
# #         output, self.filter_state = lfilter(b, a, [new_value], zi=self.filter_state)
# #         return output[0]

# #     def imu_callback(self, msg):
# #         """Process IMU data, filter acceleration, and update Kalman filter."""
# #         acceleration = msg.linear_acceleration.x
# #         imu_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        
# #         # Calibrate IMU Bias
# #         if not self.calibrated:
# #             self.calib_samples.append(acceleration)
# #             self.last_imu_time = imu_time
# #             if len(self.calib_samples) >= self.calib_target_count:
# #                 self.initial_bias = float(np.mean(self.calib_samples))
# #                 self.kf.x_hat[2, 0] = self.initial_bias
# #                 self.calibrated = True
# #                 self.get_logger().info(
# #                     f"IMU bias calibrated: {self.initial_bias:.4f} m/s²"
# #                 )
# #                 filt0 = self.low_pass_filter(acceleration)
# #                 corr0 = filt0 - self.initial_bias
# #                 self.state_history.append({
# #                     'time':  imu_time,
# #                     'x_hat': self.kf.x_hat.copy(),
# #                     'P':     self.kf.P.copy(),
# #                     'accel': corr0,
# #                 })
# #             return
        
# #         if self.last_imu_time is None:
# #             dt = self.kf.dt_default
# #         else:
# #             dt = imu_time - self.last_imu_time
# #             if dt <= 0:
# #                 self.get_logger().warn("Non-positive dt detected, using default")
# #                 dt = self.kf.dt_default
        
# #         self.dt_sum += dt
# #         self.dt_count += 1
        
# #         self.last_imu_time = imu_time
        
# #         # Apply low-pass filter to acceleration
# #         filtered_acceleration = self.low_pass_filter(acceleration)
        
# #         # Subtract fixed startup bias
# #         corr_accel = filtered_acceleration - self.initial_bias
        
# #         self.velocity_x += corr_accel * dt
# #         self.imu_filt_pub.publish(Float64(data=self.velocity_x))
        
# #         self.velocity_unfiltered_x += filtered_acceleration * dt
# #         self.imu_un_filt_pub.publish(Float64(data=self.velocity_unfiltered_x))
        
# #         # Update Kalman filter with filtered acceleration
# #         self.kf.predict(filtered_acceleration, dt=dt)
        
# #         # Buffer the current state
# #         if self.enable_oosm:
# #             self.state_history.append({
# #                 'time': imu_time,
# #                 'x_hat': self.kf.x_hat.copy(),
# #                 'P': self.kf.P.copy(),
# #                 'accel': filtered_acceleration,
# #             })
# #             self.prune_state_history()

# #     def flow_callback(self, msg):
# #         """Handle optical flow measurements with OOSM."""
# #         velocity = msg.vector.x
# #         flow_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        
# #         if abs(velocity) > 8.0:
# #             self.get_logger().warn(f"Skipped velocity update: {velocity} exceeds threshold")
# #             return

# #         if not self.enable_oosm:
# #             self.kf.update(np.array([velocity]))
# #             self.publish_state()
# #             return

# #         if not self.state_history:
# #             self.get_logger().warn("No buffered IMU states available. Skipping OOSM update.")
# #             return

# #         # Find the last entry ≤ flow_time
# #         oosm_index = None
# #         for i, entry in enumerate(self.state_history):
# #             if entry['time'] > flow_time:
# #                 break
# #             oosm_index = i

# #         if oosm_index is None:
# #             self.get_logger().warn("No buffered state older than flow time. Using standard update.")
# #             self.kf.update(np.array([velocity]))
# #             self.publish_state()
# #             return

# #         # Roll back to the buffered state
# #         state_entry = self.state_history[oosm_index]
# #         self.kf.x_hat = state_entry['x_hat'].copy()
# #         self.kf.P = state_entry['P'].copy()
# #         rollback_time = state_entry['time']

# #         # Predict to flow_time
# #         dt1 = flow_time - rollback_time
# #         if dt1 > 1e-9:
# #             self.kf.predict(state_entry['accel'], dt=dt1)

# #         # Update with the measurement
# #         self.kf.update(np.array([velocity]))

# #         # Finish the IMU interval if next state exists
# #         if oosm_index + 1 < len(self.state_history):
# #             next_entry = self.state_history[oosm_index + 1]
# #             dt2 = next_entry['time'] - flow_time
# #             if dt2 > 1e-9:
# #                 self.kf.predict(next_entry['accel'], dt=dt2)

# #             # Replay subsequent IMU steps
# #             previous_time = next_entry['time']
# #             for entry in self.state_history[oosm_index + 2:]:
# #                 dt = entry['time'] - previous_time
# #                 self.kf.predict(entry['accel'], dt=dt)
# #                 previous_time = entry['time']
# #         else:
# #             self.get_logger().info("No subsequent states to replay.")

# #         # Prune and publish
# #         self.prune_state_history()
# #         self.publish_state()

# #     def timer_callback(self):
# #         self.publish_state()
        
# #     def kalman_gain_callback(self):
# #         if self.kf.K_1 is not None:
# #             self.get_logger().info(f"Kalman gain for velocity: {self.kf.K_1[1]}")
# #         else:
# #             self.get_logger().info("Kalman gain not available yet")
        
# #     def prune_state_history(self):
# #         """Remove old entries from state history."""
# #         if self.last_imu_time is None:
# #             return
# #         threshold = 2.0  # Increased to handle larger delays
# #         self.state_history = [
# #             entry for entry in self.state_history
# #             if self.last_imu_time - entry['time'] < threshold
# #         ]

# #     def publish_state(self):
# #         state = self.kf.get_state()
# #         timestamp = self.get_clock().now().to_msg()
        
# #         state_msg = Vector3Stamped()
# #         state_msg.header.stamp = timestamp
# #         state_msg.header.frame_id = 'world'
# #         state_msg.vector.x = float(state[0])  # Position
# #         state_msg.vector.y = float(state[1])  # Velocity
# #         state_msg.vector.z = float(state[2])  # Bias
# #         self.state_pub.publish(state_msg)
        
# #         p_msg = Float64MultiArray()
# #         p_msg.data = self.kf.P.flatten().tolist()
# #         self.p_publish.publish(p_msg)

# # def main(args=None):
# #     rclpy.init(args=args)
# #     node = KalmanFilterNode()
# #     executor = MultiThreadedExecutor()
# #     executor.add_node(node)
# #     try:
# #         executor.spin()
# #     except KeyboardInterrupt:
# #         pass
# #     finally:
# #         node.destroy_node()
# #         rclpy.shutdown()

# # if __name__ == '__main__':
# #     main()
    
# # kalman_filter_node.py
# #!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import Float64, Float64MultiArray
# from geometry_msgs.msg import Vector3Stamped
# from sensor_msgs.msg import Imu
# from kalman_filter import KalmanFilter
# from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
# from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
# import numpy as np
# from scipy.signal import lfilter, butter
# import threading

# class KalmanFilterNode(Node):
#     def __init__(self):
#         super().__init__('kalman_filter_node')

#         # ── SERIALIZE ALL KF ACCESS ──
#         self.cb_group = MutuallyExclusiveCallbackGroup()
#         self._kf_lock = threading.Lock()

#         # ── PARAMETERS ──
#         self.declare_parameter('dt',               0.014)
#         self.declare_parameter('sigma_a',          0.5)
#         self.declare_parameter('sigma_flow',       0.05)   # raised from 0.01
#         self.declare_parameter('sigma_b',          0.001)
#         self.declare_parameter('imu_cutoff_freq',  2.5)
#         self.declare_parameter('imu_sampling_freq',71.4)
#         self.declare_parameter('imu_filter_order', 2)
#         self.declare_parameter('enable_oosm',      True)

#         dt           = self.get_parameter('dt').value
#         sigma_a      = self.get_parameter('sigma_a').value
#         sigma_flow   = self.get_parameter('sigma_flow').value
#         sigma_b      = self.get_parameter('sigma_b').value
#         self.imu_cutoff = self.get_parameter('imu_cutoff_freq').value
#         self.imu_fs     = self.get_parameter('imu_sampling_freq').value
#         self.imu_order  = self.get_parameter('imu_filter_order').value
#         self.enable_oosm = self.get_parameter('enable_oosm').value

#         # ── CALIBRATION ──
#         self.calib_duration_s   = 1.0
#         self.calib_target_count = int(self.calib_duration_s / dt)
#         self.calib_samples      = []
#         self.initial_bias       = 0.0
#         self.calibrated         = False

#         # ── DEBUG VELOCITIES ──
#         self.velocity_x            = 0.0
#         self.velocity_unfiltered_x = 0.0

#         # ── TIMING ──
#         self.last_imu_time = None

#         # ── PRECOMPUTE LOW‐PASS FILTER ──
#         nyq = self.imu_fs / 2.0
#         wn  = self.imu_cutoff / nyq
#         self.b, self.a      = butter(self.imu_order, wn, btype='low', analog=False)
#         self.filter_state   = np.zeros(max(len(self.a), len(self.b)) - 1)

#         # ── OOSM BUFFER ──
#         self.state_history = []

#         # ── KALMAN FILTER ──
#         self.kf = KalmanFilter(
#             dt=dt,
#             sigma_a=sigma_a,
#             sigma_flow=sigma_flow,
#             sigma_b=sigma_b
#         )
#         self.get_logger().info(
#             f"KF init: dt={dt}, σ_a={sigma_a}, σ_flow={sigma_flow}, σ_b={sigma_b}"
#         )

#         # ── QoS FOR OPTICAL FLOW ──
#         qos = QoSProfile(
#             depth=50,
#             reliability=QoSReliabilityPolicy.BEST_EFFORT,
#             history=QoSHistoryPolicy.KEEP_LAST
#         )

#         # ── SUBSCRIPTIONS ──
#         self.imu_sub = self.create_subscription(
#             Imu,
#             '/inertialsense/imu',
#             self.imu_callback,
#             10,
#             callback_group=self.cb_group
#         )
#         self.flow_sub = self.create_subscription(
#             Vector3Stamped,
#             '/optical_flow/LFN3_velocity',
#             self.flow_callback,
#             qos,
#             callback_group=self.cb_group
#         )
#         # 
#         # self.flow_sub = self.create_subscription(
#             # Vector3Stamped,
#             # '/optical_flow/LFN3_smooth_velocity',
#             # self.flow_callback,
#             # qos,
#             # callback_group=self.cb_group
#         # )

#         # ── PUBLISHERS ──
#         self.p_pub        = self.create_publisher(Float64MultiArray, 'kalman/p_matrix', 10)
#         self.state_pub    = self.create_publisher(Vector3Stamped,   '/kalman_filter/state', 10)
#         self.imu_filt_pub = self.create_publisher(Float64,            '/kalman_filter/imu_filtered', 10)
#         self.imu_raw_pub  = self.create_publisher(Float64,            '/kalman_filter/imu_unfiltered', 10)

#         # ── TIMERS ──
#         self.create_timer(dt,  self.timer_callback,       callback_group=self.cb_group)
#         self.create_timer(1.0, self.kalman_gain_callback, callback_group=self.cb_group)
#         self.create_timer(3.0, self.log_kalman_gain,      callback_group=self.cb_group)

#         self.get_logger().info('Kalman Filter Node up and running!')

#     def low_pass_filter(self, value):
#         y, self.filter_state = lfilter(self.b, self.a, [value], zi=self.filter_state)
#         return y[0]

#     def imu_callback(self, msg):
#         with self._kf_lock:
#             accel = msg.linear_acceleration.x
#             t     = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

#             # ── CALIBRATION PHASE ──
#             if not self.calibrated:
#                 self.calib_samples.append(accel)
#                 self.last_imu_time = t
#                 if len(self.calib_samples) >= self.calib_target_count:
#                     self.initial_bias = float(np.mean(self.calib_samples))
#                     self.kf.x_hat[2,0] = self.initial_bias
#                     self.calibrated = True
#                     self.get_logger().info(f"IMU bias calibrated: {self.initial_bias:.4f}")
#                     # seed the buffer so first flow has something
#                     self.state_history.append({
#                         'time':  t,
#                         'x_hat': self.kf.x_hat.copy(),
#                         'P':     self.kf.P.copy(),
#                         'accel': 0.0,
#                     })
#                 return

#             # ── USE TRUE Δt (no clamping) ──
#             if self.last_imu_time is None:
#                 dt = self.kf.dt_default
#             else:
#                 dt = t - self.last_imu_time
#             self.last_imu_time = t

#             filt_a = self.low_pass_filter(accel)
#             corr_a = filt_a - self.initial_bias

#             # debug publishes
#             self.velocity_x            += corr_a * dt
#             self.velocity_unfiltered_x += filt_a   * dt
#             self.imu_filt_pub.publish(Float64(data=self.velocity_x))
#             self.imu_raw_pub.publish(Float64(data=self.velocity_unfiltered_x))

#             # KF predict
#             self.kf.predict(filt_a, dt)

#             # buffer state and prune to 500 ms
#             if self.enable_oosm:
#                 self.state_history.append({
#                     'time':  t,
#                     'x_hat': self.kf.x_hat.copy(),
#                     'P':     self.kf.P.copy(),
#                     'accel': filt_a,
#                 })
#                 cutoff = 0.5
#                 self.state_history = [
#                     e for e in self.state_history
#                     if (t - e['time']) < cutoff
#                 ]

#     def flow_callback(self, msg):
#         with self._kf_lock:
#             vel = msg.vector.x
#             t   = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

#             if abs(vel) > 8.0:
#                 self.get_logger().warn(f"Flow {vel:.2f} out of range")
#                 return

#             # ── FALL BACK IF NO BUFFER ──
#             if not self.state_history:
#                 self.kf.update(vel)
#                 self.publish_state()
#                 return

#             # ── FIND LAST BUFFERED IMU ≤ t ──
#             idx = None
#             for i, e in enumerate(self.state_history):
#                 if e['time'] > t:
#                     break
#                 idx = i

#             # ── FALL BACK IF NO MATCH ──
#             if idx is None:
#                 self.kf.update(vel)
#                 self.publish_state()
#                 return

#             # ── TRUE OOSM: ROLLBACK & REPLAY ──
#             entry = self.state_history[idx]
#             self.kf.x_hat = entry['x_hat'].copy()
#             self.kf.P     = entry['P'].copy()

#             dt1 = t - entry['time']
#             if dt1 > 1e-9:
#                 self.kf.predict(entry['accel'], dt1)

#             self.kf.update(vel)

#             replay_time = t
#             for next_e in self.state_history[idx+1:]:
#                 dt_n = next_e['time'] - replay_time
#                 if dt_n > 1e-9:
#                     self.kf.predict(next_e['accel'], dt_n)
#                 replay_time = next_e['time']

#             self.publish_state()

#     def timer_callback(self):
#         with self._kf_lock:
#             self.publish_state()

#     def kalman_gain_callback(self):
#         with self._kf_lock:
#             if self.kf.K_1 is not None:
#                 self.get_logger().info(f"Gain[vel]: {self.kf.K_1[1,0]:.5f}")

#     def log_kalman_gain(self):
#         with self._kf_lock:
#             if self.kf.K_1 is not None:
#                 self.get_logger().info(
#                     f"[3 s log] Kalman gain on velocity = {self.kf.K_1[1,0]:.5f}"
#                 )

#     def publish_state(self):
#         pos, vel, bias = self.kf.get_state()
#         ts = self.get_clock().now().to_msg()
#         self.smoothed_vel = (
#             self.alpha * self.smoothed_vel
#           + (1.0 - self.alpha) * vel
#         )

#         msg = Vector3Stamped()
#         msg.header.stamp = ts
#         msg.header.frame_id = 'world'
#         msg.vector.x = float(pos)
#         # msg.vector.y = float(vel)
#         msg.vector.y = float(self.smoothed_vel)
#         msg.vector.z = float(bias)
#         self.state_pub.publish(msg)

#         pmsg = Float64MultiArray()
#         pmsg.data = self.kf.P.flatten().tolist()
#         self.p_pub.publish(pmsg)


# def main(args=None):
#     rclpy.init(args=args)
#     node = KalmanFilterNode()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()

#!/usr/bin/env python3
# kalman_filter_node.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Float64MultiArray
from geometry_msgs.msg import Vector3Stamped
from sensor_msgs.msg import Imu
from kalman_filter import KalmanFilter
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
import numpy as np
from scipy.signal import lfilter, butter
import threading

class KalmanFilterNode(Node):
    def __init__(self):
        super().__init__('kalman_filter_node')

        # ── SMOOTHED VELOCITY STATE ──
        self.smoothed_vel = 0.0
        self.alpha        = 0.8   # 0 < α < 1; higher α→more smoothing/lag

        # ── SERIALIZE ALL KF ACCESS ──
        self.cb_group = MutuallyExclusiveCallbackGroup()
        self._kf_lock = threading.Lock()

        # ── PARAMETERS ──
        self.declare_parameter('dt',                0.014)
        self.declare_parameter('sigma_a',           0.5)
        self.declare_parameter('sigma_flow',        0.05)
        self.declare_parameter('sigma_b',           0.001)
        self.declare_parameter('imu_cutoff_freq',   2.5)
        self.declare_parameter('imu_sampling_freq', 71.4)
        self.declare_parameter('imu_filter_order',  2)
        self.declare_parameter('enable_oosm',       False)

        dt            = self.get_parameter('dt').value
        sigma_a       = self.get_parameter('sigma_a').value
        sigma_flow    = self.get_parameter('sigma_flow').value
        sigma_b       = self.get_parameter('sigma_b').value
        self.imu_cutoff = self.get_parameter('imu_cutoff_freq').value
        self.imu_fs     = self.get_parameter('imu_sampling_freq').value
        self.imu_order  = self.get_parameter('imu_filter_order').value
        self.enable_oosm = self.get_parameter('enable_oosm').value

        # ── CALIBRATION STATE ──
        self.calib_duration_s   = 1.0
        self.calib_target_count = int(self.calib_duration_s / dt)
        self.calib_samples      = []
        self.initial_bias       = 0.0
        self.calibrated         = False

        # ── DEBUG VELOCITIES ──
        self.velocity_x            = 0.0
        self.velocity_unfiltered_x = 0.0

        # ── TIMING ──
        self.last_imu_time = None

        # ── PRECOMPUTE LOW‐PASS FILTER ──
        nyq = self.imu_fs / 2.0
        wn  = self.imu_cutoff / nyq
        self.b, self.a    = butter(self.imu_order, wn, btype='low', analog=False)
        self.filter_state = np.zeros(max(len(self.a), len(self.b)) - 1)

        # ── OOSM BUFFER ──
        self.state_history = []

        # ── INITIALIZE KALMAN FILTER ──
        self.kf = KalmanFilter(
            dt=dt,
            sigma_a=sigma_a,
            sigma_flow=sigma_flow,
            sigma_b=sigma_b,
            initial_bias=self.initial_bias   # initialize with zero bias
        )
        self.get_logger().info(
            f"KF init: dt={dt}, σ_a={sigma_a}, σ_flow={sigma_flow}, σ_b={sigma_b}"
        )

        # ── QoS FOR OPTICAL FLOW ──
        qos = QoSProfile(
            depth=50,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST
        )

        # ── SUBSCRIPTIONS ──
        self.imu_sub = self.create_subscription(
            Imu,
            '/inertialsense/imu',
            self.imu_callback,
            10,
            callback_group=self.cb_group
        )
        # self.flow_sub = self.create_subscription(
            # Vector3Stamped,
            # '/optical_flow/LFN3_velocity',
            # self.flow_callback,
            # qos,
            # callback_group=self.cb_group
        # )
        # self.flow_sub = self.create_subscription(
        #     Vector3Stamped,
        #     '/optical_flow/LK_velocity',
        #     self.flow_callback,
        #     qos,
        #     callback_group=self.cb_group
        # )
        self.flow_sub = self.create_subscription(
            Vector3Stamped,
            '/optical_flow/raft_large_velocity',
            self.flow_callback,
            qos,
            callback_group=self.cb_group
        )

        # ── PUBLISHERS ──
        self.p_pub        = self.create_publisher(Float64MultiArray, 'kalman/p_matrix', 10)
        self.state_pub    = self.create_publisher(Vector3Stamped,   '/kalman_filter/state', 10)
        self.imu_filt_pub = self.create_publisher(Float64,            '/kalman_filter/imu_filtered', 10)
        self.imu_raw_pub  = self.create_publisher(Float64,            '/kalman_filter/imu_unfiltered', 10)

        # ── TIMERS ──
        self.create_timer(dt,  self.timer_callback,       callback_group=self.cb_group)
        self.create_timer(1.0, self.kalman_gain_callback, callback_group=self.cb_group)
        self.create_timer(3.0, self.log_kalman_gain,      callback_group=self.cb_group)

        self.get_logger().info('Kalman Filter Node up and running!')

    def low_pass_filter(self, value):
        y, self.filter_state = lfilter(self.b, self.a, [value], zi=self.filter_state)
        return y[0]

    def imu_callback(self, msg):
        with self._kf_lock:
            accel = msg.linear_acceleration.x
            t     = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

            # ── CALIBRATION PHASE ──
            if not self.calibrated:
                self.calib_samples.append(accel)
                self.last_imu_time = t
                if len(self.calib_samples) >= self.calib_target_count:
                    # compute bias
                    self.initial_bias = float(np.mean(self.calib_samples))
                    # inject into filter
                    self.kf.initial_bias = self.initial_bias
                    self.kf.x_hat[2,0]   = self.initial_bias
                    self.calibrated      = True
                    self.get_logger().info(f"IMU bias calibrated: {self.initial_bias:.4f}")

                    # seed buffer so first flow has something
                    self.state_history.append({
                        'time':  t,
                        'x_hat': self.kf.x_hat.copy(),
                        'P':     self.kf.P.copy(),
                        'accel': 0.0,
                    })
                return

            # ── USE TRUE Δt (no clamping) ──
            if self.last_imu_time is None:
                dt = self.kf.dt_default
            else:
                dt = t - self.last_imu_time
            self.last_imu_time = t

            # filter acceleration
            filt_a = self.low_pass_filter(accel)
            corr_a = filt_a - self.initial_bias

            # debug publishes
            self.velocity_x            += corr_a * dt
            self.velocity_unfiltered_x += filt_a   * dt
            self.imu_filt_pub.publish(Float64(data=self.velocity_x))
            self.imu_raw_pub.publish(Float64(data=self.velocity_unfiltered_x))

            # KF predict
            self.kf.predict(filt_a, dt)

            # buffer for OOSM and prune to 500 ms
            if self.enable_oosm:
                self.state_history.append({
                    'time':  t,
                    'x_hat': self.kf.x_hat.copy(),
                    'P':     self.kf.P.copy(),
                    'accel': filt_a,
                })
                cutoff = 0.5
                self.state_history = [
                    e for e in self.state_history
                    if (t - e['time']) < cutoff
                ]

    def flow_callback(self, msg):
        with self._kf_lock:
            vel = msg.vector.x
            t   = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

            if abs(vel) > 8.0:
                self.get_logger().warn(f"Flow {vel:.2f} out of range")
                return

            # ── FALL BACK IF NO BUFFER ──
            if not self.state_history:
                self.kf.update(vel)
                self.publish_state()
                return

            # ── FIND LAST BUFFERED IMU ≤ t ──
            idx = None
            for i, e in enumerate(self.state_history):
                if e['time'] > t:
                    break
                idx = i

            # ── FALL BACK IF NO MATCH ──
            if idx is None:
                self.kf.update(vel)
                self.publish_state()
                return

            # ── TRUE OOSM: ROLLBACK & REPLAY ──
            entry = self.state_history[idx]
            self.kf.x_hat = entry['x_hat'].copy()
            self.kf.P     = entry['P'].copy()

            dt1 = t - entry['time']
            if dt1 > 1e-9:
                self.kf.predict(entry['accel'], dt1)

            self.kf.update(vel)

            replay_time = t
            for next_e in self.state_history[idx+1:]:
                dt_n = next_e['time'] - replay_time
                if dt_n > 1e-9:
                    self.kf.predict(next_e['accel'], dt_n)
                replay_time = next_e['time']

            self.publish_state()

    def timer_callback(self):
        with self._kf_lock:
            self.publish_state()

    def kalman_gain_callback(self):
        with self._kf_lock:
            if self.kf.K_1 is not None:
                self.get_logger().info(f"Gain[vel]: {self.kf.K_1[1,0]:.5f}")

    def log_kalman_gain(self):
        with self._kf_lock:
            if self.kf.K_1 is not None:
                self.get_logger().info(
                    f"[3 s log] Kalman gain on velocity = {self.kf.K_1[1,0]:.5f}"
                )

    def publish_state(self):
        pos, vel, bias = self.kf.get_state()

        # exponential smoothing of velocity
        self.smoothed_vel = (
            self.alpha * self.smoothed_vel
          + (1.0 - self.alpha) * vel
        )

        msg = Vector3Stamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world'
        msg.vector.x = float(pos)
        msg.vector.y = float(self.smoothed_vel)  # smoothed
        msg.vector.z = float(bias)
        self.state_pub.publish(msg)

        pmsg = Float64MultiArray()
        pmsg.data = self.kf.P.flatten().tolist()
        self.p_pub.publish(pmsg)


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
