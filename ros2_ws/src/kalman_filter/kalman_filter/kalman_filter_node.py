#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Float32MultiArray
from geometry_msgs.msg import Vector3Stamped
from sensor_msgs.msg import Imu
from kalman_filter import KalmanFilter
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import numpy as np

class KalmanFilterNode(Node):
    def __init__(self):
        super().__init__('kalman_filter_node')

        # QoS profile for optical flow messages.
        qos_profile = QoSProfile(
            depth=50,  # increase depth to store enough history
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST
        )
        
        # Initialize the Kalman Filter.
        self.kf = KalmanFilter(dt=0.01, sigma_a=0.1, sigma_flow=0.05, sigma_b=0.001)
        
        # Buffer to store IMU-based states.
        # Each entry is a dictionary: {'time': timestamp, 'x_hat': state, 'P': covariance, 'accel': acceleration}
        self.state_history = []
        
        # Latest IMU time (current filter time)
        self.last_imu_time = None

        # Subscribe to the IMU topic.
        self.imu_sub = self.create_subscription(
            Imu,                  
            '/inertialsense/imu',      
            self.imu_callback,        
            10
        )
        
        # Subscribe to the optical flow velocity topic (with header).
        # self.flow_sub = self.create_subscription(
        #     Vector3Stamped,
        #     '/optical_flow/LK_velocity',
        #     self.flow_callback,
        #     qos_profile
        # )
        
        self.flow_sub = self.create_subscription(
            Vector3Stamped,
            '/optical_flow/LFN_velocity',
            self.flow_callback,
            qos_profile
        )
        
        # Publishers for full state and velocity only.
        self.state_pub = self.create_publisher(
            Float32MultiArray,        
            '/kalman_filter/state',   
            10
        )
        self.state_vel_pub = self.create_publisher(
            Float64,
            '/kalman_filter/velocity',
            10
        )
        
        # Timer to publish the current state at regular intervals.
        self.timer = self.create_timer(0.01, self.timer_callback)
        
        self.get_logger().info('Kalman Filter Node (with OOSM) is up and running!')

    def imu_callback(self, msg):
        """
        On each IMU message, perform a prediction step and save the state into the history.
        """
        acceleration = msg.linear_acceleration.x
        imu_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        # dt = self.kf.dt_default if self.last_imu_time is None else max(imu_time - self.last_imu_time, 1e-3)
        # # Compute dt from the previous IMU message.
        if self.last_imu_time is None:
            dt = self.kf.dt_default
        else:
            dt = imu_time - self.last_imu_time
            if dt <= 0:
                dt = self.kf.dt_default
        
        # Propagate the filter state.
        self.kf.predict(acceleration, dt=dt)
        
        # Update the latest time.
        self.last_imu_time = imu_time
        
        # Buffer the current state (store copies to avoid mutation).
        self.state_history.append({
            'time': imu_time,
            'x_hat': self.kf.x_hat.copy(),
            'P': self.kf.P.copy(),
            'accel': acceleration
        })
        
        # Optional: prune old history (e.g., older than 2 seconds) to avoid unbounded growth.
        current_time = imu_time
        self.state_history = [entry for entry in self.state_history if current_time - entry['time'] < 2.0]

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

        # Ensure we have history.
        if not self.state_history:
            self.get_logger().warn("No buffered IMU states available. Skipping OOSM update.")
            return

        # Find the most recent state before (or equal to) the flow measurement time.
        # (We assume the state_history is time-ordered.)
        oosm_index = None
        for i, entry in enumerate(self.state_history):
            if entry['time'] > flow_time:
                break
            oosm_index = i

        if oosm_index is None:
            self.get_logger().warn("No buffered state is older than the optical flow measurement time.")
            return

        # Backup the current (latest) filter state.
        backup_state = {
            'x_hat': self.kf.x_hat.copy(),
            'P': self.kf.P.copy()
        }
        
        # Roll back the filter to the state at the measurement time.
        state_entry = self.state_history[oosm_index]
        self.kf.x_hat = state_entry['x_hat'].copy()
        self.kf.P = state_entry['P'].copy()
        rollback_time = state_entry['time']
        # self.get_logger().info(f"Rolling back filter to time {rollback_time:.3f} for OOSM update.")

        # Apply the measurement update at the time of the optical flow.
        self.kf.update(velocity)
        # self.get_logger().info(f"Applied optical flow update (velocity={velocity:.3f} m/s) at time {flow_time:.3f}")

        # Replay all IMU predictions from rollback_time to current time.
        replay_entries = [entry for entry in self.state_history if entry['time'] > rollback_time]
        # Sort entries by time (should already be sorted)
        replay_entries.sort(key=lambda e: e['time'])
        previous_time = rollback_time
        for entry in replay_entries:
            dt = entry['time'] - previous_time
            # Use the stored acceleration for that step.
            self.kf.predict(entry['accel'], dt=dt)
            previous_time = entry['time']

        # self.get_logger().info("Replayed IMU measurements after OOSM update.")

        # (Optionally) update the state_history with the new filter state.
        # For simplicity, we leave the buffer as is.
        # You might also want to remove entries older than a threshold.
        self.prune_state_history()
        self.publish_state()

    def timer_callback(self):
        self.publish_state()
        
    def prune_state_history(self):
        """
        Remove old entries from the state history that are older than a given threshold.
        This helps keep the buffer size manageable.
        """
        if self.last_imu_time is None:
            return  # nothing to prune yet

        threshold = 2.0  # seconds; adjust as needed
        # Keep only entries where the time difference is less than the threshold.
        self.state_history = [
            entry for entry in self.state_history
            if self.last_imu_time - entry['time'] < threshold
        ]


    def publish_state(self):
        state = self.kf.get_state()
        state_msg = Float32MultiArray()
        state_msg.data = state.tolist()
        self.state_pub.publish(state_msg)
        
        velocity_msg = Float64()
        velocity_msg.data = state[1]  # the velocity element
        self.state_vel_pub.publish(velocity_msg)
        
        # self.get_logger().debug(f"Published state: {state_msg.data} and velocity: {velocity_msg.data}")

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



