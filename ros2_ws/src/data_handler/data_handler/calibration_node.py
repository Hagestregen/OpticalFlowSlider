#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import time
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class IMUBiasCalibrator(Node):
    def __init__(self, calibration_time=5.0):
        """
        Subscribe to two IMU topics for a fixed duration (calibration_time seconds).
        Accumulate data and compute average (bias) for each sensor's x-acceleration.
        """
        super().__init__('imu_bias_calibrator')

        qos_profile = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST
        )

        self.declare_parameter('calibration_time', calibration_time)
        self.calibration_time = self.get_parameter('calibration_time').value

        self.get_logger().info(f"Starting IMU bias calibration for {self.calibration_time} seconds.")

        # Variables for accumulating data
        self.realsense_acc_sum = 0.0
        self.realsense_count = 0

        self.inertialsense_acc_sum = 0.0
        self.inertialsense_count = 0

        # Create subscribers
        self.realsense_sub = self.create_subscription(
            Imu,
            '/camera/camera/accel/sample',
            self.realsense_callback,
            qos_profile
        )
        self.inertialsense_sub = self.create_subscription(
            Imu,
            '/inertialsense/imu',
            self.inertialsense_callback,
            10
        )

        # Use a timer to stop calibration after X seconds
        self.start_time = time.time()
        self.calib_timer = self.create_timer(0.1, self.check_calibration_done)

    def realsense_callback(self, msg: Imu):
        # Accumulate x-axis acceleration from RealSense
        self.realsense_acc_sum += msg.linear_acceleration.x
        self.realsense_count += 1

    def inertialsense_callback(self, msg: Imu):
        # Accumulate x-axis acceleration from InertialSense
        self.inertialsense_acc_sum += msg.linear_acceleration.x
        self.inertialsense_count += 1

    def check_calibration_done(self):
        elapsed = time.time() - self.start_time
        if elapsed >= self.calibration_time:
            # Stop collecting data
            self.calib_timer.cancel()

            # Compute averages
            realsense_bias = 0.0
            inertialsense_bias = 0.0

            if self.realsense_count > 0:
                realsense_bias = self.realsense_acc_sum / self.realsense_count
            if self.inertialsense_count > 0:
                inertialsense_bias = self.inertialsense_acc_sum / self.inertialsense_count

            # Print out the biases
            self.get_logger().info("Calibration complete.")
            self.get_logger().info(f"RealSense x-acc bias: {realsense_bias:.6f} m/s^2 "
                                   f"(from {self.realsense_count} samples)")
            self.get_logger().info(f"InertialSense x-acc bias: {inertialsense_bias:.6f} m/s^2 "
                                   f"(from {self.inertialsense_count} samples)")

            self.get_logger().info("Shutting down calibration node.")
            self.destroy_node()
            rclpy.shutdown()
            return


def main(args=None):
    rclpy.init(args=args)
    node = IMUBiasCalibrator(calibration_time=5.0)  # or read from a config/parameter
    rclpy.spin(node)

    # while rclpy.ok():
    #     rclpy.spin_once(node, timeout_sec=0.1)


if __name__ == '__main__':
    main()
