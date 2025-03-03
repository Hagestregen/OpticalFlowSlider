#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Vector3Stamped
from std_msgs.msg import Float64
import cv2
import numpy as np
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class LucasKanadeNode(Node):
    def __init__(self):
        super().__init__('lucas_kanade_node')

        # Create a QoS profile for images.
        qos_profile = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST
        )

        # Subscribe to the image topic.
        self.subscription = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.image_callback,
            qos_profile
        )

        # Publisher for the computed velocity vector with header.
        self.velocity_pub = self.create_publisher(Vector3Stamped, '/optical_flow/LK_velocity', qos_profile) #Lucas-Kanade
        

        self.bridge = CvBridge()
        self.prev_gray = None
        self.prev_points = None
        self.prev_stamp = None

        # Parameters for Shi-Tomasi feature detection.
        self.feature_params = dict(
            maxCorners=100,
            qualityLevel=0.3,
            minDistance=7,
            blockSize=7
        )

        # Parameters for Lucas-Kanade optical flow.
        self.lk_params = dict(
            winSize=(15, 15),
            maxLevel=2,
            criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03)
        )

        # Conversion factor: physical meters per pixel.
        # (For example, if 1000 pixels correspond to 1 meter, then factor = 0.001)
        self.pixel_to_meter = 0.000566   # Adjust this value based on your calibration.

        self.get_logger().info('Lucas-Kanade Optical Flow Node has been started.')

    def image_callback(self, msg: Image):
        try:
            # Convert ROS Image to OpenCV image (assumes BGR8 encoding).
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Error converting image: {e}")
            return

        # Convert the frame to grayscale.
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Retrieve current timestamp from the image header (in seconds).
        current_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        if self.prev_gray is None:
            # For the first frame, initialize the previous image, features, and timestamp.
            self.prev_gray = gray
            self.prev_points = cv2.goodFeaturesToTrack(gray, mask=None, **self.feature_params)
            self.prev_stamp = current_stamp
            return

        # Calculate the time difference (dt) between frames.
        dt = current_stamp - self.prev_stamp
        if dt <= 0:
            dt = 1e-3  # avoid division by zero

        # Calculate optical flow from the previous frame to the current frame.
        next_points, status, error = cv2.calcOpticalFlowPyrLK(self.prev_gray, gray, self.prev_points, None, **self.lk_params)
        if next_points is None or status is None:
            self.get_logger().warn("No optical flow found; reinitializing features.")
            self.prev_points = cv2.goodFeaturesToTrack(gray, mask=None, **self.feature_params)
            self.prev_gray = gray
            self.prev_stamp = current_stamp
            return

        # Select good points where tracking was successful.
        good_new = next_points[status == 1]
        good_old = self.prev_points[status == 1]

        # Compute displacement vectors (in pixels) for each feature.
        displacements = good_new - good_old  # shape: (n,2)
        if displacements.size == 0:
            mean_disp = np.array([0.0, 0.0])
        else:
            # Compute the mean displacement vector.
            mean_disp = np.mean(displacements, axis=0)

        # Compute the velocity in pixels per second.
        velocity_pixels = mean_disp / dt

        # Convert the pixel velocity to physical units (meters per second).
        velocity_mps = velocity_pixels * self.pixel_to_meter

        # Create a Vector3Stamped message and populate header and vector.
        vel_msg = Vector3Stamped()
        vel_msg.header = msg.header  # Use the same header from the image
        vel_msg.vector.x = float(velocity_mps[0])
        vel_msg.vector.y = float(velocity_mps[1])
        vel_msg.vector.z = 0.0  # assuming 2D motion

        # Publish the optical flow velocity with header.
        self.velocity_pub.publish(vel_msg)
        self.get_logger().info(f"Published optical flow velocity: {vel_msg.vector.x:.3f} m/s")

        # (Optional) Visualization: draw optical flow tracks.
        mask = np.zeros_like(frame)
        for new, old in zip(good_new, good_old):
            a, b = new.ravel().astype(int)
            c, d = old.ravel().astype(int)
            mask = cv2.line(mask, (a, b), (c, d), (0, 255, 0), 2)
            frame = cv2.circle(frame, (a, b), 5, (0, 0, 255), -1)
        output = cv2.add(frame, mask)
        # cv2.imshow("Lucas-Kanade Optical Flow", output)
        # cv2.waitKey(1)

        # Update the previous frame, feature points, and timestamp for the next callback.
        self.prev_gray = gray.copy()
        self.prev_points = good_new.reshape(-1, 1, 2)
        self.prev_stamp = current_stamp

def main(args=None):
    rclpy.init(args=args)
    node = LucasKanadeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt, shutting down.")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()




# #!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image
# from geometry_msgs.msg import Vector3
# from std_msgs.msg import Float64
# import cv2
# import numpy as np
# from cv_bridge import CvBridge
# from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

# class LucasKanadeNode(Node):
#     def __init__(self):
#         super().__init__('lucas_kanade_node')

#         # Create a QoS profile for images.
#         qos_profile = QoSProfile(
#             depth=10,
#             reliability=QoSReliabilityPolicy.BEST_EFFORT,
#             history=QoSHistoryPolicy.KEEP_LAST
#         )

#         # Subscribe to the image topic.
#         self.subscription = self.create_subscription(
#             Image,
#             '/camera/camera/color/image_raw',
#             self.image_callback,
#             qos_profile
#         )

#         # Publisher for the computed velocity vector.
#         # self.velocity_pub = self.create_publisher(Vector3, '/optical_flow/LK_velocity', qos_profile)
#         self.velocity_pub = self.create_publisher(Float64, '/optical_flow/LK_velocity', qos_profile)
#         # self.velocity_divided_pub = self.create_publisher(Float64, '/optical_flow_velocity_divided', qos_profile)

#         self.bridge = CvBridge()
#         self.prev_gray = None
#         self.prev_points = None
#         self.prev_stamp = None

#         # Parameters for Shi-Tomasi feature detection.
#         self.feature_params = dict(
#             maxCorners=100,
#             qualityLevel=0.3,
#             minDistance=7,
#             blockSize=7
#         )

#         # Parameters for Lucas-Kanade optical flow.
#         self.lk_params = dict(
#             winSize=(15, 15),
#             maxLevel=2,
#             criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03)
#         )

#         # Conversion factor: physical meters per pixel.
#         # (For example, if 1000 pixels correspond to 1 meter, then factor = 0.001)
#         self.pixel_to_meter = 0.000566   # Adjust this value based on your calibration.

#         self.get_logger().info('Lucas-Kanade Optical Flow Node has been started.')

#     def image_callback(self, msg: Image):
#         try:
#             # Convert ROS Image to OpenCV image (assumes BGR8 encoding).
#             frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
#         except Exception as e:
#             self.get_logger().error(f"Error converting image: {e}")
#             return

#         # Convert the frame to grayscale.
#         gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

#         # Retrieve current timestamp from the image header (in seconds).
#         current_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

#         if self.prev_gray is None:
#             # For the first frame, initialize the previous image, features, and timestamp.
#             self.prev_gray = gray
#             self.prev_points = cv2.goodFeaturesToTrack(gray, mask=None, **self.feature_params)
#             self.prev_stamp = current_stamp
#             return

#         # Calculate the time difference (dt) between frames.
#         dt = current_stamp - self.prev_stamp
#         if dt <= 0:
#             dt = 1e-3  # avoid division by zero

#         # Calculate optical flow from the previous frame to the current frame.
#         next_points, status, error = cv2.calcOpticalFlowPyrLK(self.prev_gray, gray, self.prev_points, None, **self.lk_params)
#         if next_points is None or status is None:
#             self.get_logger().warn("No optical flow found; reinitializing features.")
#             self.prev_points = cv2.goodFeaturesToTrack(gray, mask=None, **self.feature_params)
#             self.prev_gray = gray
#             self.prev_stamp = current_stamp
#             return

#         # Select good points where tracking was successful.
#         good_new = next_points[status == 1]
#         good_old = self.prev_points[status == 1]

#         # Compute displacement vectors (in pixels) for each feature.
#         displacements = good_new - good_old  # shape: (n,2)
#         if displacements.size == 0:
#             mean_disp = np.array([0.0, 0.0])
#         else:
#             # Compute the mean displacement vector.
#             mean_disp = np.mean(displacements, axis=0)

#         # Compute the velocity in pixels per second.
#         velocity_pixels = mean_disp / dt

#         # Convert the pixel velocity to physical units (meters per second).
#         velocity_mps = velocity_pixels * self.pixel_to_meter

#         # Create a Vector3 message and publish the velocity.
#         vel_msg = Vector3()
#         vel_msg.x = float(velocity_mps[0])
#         vel_msg.y = float(velocity_mps[1])
#         vel_msg.z = 0.0  # assuming 2D motion
#         # print("current velocity", vel_msg.x)

#         # divided_msg = Float64()
#         # divided_msg.data = vel_msg.x / 2
#         vel_msg_output = Float64()
#         vel_msg_output.data = vel_msg.x
#         self.velocity_pub.publish(vel_msg_output)
#         # self.velocity_divided_pub.publish(divided_msg)

#         # (Optional) Visualization: draw optical flow tracks.
#         mask = np.zeros_like(frame)
#         for new, old in zip(good_new, good_old):
#             a, b = new.ravel().astype(int)
#             c, d = old.ravel().astype(int)
#             mask = cv2.line(mask, (a, b), (c, d), (0, 255, 0), 2)
#             frame = cv2.circle(frame, (a, b), 5, (0, 0, 255), -1)
#         output = cv2.add(frame, mask)
#         # cv2.imshow("Lucas-Kanade Optical Flow", output)
#         # cv2.waitKey(1)

#         # Update the previous frame, feature points, and timestamp for the next callback.
#         self.prev_gray = gray.copy()
#         self.prev_points = good_new.reshape(-1, 1, 2)
#         self.prev_stamp = current_stamp

# def main(args=None):
#     rclpy.init(args=args)
#     node = LucasKanadeNode()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         node.get_logger().info("Keyboard interrupt, shutting down.")
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()
#         cv2.destroyAllWindows()

# if __name__ == '__main__':
#     main()
