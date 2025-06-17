# import os
# import matplotlib
# matplotlib.use('Agg')
# import matplotlib.pyplot as plt
# from rclpy.serialization import deserialize_message
# from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
# from geometry_msgs.msg import TwistStamped
# import argparse

# def main():
#     parser = argparse.ArgumentParser(description="Plot motor present velocity from a ROS2 bag file.")
#     parser.add_argument('bag_path', type=str, help="Path to the ROS2 bag file (.db3)")
#     args = parser.parse_args()
#     bag_path = args.bag_path

#     reader = SequentialReader()
#     storage_options = StorageOptions(uri=bag_path, storage_id='sqlite3')
#     converter_options = ConverterOptions('', '')

#     try:
#         reader.open(storage_options, converter_options)
#     except Exception as e:
#         print(f"Error opening bag file: {e}")
#         return

#     times = []
#     velocities = []

#     while reader.has_next():
#         topic, data, t = reader.read_next()
#         if topic == "/motor/present_velocity":
#             try:
#                 msg = deserialize_message(data, TwistStamped)
#                 ts = t / 1e9
#                 v = msg.twist.linear.x
#                 times.append(ts)
#                 velocities.append(v)
#             except Exception as e:
#                 print(f"Error deserializing message: {e}")

#     if times:
#         t0 = times[0]
#         times = [t - t0 for t in times]
#         plt.figure(figsize=(12,6))
#         plt.plot(times, velocities, color='black')
#         plt.xlabel('Time (s)')
#         plt.ylabel('Velocity')
#         plt.title('Motor Present Velocity')
#         plt.xlim(left=0)  # Ensure x-axis starts at 0
#         plt.tight_layout()
#         output_file = os.path.splitext(os.path.basename(bag_path))[0] + "_motor_velocity.png"
#         plt.savefig(output_file)
#         print(f"Plot saved to {output_file}")
#     else:
#         print("No messages found on /motor/present_velocity")

# if __name__ == "__main__":
#     main()

import os
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from rclpy.serialization import deserialize_message
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from geometry_msgs.msg import TwistStamped
import argparse

def main():
    parser = argparse.ArgumentParser(description="Plot motor present velocity from a ROS2 bag file.")
    parser.add_argument('bag_path', type=str, help="Path to the ROS2 bag file (.db3)")
    args = parser.parse_args()
    bag_path = args.bag_path

    reader = SequentialReader()
    storage_options = StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = ConverterOptions('', '')

    try:
        reader.open(storage_options, converter_options)
    except Exception as e:
        print(f"Error opening bag file: {e}")
        return

    times = []
    velocities = []

    while reader.has_next():
        topic, data, t = reader.read_next()
        if topic == "/motor/present_velocity":
            try:
                msg = deserialize_message(data, TwistStamped)
                ts = t / 1e9
                v = msg.twist.linear.x
                times.append(ts)
                velocities.append(v)
            except Exception as e:
                print(f"Error deserializing message: {e}")

    if times:
        t0 = times[0]
        times = [t - t0 for t in times]
        plt.figure(figsize=(12,6))
        plt.plot(times, velocities, color='black')
        plt.xlabel('Time (s)')
        plt.ylabel('Velocity')
        plt.title('Motor Present Velocity')
        plt.tight_layout()
        output_file = os.path.splitext(os.path.basename(bag_path))[0] + "_motor_velocity.png"
        plt.savefig(output_file)
        print(f"Plot saved to {output_file}")
    else:
        print("No messages found on /motor/present_velocity")

if __name__ == "__main__":
    main()