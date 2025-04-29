#!/usr/bin/env python3
import os
from rosbag2_py import SequentialReader, SequentialWriter, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message, serialize_message
from geometry_msgs.msg import Vector3Stamped

# file_name = 'Experiment1_flow_960_0.db3'
# paths
INPUT_BAG = "/home/docker/OpticalFlowSlider/ros2_ws/my_rosbag/Experiment1/Experiment1_Stationary/Experiment1_raft_l_stationary_960/Experiment1_raft_l_stationary_960_0.db3" 
OUTPUT_DIR = "/home/docker/OpticalFlowSlider/ros2_ws/my_rosbag/Experiment1/Experiment1_Stationary_Shifted/"
OUTPUT_BAG= os.path.join(OUTPUT_DIR, "shifted_" + os.path.basename(INPUT_BAG))
# INPUT_BAG = '/home/docker/OpticalFlowSlider/ros2_ws/my_rosbag/Experiment1/Experiment1_flow/Experiment1_lfn3_960/' + file_name
# OUTPUT_BAG = '/home/docker/OpticalFlowSlider/ros2_ws/my_rosbag/Experiment1/Experiment1_flow_shifted/' + 'shifted_' + file_name 
SHIFT_NS = 590_000_000   # 200 ms in nanoseconds

# open reader
reader = SequentialReader()
reader.open(
    StorageOptions(uri=INPUT_BAG, storage_id='sqlite3'),
    ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr')
)

# open writer
writer = SequentialWriter()
writer.open(
    StorageOptions(uri=OUTPUT_BAG, storage_id='sqlite3'),
    ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr')
)

# copy topic definitions exactly
for topic in reader.get_all_topics_and_types():
    writer.create_topic(topic)

# process messages
while reader.has_next():
    (topic, data, t) = reader.read_next()
    # if it's one of your two optical‑flow topics, shift both the
    # bag timestamp _and_ the header.stamp inside the message
    if topic in ['/optical_flow/raft_velocity',
                 '/optical_flow/raft_smooth_velocity']:
        # shift the recorded timestamp
        t_shifted = t - SHIFT_NS

        # deserialize, adjust header, reserialize
        msg = deserialize_message(data, Vector3Stamped)
        # header.stamp is in seconds & nanoseconds
        sec = msg.header.stamp.sec
        nanosec = msg.header.stamp.nanosec - SHIFT_NS
        # borrow Python divmod to handle underflow
        sec_delta, nanosec = divmod(nanosec, 1_000_000_000)
        msg.header.stamp.sec = sec + sec_delta
        msg.header.stamp.nanosec = nanosec

        data = serialize_message(msg)

        writer.write(topic, data, t_shifted)

    else:
        # leave everything else untouched
        writer.write(topic, data, t)

# done
reader.close()
writer.close()
print("Wrote shifted bag to", OUTPUT_BAG)
