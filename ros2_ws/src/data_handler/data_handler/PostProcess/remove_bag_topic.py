#!/usr/bin/env python3
"""
filter_ros2_bag.py

Reads a ROS 2 SQLite3 bag, removes all topics whose name begins with /kalman_filter,
and writes out a new bag in the specified output directory.

Simply run:
    python3 filter_ros2_bag.py

To change bags or output location, edit the constants below.
"""

import os
from rosbag2_py import (
    SequentialReader,
    SequentialWriter,
    StorageOptions,
    ConverterOptions,
    TopicMetadata,
)

# === EDIT THESE ===
INPUT_BAG     = '/home/docker/OpticalFlowSlider/ros2_ws/my_rosbag/Experiment1/flow/lfn3_960/Experiment1_flow_960_0.db3'      # your source bag file
OUTPUT_DIR    = '/home/docker/OpticalFlowSlider/ros2_ws/my_rosbag/Experiment1/flow/lfn3_960'                # directory for the new bag
EXCLUDE_PREFIX = '/kalman_filter'                 # topic‐name prefix to remove
# ==================

def filter_bag(input_bag: str, output_dir: str, exclude_prefix: str):
    # ensure output directory exists
    os.makedirs(output_dir, exist_ok=True)

    # derive output filename
    base = os.path.splitext(os.path.basename(input_bag))[0]
    out_filename = f"{base}_noKF.db3"
    output_bag = os.path.join(output_dir, out_filename)

    # 1) Open the source bag
    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=input_bag, storage_id='sqlite3'),
        ConverterOptions(
            input_serialization_format='cdr',
            output_serialization_format='cdr'
        )
    )

    # 2) Build list of topics to keep
    all_topics = reader.get_all_topics_and_types()
    keep_topics = [t.name for t in all_topics if not t.name.startswith(exclude_prefix)]

    # 3) Open the destination bag
    writer = SequentialWriter()
    writer.open(
        StorageOptions(uri=output_bag, storage_id='sqlite3'),
        ConverterOptions(
            input_serialization_format='cdr',
            output_serialization_format='cdr'
        )
    )
    for topic_meta in all_topics:
        if topic_meta.name in keep_topics:
            writer.create_topic(TopicMetadata(
                name=topic_meta.name,
                type=topic_meta.type,
                serialization_format='cdr'
            ))

    # 4) Copy messages
    while reader.has_next():
        topic_name, data, timestamp = reader.read_next()
        if topic_name in keep_topics:
            writer.write(topic_name, data, timestamp)

    print(f"[DONE] Filtered bag saved to: {output_bag}")

if __name__ == '__main__':
    filter_bag(INPUT_BAG, OUTPUT_DIR, EXCLUDE_PREFIX)
