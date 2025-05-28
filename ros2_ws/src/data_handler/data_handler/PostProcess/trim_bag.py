import rosbag2_py
import argparse

def find_time_range(input_bag_path, topic_name):
    """
    Find the earliest and latest timestamps for the specified topic in the bag file.
    
    Args:
        input_bag_path (str): Path to the input bag file.
        topic_name (str): Topic to analyze (e.g., '/motor/present_velocity').
    
    Returns:
        tuple: (t_start, t_end) in nanoseconds.
    
    Raises:
        ValueError: If no messages are found for the specified topic.
    """
    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=input_bag_path, storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions('', '')
    reader.open(storage_options, converter_options)
    
    t_start = None
    t_end = None
    
    while reader.has_next():
        topic, data, t = reader.read_next()
        if topic == topic_name:
            if t_start is None or t < t_start:
                t_start = t
            if t_end is None or t > t_end:
                t_end = t
    
    if t_start is None or t_end is None:
        raise ValueError(f"No messages found for topic {topic_name}")
    
    return t_start, t_end

def filter_bag(input_bag_path, output_bag_path, t_start, t_end):
    """
    Create a new bag file with messages filtered to the time range [t_start, t_end].
    
    Args:
        input_bag_path (str): Path to the input bag file.
        output_bag_path (str): Path to save the filtered bag file.
        t_start (int): Start timestamp (inclusive) in nanoseconds.
        t_end (int): End timestamp (inclusive) in nanoseconds.
    """
    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=input_bag_path, storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions('', '')
    reader.open(storage_options, converter_options)
    
    writer = rosbag2_py.SequentialWriter()
    writer_storage_options = rosbag2_py.StorageOptions(uri=output_bag_path, storage_id='sqlite3')
    writer_converter_options = rosbag2_py.ConverterOptions('', '')
    writer.open(writer_storage_options, writer_converter_options)
    
    # Register all topics from the input bag in the writer
    topics = reader.get_all_topics_and_types()
    for topic in topics:
        writer.create_topic(topic)
    
    # Filter and write messages within the time range
    while reader.has_next():
        topic, data, t = reader.read_next()
        if t_start <= t <= t_end:
            writer.write(topic, data, t)

def main():
    # Parse command-line arguments
    parser = argparse.ArgumentParser(description='Trim ROS 2 bag file based on /motor/present_velocity timestamps.')
    parser.add_argument('input_bag', type=str, help='Path to the input bag file')
    parser.add_argument('output_bag', type=str, help='Path to the output bag file')
    args = parser.parse_args()
    
    topic_name = '/motor/present_velocity'
    
    # Find the time range for /motor/present_velocity
    t_start, t_end = find_time_range(args.input_bag, topic_name)
    
    # Filter the bag file based on the time range
    filter_bag(args.input_bag, args.output_bag, t_start, t_end)
    
    print(f"Filtered bag saved to {args.output_bag}")

if __name__ == '__main__':
    main()