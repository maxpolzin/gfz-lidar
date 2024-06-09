#!/usr/bin/python3

import argparse
import csv
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import rosbag2_py

def extract_packets_to_csv(input_bag, csv_filename="rslidar_packets.csv"):

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=input_bag, storage_id="mcap"),
        rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr")
    )
    
    topic_types = reader.get_all_topics_and_types()

    csv_filename = "rslidar_packets.csv"

    with open(csv_filename, 'w', newline='') as csvfile:
        csv_writer = None
        
        while reader.has_next():
            topic, data, timestamp = reader.read_next()
            msg_type = get_message(next(t.type for t in topic_types if t.name == topic))
            msg = deserialize_message(data, msg_type)
            if isinstance(msg, get_message("rslidar_msg/msg/RslidarPacket")):

                if not csv_writer:
                    headers = ['header.stamp.sec', 'header.stamp.nanosec', 'is_difop', 'is_frame_begin'] + [f'data[{i}]' for i in range(len(msg.data))]
                    csv_writer = csv.writer(csvfile)
                    csv_writer.writerow(headers)

                if msg.is_difop:
                    print(f"Found DIFOP packet at {msg.header.stamp.sec}.{msg.header.stamp.nanosec}")
                if msg.is_frame_begin:
                    print(f"Found frame begin packet at {msg.header.stamp.sec}.{msg.header.stamp.nanosec}")

                row = [msg.header.stamp.sec, msg.header.stamp.nanosec, msg.is_difop, msg.is_frame_begin] + list(msg.data)
                csv_writer.writerow(row)



def main():
    parser = argparse.ArgumentParser(description="Extract RslidarPacket messages from ROS bag to CSV")
    parser.add_argument("input_bag", help="Path to the input ROS bag file")
    args = parser.parse_args()

    extract_packets_to_csv(args.input_bag)

if __name__ == "__main__":
    main()
