#!/usr/bin/python3

import argparse
import csv
import struct
import math

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import rosbag2_py


def extract_pointcloud_to_csv(msg, csv_filename="pointcloud.csv"):
    data = msg.data
    point_step = msg.point_step
    row_step = msg.row_step
    width = msg.width
    height = msg.height
    fields = msg.fields
    
    # Find the offsets for x, y, z, and intensity
    offset_x = next(f.offset for f in fields if f.name == "x")
    offset_y = next(f.offset for f in fields if f.name == "y")
    offset_z = next(f.offset for f in fields if f.name == "z")
    offset_i = next(f.offset for f in fields if f.name == "intensity")
    
    X, Y, Z, Intensity = [], [], [], []

    for row in range(height):
        for col in range(width):
            point_offset = row * row_step + col * point_step
            x = struct.unpack_from('f', data, point_offset + offset_x)[0]
            y = struct.unpack_from('f', data, point_offset + offset_y)[0]
            z = struct.unpack_from('f', data, point_offset + offset_z)[0]
            intensity = struct.unpack_from('f', data, point_offset + offset_i)[0]

            X.append(x)
            Y.append(y)
            Z.append(z)
            Intensity.append(intensity)

    return X, Y, Z, Intensity


def write_to_csv(X, Y, Z, Intensity):
    csv_filename = "pointcloud.csv"

    with open(csv_filename, 'w', newline='') as csvfile:
        csv_writer = csv.writer(csvfile)
        csv_writer.writerow(["x", "y", "z", "intensity"])
        for x, y, z, intensity in zip(X, Y, Z, Intensity):
            csv_writer.writerow([x, y, z, intensity])



def plot_3d(X, Y, Z, Intensity):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Filter out NaN values
    filtered_data = [(x, y, z, i) for x, y, z, i in zip(X, Y, Z, Intensity) if not (math.isnan(x) or math.isnan(y) or math.isnan(z) or math.isnan(i))]
    if not filtered_data:
        print("All data points are NaN or empty.")
        return

    # Unpack filtered data
    X_f, Y_f, Z_f, Intensity_f = zip(*filtered_data)

    # Downsample data for plotting every 10th element
    scatter = ax.scatter(X_f, Y_f, Z_f, c=Intensity_f, cmap='viridis', marker='.')
    plt.colorbar(scatter, ax=ax, label='Intensity')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    # Calculate the range and midpoint for each axis
    max_range = max(max(X_f) - min(X_f), max(Y_f) - min(Y_f), max(Z_f) - min(Z_f)) / 2.0
    mid_x = (max(X_f) + min(X_f)) / 2.0
    mid_y = (max(Y_f) + min(Y_f)) / 2.0
    mid_z = (max(Z_f) + min(Z_f)) / 2.0

    # Set the limits based on the max range found
    ax.set_xlim(mid_x - max_range, mid_x + max_range)
    ax.set_ylim(mid_y - max_range, mid_y + max_range)
    ax.set_zlim(mid_z - max_range, mid_z + max_range)

    # Equalize the axes
    ax.set_box_aspect([1,1,1])  # Ensure equal aspect ratio

    plt.show()


def plot_3d_projected(X, Y, Z, Intensity):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Normalize the points onto the unit sphere
    X_p, Y_p, Z_p, Intensity_p = [], [], [], []
    for x, y, z, i in zip(X, Y, Z, Intensity):
        norm = math.sqrt(x**2 + y**2 + z**2)
        if norm != 0:  # To avoid division by zero
            X_p.append(x / norm)
            Y_p.append(y / norm)
            Z_p.append(z / norm)
            Intensity_p.append(i)  # Keep the original intensity for coloring

    # Filter out NaN values if any
    filtered_data = [(x, y, z, i) for x, y, z, i in zip(X_p, Y_p, Z_p, Intensity_p) if not (math.isnan(x) or math.isnan(y) or math.isnan(z))]
    if not filtered_data:
        print("All data points are NaN or empty after projection.")
        return

    # Unpack filtered data
    X_f, Y_f, Z_f, Intensity_f = zip(*filtered_data)

    # Plotting
    scatter = ax.scatter(X_f, Y_f, Z_f, c=Intensity_f, cmap='viridis', marker='.')
    plt.colorbar(scatter, ax=ax, label='Intensity')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('Projection on Unit Sphere')

    # Set the aspect of the plot to be equal
    ax.set_box_aspect([1, 1, 1])  # Ensure equal aspect ratio
    ax.set_xlim([-1, 1])
    ax.set_ylim([-1, 1])
    ax.set_zlim([-1, 1])

    plt.show()


def read_messages(input_bag):
    nth_pointcloud = 20

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=input_bag, storage_id="mcap"),
        rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr")
    )
    
    topic_types = reader.get_all_topics_and_types()
    count = 0
    while reader.has_next():
        topic, data, timestamp = reader.read_next()
        msg_type = get_message(next(t.type for t in topic_types if t.name == topic))
        msg = deserialize_message(data, msg_type)
        if isinstance(msg, get_message("sensor_msgs/msg/PointCloud2")):
            count += 1
            if count == nth_pointcloud:
                return msg

def main():
    parser = argparse.ArgumentParser(description="Extract and plot 10th PointCloud2 data from ROS bag")
    parser.add_argument("input_bag", help="Path to the input ROS bag file")
    args = parser.parse_args()

    msg = read_messages(args.input_bag)
    if msg:
        X, Y, Z, Intensity = extract_pointcloud_to_csv(msg)
        write_to_csv(X, Y, Z, Intensity)
        plot_3d(X[::10], Y[::10], Z[::10], Intensity)
        plot_3d_projected(X[::10], Y[::10], Z[::10], Intensity)

if __name__ == "__main__":
    main()
