import rosbag2_py
import json
from rosidl_runtime_py.utilities import get_message
from rosidl_runtime_py.convert import message_to_ordereddict
from rclpy.serialization import deserialize_message
from geometry_msgs.msg import Pose
import matplotlib.pyplot as plt
import numpy as np

def read_bag_file(bag_file_path):
    # Create a reader for the bag file
    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=bag_file_path, storage_id='mcap')
    converter_options = rosbag2_py.ConverterOptions('', '')
    reader.open(storage_options, converter_options)

    # Read all messages from topic '/vrpn_mocap/Crazyflie/pose_rpy/z'
    messages_with_time = []
    while reader.has_next():
        (topic, data, t) = reader.read_next()
        if topic == '/vrpn_mocap/Crazyflie/pose_rpy':
            # Deserialize the data into a readable format
            deserialized_data = deserialize_message(data, Pose)
            messages_with_time.append((t, deserialized_data))
    return messages_with_time

if __name__ == "__main__":
    bag_file_path = r'G:\My Drive\AAU\4 Semester\ros_2_ws\src\Crazyflie-P4-Group-460\Data_Processing\bags\Nye tests\z-test-2\z-test-1.0\2025-05-13-09-45-13_0.mcap'
    messages_with_time = read_bag_file(bag_file_path)
    
    # Extract timestamps and calculate time since the start of the bag
    start_timestamp = messages_with_time[0][0]  # First message timestamp
    relative_times = [(msg[0] - start_timestamp) / 1e9 for msg in messages_with_time]  # Time in seconds since start

    # Extract z position data
    z_positions = [msg[1].position.z for msg in messages_with_time]

    # Define the reference Z value and time interval (in seconds)
    z_ref = 1  # Use mean height as reference
    start_time = 20.0  # 10 seconds
    end_time = 80.0   # 70 seconds

    # Filter data within the time interval
    filtered_data = [(t, z) for t, z in zip(relative_times, z_positions) if start_time <= t <= end_time]
    filtered_times = [t for t, _ in filtered_data]
    filtered_positions = [z for _, z in filtered_data]

    # Calculate statistics
    max_deviation = max(abs(z - z_ref) for z in filtered_positions)
    mean_deviation = np.mean([abs(z - z_ref) for z in filtered_positions])
    std_deviation = np.std(filtered_positions)

    print(f"\nAnalysis Results:")
    print(f"Reference Z height: {z_ref:.3f} meters")
    print(f"Maximum deviation: {max_deviation:.3f} meters")
    print(f"Mean deviation: {mean_deviation:.3f} meters")
    print(f"Standard deviation: {std_deviation:.3f} meters")

    # Plot the data
    plt.figure(figsize=(12, 6))
    plt.plot(relative_times, z_positions, label='Z Position', color='blue')
    plt.axhline(y=z_ref, color='r', linestyle='--', label='Reference Height')
    plt.axvline(x=start_time, color='g', linestyle=':', label='Analysis Window')
    plt.axvline(x=end_time, color='g', linestyle=':')
    plt.title('Drone Z Position Over Time')
    plt.xlabel('Time Since Start (seconds)')
    plt.ylabel('Z Position (meters)')
    plt.grid(True)
    plt.legend()
    plt.show()