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

    # Read messages from both topics
    position_messages = []
    reference_messages = []
    
    while reader.has_next():
        (topic, data, t) = reader.read_next()
        if topic == '/vrpn_mocap/Crazyflie/pose_rpy':
            deserialized_data = deserialize_message(data, Pose)
            position_messages.append((t, deserialized_data))
        elif topic == '/ref_pose':
            deserialized_data = deserialize_message(data, Pose)
            reference_messages.append((t, deserialized_data))
    
    return position_messages, reference_messages

if __name__ == "__main__":
    bag_file_path = '/home/jacob/Documents/the_ros2_ws/src/Crazyflie-P4-Group-460/Data_Processing/bags/XY-test/xy-test-3-1.0-1.0/2025-05-05-19-46-35_0.mcap'
    position_messages, reference_messages = read_bag_file(bag_file_path)
    
    # Extract timestamps and calculate time since the start of the bag
    start_timestamp = position_messages[0][0]  # First message timestamp
    relative_times = [(msg[0] - start_timestamp) / 1e9 for msg in position_messages]  # Time in seconds since start

    # Extract position data
    x_positions = [msg[1].position.x for msg in position_messages]
    y_positions = [msg[1].position.y for msg in position_messages]
    z_positions = [msg[1].position.z for msg in position_messages]

    # Get reference positions from the last reference message
    if reference_messages:
        ref_pose = reference_messages[-1][1]
        x_ref = ref_pose.position.x
        y_ref = ref_pose.position.y
        z_ref = ref_pose.position.z
    else:
        print("Warning: No reference messages found. Using (0,0,1) as default reference.")
        x_ref = 0
        y_ref = 0
        z_ref = 1

    # Define time interval (in seconds)
    start_time = 20.0
    end_time = 80.0

    # Filter data within the time interval
    filtered_data = [(t, x, y, z) for t, x, y, z in zip(relative_times, x_positions, y_positions, z_positions) 
                    if start_time <= t <= end_time]
    filtered_times = [t for t, _, _, _ in filtered_data]
    filtered_x = [x for _, x, _, _ in filtered_data]
    filtered_y = [y for _, _, y, _ in filtered_data]
    filtered_z = [z for _, _, _, z in filtered_data]

    # Calculate statistics for each axis
    max_x_deviation = max(abs(x - x_ref) for x in filtered_x)
    max_y_deviation = max(abs(y - y_ref) for y in filtered_y)
    max_z_deviation = max(abs(z - z_ref) for z in filtered_z)

    mean_x_deviation = np.mean([abs(x - x_ref) for x in filtered_x])
    mean_y_deviation = np.mean([abs(y - y_ref) for y in filtered_y])
    mean_z_deviation = np.mean([abs(z - z_ref) for z in filtered_z])

    std_x_deviation = np.std(filtered_x)
    std_y_deviation = np.std(filtered_y)
    std_z_deviation = np.std(filtered_z)

    print(f"\nAnalysis Results:")
    print(f"Reference positions from /ref_pose - X: {x_ref:.3f}, Y: {y_ref:.3f}, Z: {z_ref:.3f} meters")
    print(f"\nMaximum deviations:")
    print(f"X: {max_x_deviation:.3f} meters")
    print(f"Y: {max_y_deviation:.3f} meters")
    print(f"Z: {max_z_deviation:.3f} meters")
    print(f"\nMean deviations:")
    print(f"X: {mean_x_deviation:.3f} meters")
    print(f"Y: {mean_y_deviation:.3f} meters")
    print(f"Z: {mean_z_deviation:.3f} meters")
    print(f"\nStandard deviations:")
    print(f"X: {std_x_deviation:.3f} meters")
    print(f"Y: {std_y_deviation:.3f} meters")
    print(f"Z: {std_z_deviation:.3f} meters")

    # Create subplots for X, Y, and Z positions
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(12, 12))

    # Plot X position with ±15cm range around reference
    ax1.plot(relative_times, x_positions, label='X Position', color='red')
    ax1.axhline(y=x_ref, color='gray', linestyle='--', label=f'Reference X ({x_ref:.3f}m)')
    ax1.axvline(x=start_time, color='g', linestyle=':', label='Analysis Window')
    ax1.axvline(x=end_time, color='g', linestyle=':')
    ax1.set_title('Drone X Position Over Time')
    ax1.set_ylabel('X Position (meters)')
    ax1.set_ylim(x_ref - 0.15, x_ref + 0.15)  # ±15cm around reference
    ax1.grid(True)
    ax1.legend()

    # Plot Y position with ±15cm range around reference
    ax2.plot(relative_times, y_positions, label='Y Position', color='green')
    ax2.axhline(y=y_ref, color='gray', linestyle='--', label=f'Reference Y ({y_ref:.3f}m)')
    ax2.axvline(x=start_time, color='g', linestyle=':', label='Analysis Window')
    ax2.axvline(x=end_time, color='g', linestyle=':')
    ax2.set_title('Drone Y Position Over Time')
    ax2.set_ylabel('Y Position (meters)')
    ax2.set_ylim(y_ref - 0.15, y_ref + 0.15)  # ±15cm around reference
    ax2.grid(True)
    ax2.legend()

    # Plot Z position (keeping original scale)
    ax3.plot(relative_times, z_positions, label='Z Position', color='blue')
    ax3.axhline(y=z_ref, color='gray', linestyle='--', label=f'Reference Z ({z_ref:.3f}m)')
    ax3.axvline(x=start_time, color='g', linestyle=':', label='Analysis Window')
    ax3.axvline(x=end_time, color='g', linestyle=':')
    ax3.set_title('Drone Z Position Over Time')
    ax3.set_xlabel('Time Since Start (seconds)')
    ax3.set_ylabel('Z Position (meters)')
    ax3.grid(True)
    ax3.legend()

    plt.tight_layout()
    plt.show()