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
    bag_file_path = '/home/jacob/Documents/the_ros2_ws/src/Crazyflie-P4-Group-460/Data_Processing/bags/Nye tests/xy-test-2/xy-test--0.5--0.5/2025-05-13-10-24-29_0.mcap'
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

    rmse_x = np.sqrt(np.mean([(x - x_ref)**2 for x in filtered_x]))
    rmse_y = np.sqrt(np.mean([(y - y_ref)**2 for y in filtered_y]))
    rmse_z = np.sqrt(np.mean([(z - z_ref)**2 for z in filtered_z]))

    std_x_deviation = np.std(filtered_x)
    std_y_deviation = np.std(filtered_y)
    std_z_deviation = np.std(filtered_z)

    print(f"\nAnalysis Results:")
    print(f"Reference positions from /ref_pose - X: {x_ref:.3f}, Y: {y_ref:.3f}, Z: {z_ref:.3f} meters")
    print(f"\nMaximum deviations:")
    print(f"X: {max_x_deviation:.3f} meters")
    print(f"Y: {max_y_deviation:.3f} meters")
    print(f"Z: {max_z_deviation:.3f} meters")
    print(f"\nRoot Mean Square Error:")
    print(f"X: {rmse_x:.3f} meters")
    print(f"Y: {rmse_y:.3f} meters")
    print(f"Z: {rmse_z:.3f} meters")
    print(f"\nStandard deviations:")
    print(f"X: {std_x_deviation:.3f} meters")
    print(f"Y: {std_y_deviation:.3f} meters")
    print(f"Z: {std_z_deviation:.3f} meters")

    # Create first figure for X, Y, Z position plots
    fig1 = plt.figure(figsize=(10, 12))
    gs = plt.GridSpec(3, 1, height_ratios=[1, 1, 1])
    
    ax1 = plt.subplot(gs[0])
    ax2 = plt.subplot(gs[1])
    ax3 = plt.subplot(gs[2])

    # Plot X position with ±15cm range around reference
    ax1.plot(relative_times, x_positions, label='X Position', color='red')
    ax1.axhline(y=x_ref, color='gray', linestyle='--', label=f'Reference X ({x_ref:.3f}m)')
    ax1.axvline(x=start_time, color='g', linestyle=':', label='Analysis Window')
    ax1.axvline(x=end_time, color='g', linestyle=':')
    ax1.set_title('Drone X Position Over Time')
    ax1.set_ylabel('X Position (meters)')
    ax1.set_ylim(x_ref - 0.15, x_ref + 0.15)
    ax1.grid(True)
    ax1.legend()

    # Plot Y position with ±15cm range around reference
    ax2.plot(relative_times, y_positions, label='Y Position', color='green')
    ax2.axhline(y=y_ref, color='gray', linestyle='--', label=f'Reference Y ({y_ref:.3f}m)')
    ax2.axvline(x=start_time, color='g', linestyle=':', label='Analysis Window')
    ax2.axvline(x=end_time, color='g', linestyle=':')
    ax2.set_title('Drone Y Position Over Time')
    ax2.set_ylabel('Y Position (meters)')
    ax2.set_ylim(y_ref - 0.15, y_ref + 0.15)
    ax2.grid(True)
    ax2.legend()

    # Plot Z position
    ax3.plot(relative_times, z_positions, label='Z Position', color='blue')
    ax3.axhline(y=z_ref, color='gray', linestyle='--', label=f'Reference Z ({z_ref:.3f}m)')
    ax3.axvline(x=start_time, color='g', linestyle=':', label='Analysis Window')
    ax3.axvline(x=end_time, color='g', linestyle=':')
    ax3.set_title('Drone Z Position Over Time')
    ax3.set_xlabel('Time Since Start (seconds)')
    ax3.set_ylabel('Z Position (meters)')
    ax3.grid(True)
    ax3.legend()

    plt.tight_layout(pad=2.0)

    # Create second figure for XY scatter plot
    fig2 = plt.figure(figsize=(8, 8))
    ax4 = plt.subplot(111)

    # Create XY scatter plot without time coloring
    filtered_xy = [(x, y) for t, x, y, z in filtered_data]
    filtered_x_scatter, filtered_y_scatter = zip(*filtered_xy)
    
    ax4.scatter(filtered_x_scatter, filtered_y_scatter, 
                color='blue', label='Drone Position', s=5, alpha=0.5)
    ax4.scatter(x_ref, y_ref, color='red', s=100, marker='*', label='Reference Point')

    # Add circles at 1cm intervals up to 5cm
    circles = []
    for radius in [0.01, 0.02, 0.03, 0.04, 0.05]:  # 1cm to 5cm
        circle = plt.Circle((x_ref, y_ref), radius, fill=False, linestyle='--', 
                          color='gray', alpha=0.5)
        ax4.add_artist(circle)
        circles.append(circle)

    # Add labels for the circles
    for i, circle in enumerate(circles):
        radius = (i + 1) * 1  # 1cm increments
        ax4.text(x_ref + circle.radius, y_ref, f'{radius}cm', 
                verticalalignment='bottom', horizontalalignment='left')

    ax4.set_title('Drone XY Position')
    ax4.set_xlabel('X Position (meters)')
    ax4.set_ylabel('Y Position (meters)')
    ax4.grid(True)
    ax4.legend()
    
    # Make the XY plot have equal aspect ratio
    ax4.set_aspect('equal')
    
    # Set limits to show the same range as individual plots (±15cm)
    ax4.set_xlim(x_ref - 0.05, x_ref + 0.05)
    ax4.set_ylim(y_ref - 0.05, y_ref + 0.05)

    plt.tight_layout()
    plt.show()