import rosbag2_py
from rclpy.serialization import deserialize_message
from geometry_msgs.msg import Pose
import matplotlib.pyplot as plt

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
    bag_file_path = r'G:\My Drive\AAU\4 Semester\ros_2_ws\src\Crazyflie-P4-Group-460\Data_Processing\bags\bane-test\bane-test-2-trapz\2025-05-05-21-30-22_0.mcap'
    position_messages, reference_messages = read_bag_file(bag_file_path)

    # Extract drone position data
    x_positions = [msg[1].position.x for msg in position_messages]
    y_positions = [msg[1].position.y for msg in position_messages]
    z_positions = [msg[1].position.z for msg in position_messages]

    # Extract waypoints from reference messages
    ref_x_positions = [msg[1].position.x for msg in reference_messages]
    ref_y_positions = [msg[1].position.y for msg in reference_messages]
    ref_z_positions = [msg[1].position.z for msg in reference_messages]

    # Create figure for 3D trajectory plot
    fig = plt.figure(figsize=(10, 10))
    ax = fig.add_subplot(111, projection='3d')

    # Plot the drone's trajectory
    ax.plot3D(x_positions, y_positions, z_positions, 'blue', label='Drone Trajectory')
    
    # Plot waypoints and connecting lines in 3D
    ax.plot3D(ref_x_positions, ref_y_positions, ref_z_positions, 'r--', label='Waypoint Path')
    ax.scatter(ref_x_positions, ref_y_positions, ref_z_positions, color='red', s=100, marker='*', label='Waypoints')

    # Set equal aspect ratio for all axes
    ax.set_box_aspect([1, 1, 1])

    # Calculate the maximum range needed across all dimensions
    margin = 0.15  # 15cm margin
    x_min, x_max = min(ref_x_positions) - margin, max(ref_x_positions) + margin
    y_min, y_max = min(ref_y_positions) - margin, max(ref_y_positions) + margin
    z_min, z_max = min(ref_z_positions) - margin, max(ref_z_positions) + margin
    
    # Find the maximum range needed
    x_range = x_max - x_min
    y_range = y_max - y_min
    z_range = z_max - z_min
    max_range = max(x_range, y_range, z_range)
    
    # Calculate the center of each dimension
    x_center = (x_max + x_min) / 2
    y_center = (y_max + y_min) / 2
    z_center = (z_max + z_min) / 2
    
    # Set equal limits centered around the middle of each dimension
    ax.set_xlim(x_center - max_range/2, x_center + max_range/2)
    ax.set_ylim(y_center - max_range/2, y_center + max_range/2)
    ax.set_zlim(z_center - max_range/2, z_center + max_range/2)

    # Add labels and title
    ax.set_xlabel('X Position (meters)')
    ax.set_ylabel('Y Position (meters)')
    ax.set_zlabel('Z Position (meters)')
    ax.set_title('Drone 3D Trajectory and Waypoints')
    ax.legend()

    # Add grid
    ax.grid(True)

    plt.tight_layout()
    plt.show()