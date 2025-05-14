import rosbag2_py
from rclpy.serialization import deserialize_message
from geometry_msgs.msg import Pose
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D

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
    bag_file_path = '/home/jacob/Documents/the_ros2_ws/src/Crazyflie-P4-Group-460/Data_Processing/bags/Nye tests/xy-test-2/xy-test-1.0-1.0/2025-05-13-10-15-28_0.mcap'
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
    
    # Calculate Euclidean distance RMSE (3D error)
    rmse_3d = np.sqrt(np.mean([
        (x - x_ref)**2 + (y - y_ref)**2 + (z - z_ref)**2 
        for x, y, z in zip(filtered_x, filtered_y, filtered_z)
    ]))

    std_x = np.std(filtered_x)
    std_y = np.std(filtered_y)
    std_z = np.std(filtered_z)

    # Print analysis results
    print(f"\nAnalysis Results:")
    print(f"Reference position: X: {x_ref:.3f}, Y: {y_ref:.3f}, Z: {z_ref:.3f} meters")
    print(f"\nMaximum deviations:")
    print(f"X: {max_x_deviation:.3f} meters")
    print(f"Y: {max_y_deviation:.3f} meters")
    print(f"Z: {max_z_deviation:.3f} meters")
    print(f"\nRoot Mean Square Error:")
    print(f"X: {rmse_x:.3f} meters")
    print(f"Y: {rmse_y:.3f} meters")
    print(f"Z: {rmse_z:.3f} meters")
    print(f"3D: {rmse_3d:.3f} meters")
    print(f"\nStandard deviations:")
    print(f"X: {std_x:.3f} meters")
    print(f"Y: {std_y:.3f} meters")
    print(f"Z: {std_z:.3f} meters")

    # Create 3D scatter plot
    fig = plt.figure(figsize=(10, 10))
    ax = fig.add_subplot(111, projection='3d')

    # Plot the filtered points
    scatter = ax.scatter(filtered_x, filtered_y, filtered_z, 
                        c=filtered_times, cmap='viridis', 
                        s=20, alpha=0.6, label='Drone Position')
    
    # Plot reference point
    ax.scatter([x_ref], [y_ref], [z_ref], color='red', s=200, marker='*', label='Reference Point')

    # Create spheres at 1cm, 2cm, 3cm, 4cm, and 5cm
    # Create a sphere
    u = np.linspace(0, 2 * np.pi, 20)
    v = np.linspace(0, np.pi, 20)
    
    # Plot spheres at different radii
    radii = [0.01, 0.02, 0.03, 0.04, 0.05]  # 1cm to 5cm
    for radius in radii:
        x_sphere = x_ref + radius * np.outer(np.cos(u), np.sin(v))
        y_sphere = y_ref + radius * np.outer(np.sin(u), np.sin(v))
        z_sphere = z_ref + radius * np.outer(np.ones(np.size(u)), np.cos(v))
        
        ax.plot_surface(x_sphere, y_sphere, z_sphere, color='gray', alpha=0.1)
    
    # Add a colorbar to show the time progression
    cbar = plt.colorbar(scatter)
    cbar.set_label('Time since start (seconds)')

    # Set equal aspect ratio
    max_range = max(max_x_deviation, max_y_deviation, max_z_deviation) * 3
    ax.set_xlim(x_ref - max_range, x_ref + max_range)
    ax.set_ylim(y_ref - max_range, y_ref + max_range)
    ax.set_zlim(z_ref - max_range, z_ref + max_range)
    
    # Labels and title
    ax.set_xlabel('X Position (meters)')
    ax.set_ylabel('Y Position (meters)')
    ax.set_zlabel('Z Position (meters)')
    ax.set_title('3D Drone Position Analysis')
    ax.legend()
    
    # Equal aspect ratio
    ax.set_box_aspect([1, 1, 1])
    
    # Create a second figure for 2D projections
    fig2, (ax1, ax2, ax3) = plt.subplots(1, 3, figsize=(15, 5))
    
    # XY projection (top view)
    scatter_xy = ax1.scatter(filtered_x, filtered_y, c=filtered_times, cmap='viridis', s=20, alpha=0.6)
    ax1.scatter(x_ref, y_ref, color='red', s=100, marker='*')
    for radius in [0.01, 0.02, 0.03, 0.04, 0.05]:  # 1cm to 5cm
        circle = plt.Circle((x_ref, y_ref), radius, fill=False, linestyle='--', color='gray', alpha=0.5)
        ax1.add_artist(circle)
    ax1.set_title('XY Projection (Top View)')
    ax1.set_xlabel('X Position (meters)')
    ax1.set_ylabel('Y Position (meters)')
    ax1.grid(True)
    ax1.set_aspect('equal')
    
    # XZ projection (front view)
    scatter_xz = ax2.scatter(filtered_x, filtered_z, c=filtered_times, cmap='viridis', s=20, alpha=0.6)
    ax2.scatter(x_ref, z_ref, color='red', s=100, marker='*')
    for radius in [0.01, 0.02, 0.03, 0.04, 0.05]:  # 1cm to 5cm
        circle = plt.Circle((x_ref, z_ref), radius, fill=False, linestyle='--', color='gray', alpha=0.5)
        ax2.add_artist(circle)
    ax2.set_title('XZ Projection (Front View)')
    ax2.set_xlabel('X Position (meters)')
    ax2.set_ylabel('Z Position (meters)')
    ax2.grid(True)
    ax2.set_aspect('equal')
    
    # YZ projection (side view)
    scatter_yz = ax3.scatter(filtered_y, filtered_z, c=filtered_times, cmap='viridis', s=20, alpha=0.6)
    ax3.scatter(y_ref, z_ref, color='red', s=100, marker='*')
    for radius in [0.01, 0.02, 0.03, 0.04, 0.05]:  # 1cm to 5cm
        circle = plt.Circle((y_ref, z_ref), radius, fill=False, linestyle='--', color='gray', alpha=0.5)
        ax3.add_artist(circle)
    ax3.set_title('YZ Projection (Side View)')
    ax3.set_xlabel('Y Position (meters)')
    ax3.set_ylabel('Z Position (meters)')
    ax3.grid(True)
    ax3.set_aspect('equal')
    
    # Set limits for 2D projections
    ax1.set_xlim(x_ref - 0.05, x_ref + 0.05)
    ax1.set_ylim(y_ref - 0.05, y_ref + 0.05)
    ax2.set_xlim(x_ref - 0.05, x_ref + 0.05)
    ax2.set_ylim(z_ref - 0.05, z_ref + 0.05)
    ax3.set_xlim(y_ref - 0.05, y_ref + 0.05)
    ax3.set_ylim(z_ref - 0.05, z_ref + 0.05)
    
    # Add colorbar for the 2D plots
    plt.colorbar(scatter_xy, ax=ax1, label='Time (seconds)')
    plt.colorbar(scatter_xz, ax=ax2, label='Time (seconds)')
    plt.colorbar(scatter_yz, ax=ax3, label='Time (seconds)')
    
    plt.tight_layout()
    
    # Save results to file in the XY_test folder
    folder_path = '/home/jacob/Documents/the_ros2_ws/src/Crazyflie-P4-Group-460/Data_Processing/Results/XY_test'
    # Make a txt file with the results
    with open(f'{folder_path}/3D_analysis_({x_ref:.2f}_{y_ref:.2f}_{z_ref:.2f}).txt', 'w') as f:
        f.write(f"Reference position: X: {x_ref:.3f}, Y: {y_ref:.3f}, Z: {z_ref:.3f} meters\n")
        f.write(f"\nMaximum deviations:\n")
        f.write(f"X: {max_x_deviation:.4f} meters\n")
        f.write(f"Y: {max_y_deviation:.4f} meters\n")
        f.write(f"Z: {max_z_deviation:.4f} meters\n")
        f.write(f"\nRoot Mean Square Error:\n")
        f.write(f"X: {rmse_x:.4f} meters\n")
        f.write(f"Y: {rmse_y:.4f} meters\n")
        f.write(f"Z: {rmse_z:.4f} meters\n")
        f.write(f"3D: {rmse_3d:.4f} meters\n")
        f.write(f"\nStandard deviations:\n")
        f.write(f"X: {std_x:.4f} meters\n")
        f.write(f"Y: {std_y:.4f} meters\n")
        f.write(f"Z: {std_z:.4f} meters\n")
    
    # Save figures
    fig.savefig(f'{folder_path}/3D_analysis_({x_ref:.2f}_{y_ref:.2f}_{z_ref:.2f})_3D.png')
    fig2.savefig(f'{folder_path}/3D_analysis_({x_ref:.2f}_{y_ref:.2f}_{z_ref:.2f})_projections.png')
    
    plt.show()