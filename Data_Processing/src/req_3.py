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
    bag_file_path = '/home/jacob/Documents/the_ros2_ws/src/Crazyflie-P4-Group-460/Data_Processing/bags/Nye tests/Waypoint-test-2/waypoint-test-1.0-1.0-1.5/2025-05-13-12-42-49_0.mcap'
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
    start_time = 30.0
    end_time = 60.0

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
    
    mean_distance_error = np.mean([
        np.sqrt((x - x_ref)**2 + (y - y_ref)**2 + (z - z_ref)**2)
        for x, y, z in zip(filtered_x, filtered_y, filtered_z)
    ])
    
    max_distance_error = max([
        np.sqrt((x - x_ref)**2 + (y - y_ref)**2 + (z - z_ref)**2)
        for x, y, z in zip(filtered_x, filtered_y, filtered_z)
    ])
    
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
    
    print(f"\nX results:")
    print(f"Maximum deviation: {max_x_deviation * 1000:.3f} mm")
    print(f"Mean deviation: {(np.mean(filtered_x) - x_ref) * 1000:.3f} mm")
    print(f"Root Mean Square Error: {std_x * 1000:.3f} mm")
    
    print(f"\nY results:")
    print(f"Maximum deviation: {max_y_deviation * 1000:.3f} mm")
    print(f"Mean deviation: {(np.mean(filtered_y) - y_ref) * 1000:.3f} mm")
    print(f"Root Mean Square Error: {std_y * 1000:.3f} mm")
    
    print(f"\nZ results:")
    print(f"Maximum deviation: {max_z_deviation * 1000:.3f} mm")
    print(f"Mean deviation: {(np.mean(filtered_z) - z_ref) * 1000:.3f} mm")
    print(f"Root Mean Square Error: {std_z * 1000:.3f} mm")
    
    print(f"\n3D Results:")
    print(f"Maximum distance error: {max_distance_error * 1000:.3f} mm")
    print(f"Mean distance error: {mean_distance_error * 1000:.3f} mm")
    print(f"Root Mean Square Error: {rmse_3d * 1000:.3f} mm")
    
    
    # Create 3D scatter plot    
    fig = plt.figure(figsize=(10, 10))
    ax = fig.add_subplot(111, projection='3d')

    # Create spheres at 1cm, 2cm, 3cm, 4cm, and 5cm first (bottom layer)
    u = np.linspace(0, 2 * np.pi, 20)
    v = np.linspace(0, np.pi, 20)
    
    # Plot spheres at different radii with labels for legend
    #radii = [0.01, 0.02, 0.03, 0.04, 0.05]  # 1cm to 5cm
    radii = [0.01, 0.02]
    sphere_surfaces = []  # Store surface objects for legend

    for radius in radii:
        x_sphere = x_ref + radius * np.outer(np.cos(u), np.sin(v))
        y_sphere = y_ref + radius * np.outer(np.sin(u), np.sin(v))
        z_sphere = z_ref + radius * np.outer(np.ones(np.size(u)), np.cos(v))
        
        # Plot the sphere surface
        surf = ax.plot_surface(x_sphere, y_sphere, z_sphere, color='gray', alpha=0.1)
        
        # Add a text label next to the sphere
        ax.text(x_ref + radius, y_ref, z_ref, f"{int(radius*100)} cm", 
                color='black', fontsize=8)
        
        # Create a proxy artist for the legend
        sphere_proxy = plt.Line2D([0], [0], linestyle="none", c='gray', 
                                marker='o', alpha=0.3, markersize=5+radius*200,
                                label=f'{int(radius*100)} cm radius')
        sphere_surfaces.append(sphere_proxy)
    
    # Plot the filtered points with solid color next (middle layer)
    scatter = ax.scatter(filtered_x, filtered_y, filtered_z, 
                        color='blue', # Using solid blue color instead of gradient
                        s=5, alpha=0.6, label='Drone Position')
    
    # Plot reference point last (top layer)
    ref_point = ax.scatter([x_ref], [y_ref], [z_ref], color='red', s=200, marker='*', label='Reference Point', zorder=10)

    # Set fixed axis limits to ±5 cm from reference point
    zoom = 0.02
    ax.set_xlim(x_ref - zoom, x_ref + zoom)
    ax.set_ylim(y_ref - zoom, y_ref + zoom)
    ax.set_zlim(z_ref - zoom, z_ref + zoom)
    
    # Labels and title
    ax.set_xlabel('X Position (meters)')
    ax.set_ylabel('Y Position (meters)')
    ax.set_zlabel('Z Position (meters)')
    ax.set_title('3D Drone Position Analysis')
    
    # Add all sphere proxy artists to the legend
    handles, labels = ax.get_legend_handles_labels()
    handles.extend(sphere_surfaces)
    ax.legend(handles=handles)
    
    # Equal aspect ratio
    ax.set_box_aspect([1, 1, 1])
        
    # Save results to file in the XY_test folder
    folder_path = '/home/jacob/Documents/the_ros2_ws/src/Crazyflie-P4-Group-460/Data_Processing/Results/waypoint_test'
    # Make a txt file with the results
    with open(f'{folder_path}/waypoint_test_({x_ref:.2f}_{y_ref:.2f}_{z_ref:.2f}).txt', 'w') as f:
        f.write(f"Reference position: X: {x_ref:.3f}, Y: {y_ref:.3f}, Z: {z_ref:.3f} meters\n")

        f.write(f"X results:\n")
        f.write(f"Maximum deviation: {max_x_deviation * 1000:.3f} mm\n")
        f.write(f"Mean deviation: {(np.mean(filtered_x) - x_ref) * 1000:.3f} mm\n")
        f.write(f"Root Mean Square Error: {std_x * 1000:.3f} mm\n")
        f.write(f"\nY results:\n")
        f.write(f"Maximum deviation: {max_y_deviation * 1000:.3f} mm\n")
        f.write(f"Mean deviation: {(np.mean(filtered_y) - y_ref) * 1000:.3f} mm\n")
        f.write(f"Root Mean Square Error: {std_y * 1000:.3f} mm\n")
        f.write(f"\nZ results:\n")
        f.write(f"Maximum deviation: {max_z_deviation * 1000:.3f} mm\n")
        f.write(f"Mean deviation: {(np.mean(filtered_z) - z_ref) * 1000:.3f} mm\n")
        f.write(f"Root Mean Square Error: {std_z * 1000:.3f} mm\n")
        f.write(f"\n3D Results:\n")
        f.write(f"Maximum distance error: {max_distance_error * 1000:.3f} mm\n")
        f.write(f"Mean distance error: {mean_distance_error * 1000:.3f} mm\n")
        f.write(f"Root Mean Square Error: {rmse_3d * 1000:.3f} mm\n")

    
    # Save figures
    fig.savefig(f'{folder_path}/waypoint_test_({x_ref:.2f}_{y_ref:.2f}_{z_ref:.2f})_3D.png')
        
    plt.show()