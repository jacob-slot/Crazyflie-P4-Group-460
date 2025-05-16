import rosbag2_py
from rclpy.serialization import deserialize_message
from geometry_msgs.msg import Pose
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import os
import math
from matplotlib import cm
from matplotlib.colors import Normalize


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


def calculate_distance_3d(p1, p2):
    """Calculate Euclidean distance between two 3D points"""
    return np.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2 + (p2[2] - p1[2])**2)


def detect_waypoint_reached(position_data, waypoint, threshold=0.15):
    """
    Detect when the drone reaches a waypoint
    Args:
        position_data: List of (timestamp, x, y, z) tuples
        waypoint: (x, y, z) tuple representing the waypoint
        threshold: Distance threshold in meters to consider waypoint reached
        
    Returns:
        Index of the first position where waypoint is reached, or -1 if not reached
    """
    for i, (t, x, y, z) in enumerate(position_data):
        distance = calculate_distance_3d((x, y, z), waypoint)
        if distance < threshold:
            return i
    return -1


def calculate_path_length(positions):
    """
    Calculate the total length of a path
    Args:
        positions: List of (x, y, z) tuples representing positions
        
    Returns:
        Total path length in meters
    """
    path_length = 0
    for i in range(1, len(positions)):
        path_length += calculate_distance_3d(positions[i-1], positions[i])
    return path_length


def calculate_speed_stats(positions_with_time):
    """
    Calculate instantaneous speed at each point
    Args:
        positions_with_time: List of (timestamp, x, y, z) tuples
        
    Returns:
        List of (timestamp, speed) tuples
    """
    speeds = []
    
    # Calculate speeds with a window size to reduce noise
    window_size = 5  # Use 5 frames to calculate speed
    
    for i in range(window_size, len(positions_with_time)):
        # Use points that are window_size apart
        t_prev, x_prev, y_prev, z_prev = positions_with_time[i-window_size]
        t_curr, x_curr, y_curr, z_curr = positions_with_time[i]
        
        # Calculate distance traveled and time elapsed
        distance = calculate_distance_3d((x_prev, y_prev, z_prev), (x_curr, y_curr, z_curr))
        time_elapsed = (t_curr - t_prev)  # already in seconds
        
        # Calculate speed (handle zero time case)
        if time_elapsed > 0.001:  # Avoid division by very small numbers
            speed = distance / time_elapsed
            # Filter out any unrealistic speeds (drones can't go faster than ~10 m/s)
            if speed < 10:
                speeds.append((t_curr, speed))
            else:
                speeds.append((t_curr, 0))  # Zero out extreme values
        else:
            speeds.append((t_curr, 0))
            
    return speeds


if __name__ == "__main__":
    # Path to bag file - use the specified file
    bag_file_path = '/home/jacob/Documents/the_ros2_ws/src/Crazyflie-P4-Group-460/Data_Processing/bags/Nye tests/path-test-2/pathe-test-rektangel/2025-05-13-11-28-39_0.mcap'
    
    # Check if file exists
    if not os.path.exists(bag_file_path):
        print(f"Error: Specified bag file not found: {bag_file_path}")
        exit(1)
    
    print(f"Analyzing bag file: {bag_file_path}")
    position_messages, reference_messages = read_bag_file(bag_file_path)
    
    if not position_messages:
        print("Error: No position messages found in the bag file")
        exit(1)
        
    if not reference_messages:
        print("Warning: No reference messages (waypoints) found in the bag file")
    
    # Extract timestamps and calculate time since the start of the bag
    start_timestamp = position_messages[0][0]  # First message timestamp
    
    # Process drone position data
    position_data = []  # (timestamp, x, y, z)
    for msg in position_messages:
        timestamp = msg[0]
        relative_time = (timestamp - start_timestamp) / 1e9  # Convert to seconds
        x = msg[1].position.x
        y = msg[1].position.y
        z = msg[1].position.z
        position_data.append((relative_time, x, y, z))
    
    # Process waypoint data
    waypoints = []  # (x, y, z)
    for msg in reference_messages:
        x = msg[1].position.x
        y = msg[1].position.y
        z = msg[1].position.z
        # Only add unique waypoints
        point = (x, y, z)
        if point not in waypoints:
            waypoints.append(point)
    
    print(f"Found {len(waypoints)} unique waypoints in the path:")
    for i, wp in enumerate(waypoints):
        print(f"  Waypoint {i+1}: ({wp[0]:.2f}, {wp[1]:.2f}, {wp[2]:.2f})")
    
    # Detect when the drone reaches the first waypoint
    if waypoints:
        first_waypoint_index = detect_waypoint_reached(
            [(t, x, y, z) for t, x, y, z in position_data], 
            waypoints[0]
        )
        
        if first_waypoint_index >= 0:
            print(f"Drone reached first waypoint at t = {position_data[first_waypoint_index][0]:.2f} seconds")
            start_time = position_data[first_waypoint_index][0]
            
            # Detect when the drone reaches the final waypoint
            last_waypoint_index = detect_waypoint_reached(
                [(t, x, y, z) for t, x, y, z in position_data],
                waypoints[-1]
            )
            
            if last_waypoint_index >= 0:
                print(f"Drone reached final waypoint at t = {position_data[last_waypoint_index][0]:.2f} seconds")
                end_time = position_data[last_waypoint_index][0]
                
                # Calculate flight time
                flight_time = end_time - start_time
                print(f"Total flight time: {flight_time:.2f} seconds")
                
                # Calculate path length for the actual drone path
                path_positions = [(x, y, z) for _, x, y, z in position_data[first_waypoint_index:last_waypoint_index+1]]
                actual_path_length = calculate_path_length(path_positions)
                print(f"Actual path length: {actual_path_length:.2f} meters")
                
                # Calculate ideal path length (straight lines between waypoints)
                ideal_path_length = calculate_path_length(waypoints)
                print(f"Ideal path length: {ideal_path_length:.2f} meters")
                
                # Calculate average speed using ideal path length
                avg_speed_ideal = ideal_path_length / flight_time
                print(f"Average speed: {avg_speed_ideal:.2f} m/s ({avg_speed_ideal * 3.6:.2f} km/h)")
                
                # Calculate average speed using actual path length (for reference)
                avg_speed_actual = actual_path_length / flight_time
                #print(f"Average speed (actual path): {avg_speed_actual:.2f} m/s ({avg_speed_actual * 3.6:.2f} km/h)")
                
                # Calculate speed at each position
                flight_positions = position_data[first_waypoint_index:last_waypoint_index+1]
                speeds_for_path = []
                
                # Calculate instantaneous speeds for coloring the path
                window_size = 5  # Use 5 frames to calculate speed
                for i in range(window_size, len(flight_positions)):
                    # Use points that are window_size apart
                    t_prev, x_prev, y_prev, z_prev = flight_positions[i-window_size]
                    t_curr, x_curr, y_curr, z_curr = flight_positions[i]
                    
                    # Calculate distance traveled and time elapsed
                    distance = calculate_distance_3d((x_prev, y_prev, z_prev), (x_curr, y_curr, z_curr))
                    time_elapsed = (t_curr - t_prev)  # already in seconds
                    
                    # Calculate speed (handle zero time case)
                    if time_elapsed > 0.001:  # Avoid division by very small numbers
                        speed = distance / time_elapsed
                        # Filter out any unrealistic speeds
                        if speed < 10:
                            speeds_for_path.append(speed)
                        else:
                            speeds_for_path.append(0)
                    else:
                        speeds_for_path.append(0)
                
                # Need to add speed values for the first window_size points
                # Use the first calculated speed for these points
                first_speed = speeds_for_path[0] if speeds_for_path else 0
                speeds_for_path = [first_speed] * window_size + speeds_for_path
                
                # Calculate instantaneous speeds (for analysis)
                speeds = calculate_speed_stats(position_data[first_waypoint_index:last_waypoint_index+1])
                if speeds:
                    speed_times = [t for t, _ in speeds]
                    speed_values = [s for _, s in speeds]
                    max_speed = max(speed_values) if speed_values else 0
                    avg_inst_speed = sum(speed_values) / len(speed_values) if speed_values else 0
                    print(f"Maximum speed: {max_speed:.2f} m/s ({max_speed * 3.6:.2f} km/h)")
                    print(f"Average instantaneous speed: {avg_inst_speed:.2f} m/s ({avg_inst_speed * 3.6:.2f} km/h)")
                
                # Create 3D plot of drone path and waypoints - Smaller size for A4 page
                plt.rcParams.update({'font.size': 10})  # Increase base font size
                
                # A4 size in inches: 8.27 x 11.69
                # Half A4 (landscape orientation would be about 5.8 x 8.27)
                fig = plt.figure(figsize=(8, 6))
                ax = fig.add_subplot(111, projection='3d')
                
                # Extract the portion of the flight between first and last waypoints
                flight_x = [x for _, x, _, _ in flight_positions]
                flight_y = [y for _, _, y, _ in flight_positions]
                flight_z = [z for _, _, _, z in flight_positions]
                
                # Get waypoint coordinates for plotting
                waypoint_x = [w[0] for w in waypoints]
                waypoint_y = [w[1] for w in waypoints]
                waypoint_z = [w[2] for w in waypoints]
                
                # Define colormap and normalization for speeds
                cmap = cm.jet
                norm = Normalize(vmin=0, vmax=max(speeds_for_path) if speeds_for_path else 1)
                
                # Plot drone path with color based on speed - thinner line for smaller plot
                for i in range(1, len(flight_x)):
                    speed = speeds_for_path[i]
                    color = cmap(norm(speed))
                    ax.plot3D([flight_x[i-1], flight_x[i]], 
                             [flight_y[i-1], flight_y[i]], 
                             [flight_z[i-1], flight_z[i]], 
                             color=color, linewidth=2)
                
                # Plot ideal path between waypoints
                ax.plot3D(waypoint_x, waypoint_y, waypoint_z, 'r--', linewidth=1, label='Ideal Path')
                
                # Plot waypoints
                ax.scatter(waypoint_x, waypoint_y, waypoint_z, color='red', s=50, marker='*', label='Waypoints')
                
                # Add waypoint labels with smaller font
                for i, (x, y, z) in enumerate(waypoints):
                    ax.text(x, y, z, f"{i+1}", color='darkred', fontsize=8)
                
                # Calculate the maximum range needed across all dimensions
                margin = 0.25  # 25cm margin
                x_min = min(min(flight_x), min(waypoint_x)) - margin
                x_max = max(max(flight_x), max(waypoint_x)) + margin
                y_min = min(min(flight_y), min(waypoint_y)) - margin
                y_max = max(max(flight_y), max(waypoint_y)) + margin
                z_min = min(min(flight_z), min(waypoint_z)) - margin
                z_max = max(max(flight_z), max(waypoint_z)) + margin
                
                # Determine the maximum range across all dimensions for equal scaling
                max_range = max(x_max - x_min, y_max - y_min, z_max - z_min)
                
                # Calculate the mid point of each dimension
                x_mid = (x_max + x_min) / 2
                y_mid = (y_max + y_min) / 2
                z_mid = (z_max + z_min) / 2
                
                # Set axis limits to be equal in all dimensions
                ax.set_xlim(x_mid - max_range/2.6, x_mid + max_range/2.6)
                ax.set_ylim(y_mid - max_range/2.6, y_mid + max_range/2.6)
                ax.set_zlim(z_mid - max_range/2.6, z_mid + max_range/2.6)
                
                # Enforce equal scaling in all dimensions
                ax.set_box_aspect([1, 1, 1])
                
                # Labels and title
                ax.set_xlabel('X Position (m)')
                ax.set_ylabel('Y Position (m)')
                ax.set_zlabel('Z Position (m)')
                shape_name = os.path.basename(os.path.dirname(bag_file_path))  # Get the shape name from directory
                ax.set_title(f'Drone Path Speed Analysis', fontsize=12)
                
                # Add a colorbar for speed reference
                sm = plt.cm.ScalarMappable(cmap=cmap, norm=norm)
                sm.set_array([])
                cbar = fig.colorbar(sm, ax=ax, pad=0.05, shrink=0.8)
                cbar.set_label('Speed (m/s)', fontsize=9)
                cbar.ax.tick_params(labelsize=8)
                
                # Add legend with smaller font
                ax.legend(loc='upper right', fontsize=8, framealpha=0.7)
                
                # Add text with statistics - moved further to the left
                stats_text = (f"Avg Speed: {avg_speed_ideal:.2f} m/s\n"
                             f"Max Speed: {max_speed:.2f} m/s\n"
                             f"Path Length: {ideal_path_length:.2f} m\n"
                             f"Flight Time: {flight_time:.2f} s")
                ax.text2D( - 0.1, 0.01, stats_text, transform=ax.transAxes, 
                         fontsize=8, verticalalignment='bottom', horizontalalignment='left',
                         bbox=dict(boxstyle="round,pad=0.3", facecolor='white', alpha=0.7))
                
                # Improve tick label size
                ax.tick_params(axis='both', which='major', labelsize=8)
                
                
                # Save results to a file
                results_dir = '/home/jacob/Documents/the_ros2_ws/src/Crazyflie-P4-Group-460/Data_Processing/Results/speed_test'
                os.makedirs(results_dir, exist_ok=True)
                
                shape_name = os.path.basename(os.path.dirname(bag_file_path))
                
                # Same content for results text file
                with open(f'{results_dir}/speed_test_{shape_name}.txt', 'w') as f:
                    f.write(f"Path Shape: {shape_name}\n")
                    f.write(f"Number of waypoints: {len(waypoints)}\n")
                    f.write(f"Flight time (first to last waypoint): {flight_time:.2f} seconds\n")
                    f.write(f"Path length: {ideal_path_length:.2f} meters\n")
                    f.write(f"Average speed: {avg_speed_ideal:.2f} m/s\n")
                    
                    f.write("\n")
                    f.write(f"Fun Facts til Tobias:\n")
                    f.write(f"Maximum speed: {max_speed:.2f} m/s ({max_speed * 3.6:.2f} km/h)\n")
                    f.write(f"Actual drone path length: {actual_path_length:.2f} meters\n")
                    f.write(f"Path efficiency: {ideal_path_length/actual_path_length*100:.2f}%\n")
                    
                
                # Save the figures with appropriate DPI for A4 printing (300 DPI is standard for print)
                # For half A4 page, making sure it fits well
                fig.tight_layout()
                fig.savefig(f'{results_dir}/speed_test_{shape_name}_path.png', bbox_inches='tight')
                fig.savefig(f'{results_dir}/speed_test_{shape_name}_path.pdf', bbox_inches='tight')  # PDF for best print quality
                
                plt.show()
                
            else:
                print("Error: Drone did not reach the final waypoint")
        else:
            print("Error: Drone did not reach the first waypoint")
    else:
        print("Error: No waypoints found in the data")