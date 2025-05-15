import rosbag2_py
from rclpy.serialization import deserialize_message
from geometry_msgs.msg import Pose
import matplotlib.pyplot as plt
import numpy as np
import os

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

def analyze_overshoot(times, positions, reference_value, axis_name, start_time_idx=0):
    """
    Analyze the overshoot along a specific axis.
    
    Args:
        times: List of timestamps
        positions: List of position values along the axis
        reference_value: The reference value to measure overshoot from
        axis_name: Name of the axis (X, Y, or Z)
        start_time_idx: Index to start analysis from
        
    Returns:
        Dict containing overshoot metrics
    """
    # Start from the specified index
    times = times[start_time_idx:]
    positions = positions[start_time_idx:]
    
    # Calculate error relative to reference
    errors = [pos - reference_value for pos in positions]
    
    # First let's check if we're approaching the reference value from above or below
    # We need to know this to correctly identify overshoot
    approaching_from_below = False
    approaching_from_above = False
    
    # Look at the first 20% of the data to determine approach direction
    check_range = min(len(positions) // 5, 100)  # Either 20% or first 100 points
    
    if check_range > 0:
        avg_initial = sum(positions[:check_range]) / check_range
        if avg_initial < reference_value:
            approaching_from_below = True
        else:
            approaching_from_above = True
    
    # Find points where drone crosses the reference value
    crossings = []
    crossing_times = []
    
    # Detect reference crossings by checking for sign changes in the error
    for i in range(1, len(errors)):
        if (errors[i-1] <= 0 and errors[i] > 0) or (errors[i-1] >= 0 and errors[i] < 0):
            crossings.append(i)
            crossing_times.append(times[i])
    
    if not crossings:
        return {
            "max_overshoot": 0,
            "max_overshoot_time": 0,
            "overshoot_percent": 0,
            "settling_time": 0,
            "has_crossings": False,
            "position_at_overshoot": reference_value,
            "time_at_overshoot": 0
        }
    
    # Look for the first significant crossing where drone reaches the reference
    first_real_crossing = 0
    for i, crossing in enumerate(crossings):
        # Skip crossing if it's too early in the recording
        if times[crossing] < 1.0:
            continue
        
        # Check if the crossing is a proper approach to the reference
        if (approaching_from_below and errors[crossing] > 0) or \
           (approaching_from_above and errors[crossing] < 0):
            first_real_crossing = i
            break
    
    if first_real_crossing >= len(crossings):
        return {
            "max_overshoot": 0,
            "max_overshoot_time": 0,
            "overshoot_percent": 0,
            "settling_time": 0,
            "has_crossings": False,
            "position_at_overshoot": reference_value,
            "time_at_overshoot": 0
        }
    
    crossing_idx = crossings[first_real_crossing]
    
    # Look for overshoot: maximum deviation in the opposite direction after crossing
    max_overshoot = 0
    max_overshoot_index = crossing_idx
    max_overshoot_time = 0
    position_at_overshoot = reference_value
    
    # Define search window - look for about 5-10 seconds after crossing
    # or 10-15 seconds into the recording if that's when we expect overshoot
    window_end = min(len(positions), crossing_idx + 300)  # Assuming 30-50 Hz data rate
    
    if approaching_from_below:
        # If approaching from below, overshoot is when we go above reference
        for i in range(crossing_idx, window_end):
            error = positions[i] - reference_value
            if error > max_overshoot:
                max_overshoot = error
                max_overshoot_index = i
                max_overshoot_time = times[i] - times[crossing_idx]
                position_at_overshoot = positions[i]
    else:
        # If approaching from above, overshoot is when we go below reference
        for i in range(crossing_idx, window_end):
            error = reference_value - positions[i]
            if error > max_overshoot:
                max_overshoot = error
                max_overshoot_index = i
                max_overshoot_time = times[i] - times[crossing_idx]
                position_at_overshoot = positions[i]
    
    # Calculate overshoot percentage (if applicable)
    overshoot_percent = (max_overshoot / abs(reference_value)) * 100 if reference_value != 0 else float('inf')
    
    # Calculate settling time (time to stay within 2% of reference)
    settling_threshold = 0.05 * abs(reference_value) if reference_value != 0 else 0.02
    settling_time = 0
    settled = False
    
    for i in range(max_overshoot_index, len(positions)):
        if abs(positions[i] - reference_value) <= settling_threshold:
            if not settled:
                settling_time = times[i] - times[crossing_idx]
                settled = True
        else:
            settled = False
    
    # Check if this is a true overshoot
    if max_overshoot < 0.005:  # Less than 5mm might be noise
        max_overshoot = 0
    
    # Return analysis results
    return {
        "max_overshoot": max_overshoot,
        "max_overshoot_time": max_overshoot_time,
        "overshoot_percent": overshoot_percent,
        "settling_time": settling_time,
        "has_crossings": True,
        "position_at_overshoot": position_at_overshoot,
        "time_at_overshoot": times[max_overshoot_index] if max_overshoot_index < len(times) else 0
    }

if __name__ == "__main__":
    # Path to bag file - use the specified file
    bag_file_path = '/home/jacob/Documents/the_ros2_ws/src/Crazyflie-P4-Group-460/Data_Processing/bags/Nye tests/Waypoint-test-2/waypoint-test-1.5-1.0-1.25/2025-05-13-12-46-05_0.mcap'
    
    # Check if file exists, otherwise use the default
    if not os.path.exists(bag_file_path):
        print(f"Warning: Specified bag file not found. Attempting to use a different file.")
        bag_file_path = '/home/jacob/Documents/the_ros2_ws/src/Crazyflie-P4-Group-460/Data_Processing/bags/Nye tests/xy-test-2/xy-test-1.0-0.0'
        
        # Find the first mcap file if specific one doesn't exist
        bag_files = [os.path.join(bag_file_path, f) for f in os.listdir(bag_file_path) if f.endswith('.mcap')]
        if bag_files:
            bag_file_path = bag_files[0]
            print(f"Found alternative bag file: {bag_file_path}")
        else:
            print("Error: No suitable bag files found.")
            exit(1)
    
    print(f"Analyzing bag file: {bag_file_path}")
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
    
    print(f"Reference positions: X: {x_ref:.3f}, Y: {y_ref:.3f}, Z: {z_ref:.3f} meters")

    # Define analysis time window (in seconds)
    # Focus window to look at data from the start to well past the expected overshoot time
    start_time = 0.0  # Start from beginning
    end_time = 40.0   # End time - should be plenty to catch overshoot at ~15 seconds
    
    # Find indices for the time window
    start_idx = 0
    while start_idx < len(relative_times) and relative_times[start_idx] < start_time:
        start_idx += 1
    
    end_idx = len(relative_times) - 1
    while end_idx > 0 and relative_times[end_idx] > end_time:
        end_idx -= 1
    
    # Filter data within the time interval
    filtered_times = relative_times[start_idx:end_idx+1]
    filtered_x = x_positions[start_idx:end_idx+1]
    filtered_y = y_positions[start_idx:end_idx+1]
    filtered_z = z_positions[start_idx:end_idx+1]

    # Analyze overshoot in each axis, starting analysis from beginning
    x_overshoot = analyze_overshoot(filtered_times, filtered_x, x_ref, "X")
    y_overshoot = analyze_overshoot(filtered_times, filtered_y, y_ref, "Y")
    z_overshoot = analyze_overshoot(filtered_times, filtered_z, z_ref, "Z")

    # Print analysis results
    print(f"\nOvershoot Analysis Results:")
    
    print(f"\nX-axis results:")
    if x_overshoot["has_crossings"]:
        print(f"Maximum overshoot: {x_overshoot['max_overshoot'] * 1000:.2f} mm")
        print(f"Absolute position at overshoot: {x_overshoot['position_at_overshoot']:.3f} m")
        print(f"Time of maximum overshoot: {x_overshoot['time_at_overshoot']:.2f} seconds")
        print(f"Overshoot percentage: {x_overshoot['overshoot_percent']:.2f}%")
        print(f"settling time (5%): {x_overshoot['settling_time']:.2f} seconds")
    else:
        print("No reference crossings detected")
    
    print(f"\nY-axis results:")
    if y_overshoot["has_crossings"]:
        print(f"Maximum overshoot: {y_overshoot['max_overshoot'] * 1000:.2f} mm")
        print(f"Absolute position at overshoot: {y_overshoot['position_at_overshoot']:.3f} m")
        print(f"Time of maximum overshoot: {y_overshoot['time_at_overshoot']:.2f} seconds")
        print(f"Overshoot percentage: {y_overshoot['overshoot_percent']:.2f}%")
        print(f"settling time (5%): {y_overshoot['settling_time']:.2f} seconds")
    else:
        print("No reference crossings detected")
    
    print(f"\nZ-axis results:")
    if z_overshoot["has_crossings"]:
        print(f"Maximum overshoot: {z_overshoot['max_overshoot'] * 1000:.2f} mm")
        print(f"Absolute position at overshoot: {z_overshoot['position_at_overshoot']:.3f} m")
        print(f"Time of maximum overshoot: {z_overshoot['time_at_overshoot']:.2f} seconds")
        print(f"Overshoot percentage: {z_overshoot['overshoot_percent']:.2f}%")
        print(f"settling time (5%): {z_overshoot['settling_time']:.2f} seconds")
    else:
        print("No reference crossings detected")

    # Create subplots for X, Y, and Z positions
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(12, 15))
    
    # Plot X position with annotations for overshoot
    ax1.plot(relative_times, x_positions, label='X Position', color='red')
    ax1.axhline(y=x_ref, color='gray', linestyle='--', label=f'Reference X ({x_ref:.3f}m)')
    if x_overshoot["has_crossings"] and x_overshoot["max_overshoot"] > 0:
        # Mark the overshoot directly at the time it occurs
        ax1.scatter([x_overshoot["time_at_overshoot"]], [x_overshoot["position_at_overshoot"]], color='red', s=100, marker='X')
        ax1.annotate(f'Max overshoot: {x_overshoot["max_overshoot"]*1000:.2f}mm', 
                    xy=(x_overshoot["time_at_overshoot"], x_overshoot["position_at_overshoot"]),
                    xytext=(x_overshoot["time_at_overshoot"]+2, x_overshoot["position_at_overshoot"]),
                    arrowprops=dict(facecolor='red', shrink=0.05))
    
    # Add text about timing
    ax1.text(0.6, 0.8, f"Overshoot at t={x_overshoot['time_at_overshoot']:.2f}s", 
             transform=ax1.transAxes, ha='center', va='top')
    
    ax1.set_title('Drone X Position Over Time with Overshoot Analysis')
    ax1.set_ylabel('X Position (meters)')
    ax1.set_xlim(5, 30)
    ax1.set_ylim(x_ref - 0.30, x_ref + 0.30)  # ±30cm around reference
    ax1.grid(True)
    ax1.legend()

    # Plot Y position with annotations for overshoot
    ax2.plot(relative_times, y_positions, label='Y Position', color='green')
    ax2.axhline(y=y_ref, color='gray', linestyle='--', label=f'Reference Y ({y_ref:.3f}m)')
    if y_overshoot["has_crossings"] and y_overshoot["max_overshoot"] > 0:
        # Mark the overshoot directly at the time it occurs
        ax2.scatter([y_overshoot["time_at_overshoot"]], [y_overshoot["position_at_overshoot"]], color='green', s=100, marker='X')
        ax2.annotate(f'Max overshoot: {y_overshoot["max_overshoot"]*1000:.2f}mm', 
                    xy=(y_overshoot["time_at_overshoot"], y_overshoot["position_at_overshoot"]),
                    xytext=(y_overshoot["time_at_overshoot"]+2, y_overshoot["position_at_overshoot"]),
                    arrowprops=dict(facecolor='green', shrink=0.05))
    
    # Add text about timing
    ax2.text(0.6, 0.8, f"Overshoot at t={y_overshoot['time_at_overshoot']:.2f}s", 
             transform=ax2.transAxes, ha='center', va='top')
    
    ax2.set_title('Drone Y Position Over Time with Overshoot Analysis')
    ax2.set_ylabel('Y Position (meters)')
    #ax2.set_xlim(0, end_time)
    ax2.set_xlim(5, 30)
    ax2.set_ylim(y_ref - 0.30, y_ref + 0.30)  # ±30cm around reference
    ax2.grid(True)
    ax2.legend()

    # Plot Z position with annotations for overshoot
    ax3.plot(relative_times, z_positions, label='Z Position', color='blue')
    ax3.axhline(y=z_ref, color='gray', linestyle='--', label=f'Reference Z ({z_ref:.3f}m)')
    if z_overshoot["has_crossings"] and z_overshoot["max_overshoot"] > 0:
        # Mark the overshoot directly at the time it occurs
        ax3.scatter([z_overshoot["time_at_overshoot"]], [z_overshoot["position_at_overshoot"]], color='blue', s=100, marker='X')
        ax3.annotate(f'Max overshoot: {z_overshoot["max_overshoot"]*1000:.2f}mm', 
                    xy=(z_overshoot["time_at_overshoot"], z_overshoot["position_at_overshoot"]),
                    xytext=(z_overshoot["time_at_overshoot"]+2, z_overshoot["position_at_overshoot"]),
                    arrowprops=dict(facecolor='blue', shrink=0.05))
    
    # Add text about timing
    ax3.text(0.6, 0.8, f"Overshoot at t={z_overshoot['time_at_overshoot']:.2f}s", 
             transform=ax3.transAxes, ha='center', va='top')
    
    ax3.set_title('Drone Z Position Over Time with Overshoot Analysis')
    ax3.set_xlabel('Time Since Start (seconds)')
    ax3.set_ylabel('Z Position (meters)')
    ax3.set_xlim(5, 30)
    ax3.set_ylim(z_ref - 0.30, z_ref + 0.30)  # ±30cm around reference
    ax3.grid(True)
    ax3.legend()

    # Adjust layout
    plt.tight_layout()
    
    # Create directory if it doesn't exist
    folder_path = '/home/jacob/Documents/the_ros2_ws/src/Crazyflie-P4-Group-460/Data_Processing/Results/overshoot_test'
    os.makedirs(folder_path, exist_ok=True)
    
    # Save results to file
    with open(f'{folder_path}/overshoot_test_({x_ref:.2f}_{y_ref:.2f}_{z_ref:.2f}).txt', 'w') as f:
        f.write(f"Analyzing bag: {bag_file_path}\n")
        f.write(f"Reference positions: X: {x_ref:.3f}, Y: {y_ref:.3f}, Z: {z_ref:.3f} meters\n\n")
        
        f.write(f"X-axis results:\n")
        if x_overshoot["has_crossings"]:
            f.write(f"Maximum overshoot: {x_overshoot['max_overshoot'] * 1000:.2f} mm\n")
            f.write(f"Absolute position at overshoot: {x_overshoot['position_at_overshoot']:.3f} m\n")
            f.write(f"Time of maximum overshoot: {x_overshoot['time_at_overshoot']:.2f} seconds\n")
            f.write(f"Overshoot percentage: {x_overshoot['overshoot_percent']:.2f}%\n")
        else:
            f.write("No reference crossings detected\n")
        
        f.write(f"\nY-axis results:\n")
        if y_overshoot["has_crossings"]:
            f.write(f"Maximum overshoot: {y_overshoot['max_overshoot'] * 1000:.2f} mm\n")
            f.write(f"Absolute position at overshoot: {y_overshoot['position_at_overshoot']:.3f} m\n")
            f.write(f"Time of maximum overshoot: {y_overshoot['time_at_overshoot']:.2f} seconds\n")
            f.write(f"Overshoot percentage: {y_overshoot['overshoot_percent']:.2f}%\n")
        else:
            f.write("No reference crossings detected\n")
        
        f.write(f"\nZ-axis results:\n")
        if z_overshoot["has_crossings"]:
            f.write(f"Maximum overshoot: {z_overshoot['max_overshoot'] * 1000:.2f} mm\n")
            f.write(f"Absolute position at overshoot: {z_overshoot['position_at_overshoot']:.3f} m\n")
            f.write(f"Time of maximum overshoot: {z_overshoot['time_at_overshoot']:.2f} seconds\n")
            f.write(f"Overshoot percentage: {z_overshoot['overshoot_percent']:.2f}%\n")
        else:
            f.write("No reference crossings detected\n")

    # Save the figure
    fig.savefig(f'{folder_path}/overshoot_test_({x_ref:.2f}_{y_ref:.2f}_{z_ref:.2f}).png')
    
    plt.show()