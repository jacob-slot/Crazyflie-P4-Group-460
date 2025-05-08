import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import os

def read_csv_file(csv_file_path, downsample_factor=10):
    """
    Read position and reference data from a CSV file.
    The CSV file should have columns: timestamp, x, y, z, x_ref, y_ref, z_ref
    
    Args:
        csv_file_path (str): Path to the CSV file
        downsample_factor (int): Only keep every nth point to reduce plotting complexity
        
    Returns:
        tuple: Two lists containing position and reference data
    """
    # Read the CSV file
    try:
        data = pd.read_csv(csv_file_path)
        
        # Downsample data
        data = data.iloc[::downsample_factor]
        print(f"Downsampled data from {len(pd.read_csv(csv_file_path))} to {len(data)} points")
        
        # Extract position data
        positions = [(row.x, row.y, row.z) for _, row in data.iterrows()]
        
        # Extract reference data 
        references = [(row.x_ref, row.y_ref, row.z_ref) for _, row in data.iterrows()]
        
        return positions, references
    except Exception as e:
        print(f"Error reading CSV file: {e}")
        return [], []

def plot_3d_trajectory(positions, references):
    """
    Create a 3D plot of the drone trajectory and reference waypoints.
    
    Args:
        positions (list): List of (x, y, z) position tuples
        references (list): List of (x_ref, y_ref, z_ref) reference tuples
    """
    if not positions or not references:
        print("No data to plot")
        return
        
    # Extract x, y, z coordinates
    x_positions = [pos[0] for pos in positions]
    y_positions = [pos[1] for pos in positions]
    z_positions = [pos[2] for pos in positions]
    
    # Extract reference waypoints
    ref_x_positions = [ref[0] for ref in references]
    ref_y_positions = [ref[1] for ref in references]
    ref_z_positions = [ref[2] for ref in references]
    
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
    x_min = min(min(x_positions), min(ref_x_positions)) - margin
    x_max = max(max(x_positions), max(ref_x_positions)) + margin
    y_min = min(min(y_positions), min(ref_y_positions)) - margin
    y_max = max(max(y_positions), max(ref_y_positions)) + margin
    z_min = min(min(z_positions), min(ref_z_positions)) - margin
    z_max = max(max(z_positions), max(ref_z_positions)) + margin
    
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

if __name__ == "__main__":
    # Ask for CSV file path if not provided
    csv_file_path = r'G:\My Drive\AAU\4 Semester\ros_2_ws\src\Crazyflie-P4-Group-460\Data_Processing\csv\simulink_output_only_p_path.csv'
    
    # Set downsample factor (higher number = fewer points)
    downsample_factor = 500  # Only plot every 20th point
    
    # Check if file exists
    if not os.path.exists(csv_file_path):
        print(f"Error: File '{csv_file_path}' not found.")
    else:
        # Read data and create plot
        positions, references = read_csv_file(csv_file_path, downsample_factor)
        plot_3d_trajectory(positions, references)