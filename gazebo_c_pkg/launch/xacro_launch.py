import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():

    # Specify package and file paths
    pkg_name = 'gazebo_c_pkg'
    urdf_file = 'models/crazyflie_xacro/crazyflie.urdf.xacro'
    rviz_config_file = 'config/test.rviz'

    # Get file paths
    xacro_path = os.path.join(get_package_share_directory(pkg_name), urdf_file)
    rviz_config_path = os.path.join(get_package_share_directory(pkg_name), rviz_config_file)


    # Process URDF/Xacro
    robot_description_raw = xacro.process_file(xacro_path).toxml()

    # Robot State Publisher
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_raw}]

    )

    # Joint State Publisher GUI
    joint_state_publisher = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen',
        parameters=[{'use_sim_time': True}]
        
    )

    # RViz2 with custom config
    node_rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path]
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch'), '/gz_sim.launch.py']),
        launch_arguments={'gz_args': [os.path.join(get_package_share_directory('gazebo_c_pkg'), 'worlds', 'empty.sdf'), '']}.items()
    )


    spawn_robot = Node(
    package='ros_gz_sim',
    executable='create',
    arguments=['-topic', '/robot_description'],  # <- this
    output='screen'
    )

    return LaunchDescription([
        node_robot_state_publisher,
        node_rviz2,
        #gazebo,
        #spawn_robot
        #spawn_entity
        #joint_state_publisher
    ])

