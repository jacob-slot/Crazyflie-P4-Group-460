import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():

    # Specify package and file paths
    rviz_config_file = 'config/test.rviz'

    # Get file paths
    rviz_config_path = os.path.join(get_package_share_directory('gazebo_c_pkg'), rviz_config_file)
    vprn_mocap_path = os.path.join(get_package_share_directory('vrpn_mocap'), 'launch')
    control_pkg_path = os.path.join(get_package_share_directory('control_pkg'))

    # RViz2 with custom config
    node_rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path]
    )
    
    # ROS 2 bag recording node
    rosbag_record_node = Node(
        package='ros2',
        executable='bag',
        name='rosbag_record',
        output='screen',
        arguments=['record', '/vrpn_mocap/Crazyflie/pose_rpy', '/CfLog', '/control_signals']
    )
    
    
    # vrpn_mocap
    mocap_node = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(vprn_mocap_path, 'client.launch.yaml')
        ),
        launch_arguments={
            'server': '192.168.1.33',
            'port': '3883'
        }.items()
    )
    
    # pathfinding node
    node_pathfinding = Node(
        package='pathfinding',
        executable='pathfinding',
        output='screen',
        name='pathfinding',
    )

    # controll_pkg launch file
    control_node = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(control_pkg_path, 'controller_launch.xml')
        ),
    )

    # drone_interface
    drone_interface_node = Node(
        package='drone_interface_pkg',
        executable='drone_interface',
        output='screen',
        name='drone_interface',
    )


    return LaunchDescription([
        #node_rviz2,
        mocap_node,
        node_pathfinding,
        control_node,
        drone_interface_node,
        rosbag_record_node,
        
    ])

