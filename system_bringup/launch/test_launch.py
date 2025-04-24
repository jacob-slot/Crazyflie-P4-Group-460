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
    urdf_file = 'models/crazyflie_xacro/crazyflie.urdf.xacro'
    rviz_config_file = 'config/test.rviz'

    # Get file paths
    xacro_path = os.path.join(get_package_share_directory('gazebo_c_pkg'), urdf_file)
    rviz_config_path = os.path.join(get_package_share_directory('gazebo_c_pkg'), rviz_config_file)
    vprn_mocap_path = os.path.join(get_package_share_directory('vrpn_mocap'), 'launch')


    # Process URDF/Xacro
    robot_description_raw = xacro.process_file(xacro_path).toxml()

    # Robot State Publisher
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_raw}]

    )

    # RViz2 with custom config
    node_rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path]
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

    return LaunchDescription([
        node_rviz2,
        mocap_node,
        #gazebo,
        #spawn_robot
        #spawn_entity
        #joint_state_publisher,
        #node_robot_state_publisher,
    ])

