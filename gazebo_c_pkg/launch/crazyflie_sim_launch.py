from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    return LaunchDescription([
        
        # Gazebo simulation nodes

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                '/opt/ros/jazzy/share/ros_gz_sim/launch/gz_sim.launch.py'
            ]),
            launch_arguments={
                'gz_args': LaunchConfiguration('gz_args', default='../simulator_files/gazebo/worlds/crazyflie_world.sdf')
            
            }.items()
        ),

    ])

