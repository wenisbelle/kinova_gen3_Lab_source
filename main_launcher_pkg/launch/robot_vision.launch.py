import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node 
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    kinova_vision = os.path.join(
        get_package_share_directory('kinova_vision'), 'launch', 'kinova_vision.launch.py')
    

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(kinova_vision),
            launch_arguments={
                'depth_registration': 'true',
                }.items(),
        ),

    ])
