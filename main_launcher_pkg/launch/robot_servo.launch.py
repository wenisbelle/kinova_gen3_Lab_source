import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node 

def generate_launch_description():

    moveit_servo = os.path.join(
        get_package_share_directory('moveit_servo'), 'launch', 'kinova_joy.launch.py')
    
    joy_node = Node(
        package="joy_linux",
        executable="joy_linux_node",
        output="screen",
    )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', os.path.join(get_package_share_directory('main_launcher_pkg'), 'config', 'moveit_servo.rviz')],
    )

    return LaunchDescription([
        # Launch first_launch.py
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(moveit_servo),
            launch_arguments={}.items(),
        ),
        joy_node,
        rviz2

    ])
