import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node 
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    kortex_bringup = os.path.join(
        get_package_share_directory('kortex_bringup'), 'launch', 'kortex_sim_control.launch.py')
    
    gripper_contact_point = Node(
        package='moveit2_scripts',  
        executable='tf2_contact_point',  
        name='tf2_contact_point',  
        output='screen', 
    )


    return LaunchDescription([
        # Launch first_launch.py
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(kortex_bringup),
            launch_arguments={
                'use_sim_time': 'true',
                'launch_rviz': 'false',
                'robot_controller': 'joint_trajectory_controller',
                'vision': 'true',
                'gripper': 'robotiq_2f_85'
            }.items(),
        ),
        gripper_contact_point,

    ])
