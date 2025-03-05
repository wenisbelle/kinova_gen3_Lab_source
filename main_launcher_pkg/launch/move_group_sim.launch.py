import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    kinova_move_group = os.path.join(
        get_package_share_directory('kinova_gen3_7dof_robotiq_2f_85_moveit_config'), 'launch', 'sim.launch.py')

    return LaunchDescription([
        # Launch first_launch.py
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(kinova_move_group),
            launch_arguments={
                'use_sim_time': 'true',
                'vision': 'true',
            }.items(),
        ),

    ])
