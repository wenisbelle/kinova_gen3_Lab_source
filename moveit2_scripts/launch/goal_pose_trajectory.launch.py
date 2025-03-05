import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder



def generate_launch_description():
    
    moveit_config = MoveItConfigsBuilder(
        "gen3", package_name="kinova_gen3_7dof_robotiq_2f_85_moveit_config"
    ).to_moveit_configs()


    # MoveItCpp demo executable
   
    moveit_cpp_node = Node(
    name="goal_pose_trajectory",
    package="moveit2_scripts",
    executable="goal_pose_trajectory",
    output="screen",
    parameters=[
        moveit_config.robot_description,
        moveit_config.robot_description_semantic,
        moveit_config.robot_description_kinematics,
        {'use_sim_time': True},
    ],
)

    return LaunchDescription(
        [moveit_cpp_node]
    )