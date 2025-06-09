import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from launch_param_builder import ParameterBuilder


def generate_launch_description():
    
    description_arguments = {
        "robot_ip": "xxx.yyy.zzz.www",
        "use_fake_hardware": "true",
        "gripper": "robotiq_2f_85",
        "dof": "7",
    }
    
    moveit_config = (
        MoveItConfigsBuilder("gen3", package_name="kinova_gen3_7dof_robotiq_2f_85_moveit_config")
        .robot_description(mappings=description_arguments)
        .to_moveit_configs()
    )

    # Get parameters for the Pose Tracking node
    servo_params = {
        "moveit_servo": ParameterBuilder("moveit_servo")
        .yaml("config/kinova_pose_tracking_settings_sim.yaml")
        .yaml("config/kinova_simulated_config_pose_tracking.yaml")
        .to_dict()
    }


    pose_tracking_node = Node(
        package="moveit_servo",
        executable="servo_pose_tracking_demo",
        # prefix=['xterm -e gdb -ex run --args'],
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            servo_params,
            {"use_sim_time": True} 
        ],
    )


    return LaunchDescription(
        [
            pose_tracking_node,
        ]
    )
