from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("gen3", package_name="kinova_gen3_7dof_robotiq_2f_85_moveit_config").to_moveit_configs()
    return generate_demo_launch(moveit_config)
