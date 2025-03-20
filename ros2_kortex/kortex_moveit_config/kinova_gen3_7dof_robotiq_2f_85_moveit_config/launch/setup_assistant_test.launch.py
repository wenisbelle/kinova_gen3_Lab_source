from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='moveit_setup_assistant',
            executable='moveit_setup_assistant',
            name='moveit_setup_assistant',
            output='screen',
            parameters=[{
                'urdf': {
                    'package': 'kortex_description',
                    'relative_path': 'robots/gen3.xacro',
                    'xacro_args': {
                        'gripper': 'robotiq_2f_85',
                        'dof': '7',
                    },
                },
                'srdf': {
                    'relative_path': 'config/gen3.srdf',
                },
            }],
        ),
    ])