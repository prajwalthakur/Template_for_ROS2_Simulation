from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the path to the installed project_utils config directory
    # config = os.path.join(
    #     get_package_share_directory('project_utils'),
    #     'config',
    #     'sim_config.yaml'
    # )

    return LaunchDescription([
        Node(
            package='visualizer',
            executable='visualizer_node',
            name='visualizer_node',
            output='screen',
            # parameters=[config]
        )
    ])
