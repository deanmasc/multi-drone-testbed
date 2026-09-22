"""Launch the shared algorithm manager for every configured algorithm."""
from launch_ros.actions import Node


def controller_nodes(config, config_path, auto_start_delay=2.0):
    return [Node(package='drone_testbed', executable='algorithm_manager',
                 name='algorithm_manager', parameters=[{
                     'config_file': config_path, 'auto_start_delay': auto_start_delay}],
                 output='screen')]
