"""Launch file for simulation mode.

Reads testbed.yaml and spawns:
  - One drone_node per drone
  - One algorithm_manager
  - One sim_visualizer
"""

import os
import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_setup(context):
    config_rel = LaunchConfiguration('config').perform(context)

    # Resolve config path relative to package share directory
    try:
        from ament_index_python.packages import get_package_share_directory
        pkg_share = get_package_share_directory('drone_testbed')
        config_path = os.path.join(pkg_share, config_rel)
    except Exception:
        config_path = config_rel

    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)

    sim_cfg = config['simulation']
    nodes = []

    # Spawn one drone_node per drone
    for drone in config['drones']:
        nodes.append(Node(
            package='drone_testbed',
            executable='drone_node',
            name=f'drone_node_{drone["id"]}',
            parameters=[{
                'drone_id': drone['id'],
                'initial_position': drone.get('initial_position', [0.0, 0.0]),
                'initial_velocity': drone.get('initial_velocity', [0.0, 0.0]),
                'dt': sim_cfg.get('dt', 0.1),
                'sim_rate': sim_cfg.get('sim_rate', 10.0),
                'bounds': sim_cfg.get('bounds', 2.0),
            }],
            output='screen',
        ))

    # Algorithm manager
    nodes.append(Node(
        package='drone_testbed',
        executable='algorithm_manager',
        name='algorithm_manager',
        parameters=[{
            'config_file': config_path,
        }],
        output='screen',
    ))

    # Visualizer
    nodes.append(Node(
        package='drone_testbed',
        executable='sim_visualizer',
        name='sim_visualizer',
        parameters=[{
            'config_file': config_path,
        }],
        output='screen',
    ))

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'config',
            default_value='config/testbed.yaml',
            description='Path to testbed YAML config file',
        ),
        OpaqueFunction(function=_launch_setup),
    ])
