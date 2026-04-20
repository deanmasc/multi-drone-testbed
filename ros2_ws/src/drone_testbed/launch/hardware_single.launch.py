"""Launch file for single drone hardware mode.

Starts:
  - mocap_state_node  (VICON → drone state)
  - crazyflie_node    (receives commands → sends to real drone)
  - algorithm_manager (runs selected algorithm)

Does NOT start sim_visualizer or drone_node -- those are sim-only.

Prerequisites on the lab machine:
  1. Crazyswarm2 running: ros2 launch crazyflie launch.py
  2. VICON system running and publishing /poses
  3. Crazyradio USB dongle plugged in
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('drone_id', default_value='drone1'),
        DeclareLaunchArgument('cf_name', default_value='/cf1'),
        DeclareLaunchArgument('mocap_name', default_value='cf1'),
        DeclareLaunchArgument(
            'config',
            default_value='config/testbed.yaml',
        ),

        # Converts VICON /poses → /drone1/state
        Node(
            package='drone_testbed',
            executable='mocap_state_node',
            name='mocap_state_node',
            parameters=[{
                'drone_id': LaunchConfiguration('drone_id'),
                'mocap_name': LaunchConfiguration('mocap_name'),
            }],
            output='screen',
        ),

        # Connects to real Crazyflie, streams cmdFullState
        Node(
            package='drone_testbed',
            executable='crazyflie_node',
            name='crazyflie_node',
            parameters=[{
                'drone_id': LaunchConfiguration('drone_id'),
                'cf_name': LaunchConfiguration('cf_name'),
            }],
            output='screen',
        ),

        # Runs the selected distributed algorithm
        Node(
            package='drone_testbed',
            executable='algorithm_manager',
            name='algorithm_manager',
            parameters=[{
                'config_file': LaunchConfiguration('config'),
            }],
            output='screen',
        ),
    ])
