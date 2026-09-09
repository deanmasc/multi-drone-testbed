"""Launch file for single drone hardware mode.

Starts:
  - mocap_state_node  (VICON → drone state)
  - crazyflie_node    (receives commands → sends to real drone)
  - algorithm_manager (runs selected algorithm)
  - live_visualizer   (real-time commanded vs actual plot; gui:=false to skip)

Does NOT start sim_visualizer or drone_node -- those are sim-only.

Prerequisites on the lab machine:
  1. Crazyswarm2 running: ros2 launch crazyflie launch.py
  2. VICON system running and publishing /poses
  3. Crazyradio USB dongle plugged in
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('drone_id', default_value='drone1'),
        DeclareLaunchArgument('cf_name', default_value='drone_1'),
        DeclareLaunchArgument('mocap_name', default_value='drone_1'),
        DeclareLaunchArgument(
            'config',
            default_value='config/testbed.yaml',
        ),
        # Seconds to fly after takeoff before auto-landing. 0 = until Ctrl+C.
        DeclareLaunchArgument('flight_duration', default_value='0.0'),
        # True for a multi-marker rigid body under "vendor" tracking, where
        # mocap orientation is real. Set false for single-marker/position-only
        # tracking, which has no orientation to read.
        DeclareLaunchArgument('use_mocap_yaw', default_value='true'),
        # Half-extent of the allowed x/y box about the world origin. Must be
        # large enough for the whole commanded path or the setpoint gets clipped
        # mid-flight; small enough that a runaway stays in the capture volume.
        DeclareLaunchArgument('geofence', default_value='1.5'),
        # How far the setpoint may get ahead of the drone before it stops
        # advancing. The anti-windup for a drone that cannot follow.
        DeclareLaunchArgument('max_lead', default_value='0.3'),
        # Real-time plot of commanded vs actual position. Set false when
        # running headless or over a connection without display forwarding.
        DeclareLaunchArgument('gui', default_value='true'),
        # Seconds of path held on the top-down view. Must exceed one lap of
        # whatever is being flown or the shape never closes: a 0.5m square lap
        # is 11.6s and a Trochoidal orbit is 21s, so the node's own 8s default
        # is too short for either. 30s covers both with room to compare laps.
        DeclareLaunchArgument('trail_seconds', default_value='30.0'),

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
                'flight_duration': ParameterValue(
                    LaunchConfiguration('flight_duration'), value_type=float,
                ),
                'use_mocap_yaw': ParameterValue(
                    LaunchConfiguration('use_mocap_yaw'), value_type=bool,
                ),
                'geofence': ParameterValue(
                    LaunchConfiguration('geofence'), value_type=float,
                ),
                'max_lead': ParameterValue(
                    LaunchConfiguration('max_lead'), value_type=float,
                ),
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

        # Real-time plot: where the drone is vs where it was told to be.
        # Takes the same geofence value as crazyflie_node so the box it draws
        # is the box actually being enforced.
        Node(
            package='drone_testbed',
            executable='live_visualizer',
            name='live_visualizer',
            condition=IfCondition(LaunchConfiguration('gui')),
            parameters=[{
                'config_file': LaunchConfiguration('config'),
                'geofence': ParameterValue(
                    LaunchConfiguration('geofence'), value_type=float,
                ),
                'max_lead': ParameterValue(
                    LaunchConfiguration('max_lead'), value_type=float,
                ),
                'trail_seconds': ParameterValue(
                    LaunchConfiguration('trail_seconds'), value_type=float,
                ),
            }],
            output='screen',
        ),
    ])
