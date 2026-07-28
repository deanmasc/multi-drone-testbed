"""Launch file for VICON-only pipeline validation (no flight).

Starts:
  - mocap_state_node  (VICON -> drone state)
  - algorithm_manager (runs PrintState, logs position/velocity)

Deliberately does NOT start crazyflie_node, so no commands are ever
sent to the drone -- this only validates that VICON pose data is
flowing into ROS2 correctly.

Prerequisites on the lab machine:
  1. Crazyswarm2's motion capture bridge running and publishing /poses
     (e.g. via `ros2 launch crazyflie launch.py`)
  2. VICON Tracker open, in Live mode, with the drone's rigid body visible
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('drone_id', default_value='drone1'),
        DeclareLaunchArgument('mocap_name', default_value='drone_4'),
        DeclareLaunchArgument(
            'config',
            default_value='config/testbed_vicon_test.yaml',
        ),

        # Converts VICON /poses -> /drone1/state
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

        # Runs PrintState -- logs state to terminal, never commands motion
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
