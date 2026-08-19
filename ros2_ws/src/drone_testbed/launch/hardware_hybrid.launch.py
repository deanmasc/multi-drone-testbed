"""Hybrid hardware/simulation launch: one real drone, the rest virtual.

Starts:
  - mocap_state_node  (VICON -> /<hw_drone>/state)
  - crazyflie_node    (/<hw_drone>/cmd_accel -> the real Crazyflie)
  - drone_node        (one per remaining drone in the config, simulated)
  - algorithm_manager (runs the algorithm over ALL of them together)
  - live_visualizer   (draws real and virtual identically)

Why this works with no special machinery: every drone publishes /<id>/state,
and algorithm_manager cannot tell whether a given state came from VICON or
from a numerical integrator. So the real drone genuinely reacts to the
simulated ones and they react back. The only asymmetry is that the real one
has flight dynamics, radio latency and tracking error, which is exactly the
part worth testing.

The hardware drone is whichever id is named by the hw_drone argument; it must
appear in the config's drone list. Every other drone in that list is simulated.

Prerequisites on the lab machine:
  ros2 launch crazyflie launch.py backend:=cflib     (terminal 1)

Usage:
  ros2 launch drone_testbed hardware_hybrid.launch.py \
      config:=config/testbed_hybrid.yaml cf_name:=drone_1
"""

import os
import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _launch_setup(context):
    cfg = lambda name: LaunchConfiguration(name).perform(context)

    config_rel = cfg('config')
    try:
        from ament_index_python.packages import get_package_share_directory
        config_path = os.path.join(
            get_package_share_directory('drone_testbed'), config_rel,
        )
    except Exception:
        config_path = config_rel
    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)

    sim_cfg = config.get('simulation', {})
    drone_ids = [d['id'] for d in config['drones']]
    hw_drone = cfg('hw_drone')
    if hw_drone not in drone_ids:
        raise RuntimeError(
            f'hw_drone "{hw_drone}" is not in {config_rel}, which lists '
            f'{drone_ids}. The real drone must be one of the configured drones '
            f'or the algorithm will never see its state and will never command '
            f'anything.'
        )

    geofence = float(cfg('geofence'))
    nodes = []

    # --- the real drone -------------------------------------------------
    nodes.append(Node(
        package='drone_testbed', executable='mocap_state_node',
        name='mocap_state_node',
        parameters=[{'drone_id': hw_drone, 'mocap_name': cfg('mocap_name')}],
        output='screen',
    ))
    nodes.append(Node(
        package='drone_testbed', executable='crazyflie_node',
        name='crazyflie_node',
        parameters=[{
            'drone_id': hw_drone,
            'cf_name': cfg('cf_name'),
            'flight_duration': float(cfg('flight_duration')),
            'use_mocap_yaw': cfg('use_mocap_yaw').lower() in ('true', '1'),
            'geofence': geofence,
            'max_lead': float(cfg('max_lead')),
        }],
        output='screen',
    ))

    # --- the virtual ones -----------------------------------------------
    for drone in config['drones']:
        if drone['id'] == hw_drone:
            continue
        nodes.append(Node(
            package='drone_testbed', executable='drone_node',
            name=f'drone_node_{drone["id"]}',
            parameters=[{
                'drone_id': drone['id'],
                'initial_position': drone.get('initial_position', [0.0, 0.0]),
                'initial_velocity': drone.get('initial_velocity', [0.0, 0.0]),
                'dt': sim_cfg.get('dt', 0.1),
                'sim_rate': sim_cfg.get('sim_rate', 10.0),
                # Hold the simulated drones to the same box the real one is
                # geofenced to, so a virtual agent cannot lead the real one
                # somewhere it would be aborted for following.
                'bounds': sim_cfg.get('bounds', geofence),
            }],
            output='screen',
        ))

    # --- shared ----------------------------------------------------------
    nodes.append(Node(
        package='drone_testbed', executable='algorithm_manager',
        name='algorithm_manager',
        parameters=[{
            'config_file': config_rel,
            # Must outlast takeoff. Until this fires nothing moves, so the
            # simulated drones hold station while the real one climbs.
            'auto_start_delay': float(cfg('auto_start_delay')),
        }],
        output='screen',
    ))
    nodes.append(Node(
        package='drone_testbed', executable='live_visualizer',
        name='live_visualizer',
        condition=IfCondition(LaunchConfiguration('gui')),
        parameters=[{
            'config_file': config_rel,
            'geofence': geofence,
            'max_lead': float(cfg('max_lead')),
            'trail_seconds': float(cfg('trail_seconds')),
        }],
        output='screen',
    ))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value='config/testbed_hybrid.yaml'),
        # Which configured drone is the real one; all others are simulated.
        DeclareLaunchArgument('hw_drone', default_value='drone1'),
        DeclareLaunchArgument('cf_name', default_value='drone_1'),
        DeclareLaunchArgument('mocap_name', default_value='drone_1'),
        DeclareLaunchArgument('flight_duration', default_value='60.0'),
        DeclareLaunchArgument('use_mocap_yaw', default_value='true'),
        DeclareLaunchArgument('geofence', default_value='1.5'),
        DeclareLaunchArgument('max_lead', default_value='0.3'),
        # Longer than hardware_single's 2.0s: this must exceed takeoff, or the
        # simulated drones fly the formation without the real one.
        DeclareLaunchArgument('auto_start_delay', default_value='8.0'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('trail_seconds', default_value='40.0'),
        OpaqueFunction(function=_launch_setup),
    ])
