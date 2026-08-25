"""Hybrid hardware/simulation launch: N real drones, the rest virtual.

Starts:
  - mocap_state_node  (VICON -> /<id>/state)       one per real drone
  - crazyflie_node    (/<id>/cmd_accel -> radio)   one per real drone
  - drone_node        (one per remaining drone in the config, simulated)
  - algorithm_manager (runs the algorithm over ALL of them together)
  - live_visualizer   (draws real and virtual identically)

Why this works with no special machinery: every drone publishes /<id>/state,
and algorithm_manager cannot tell whether a given state came from VICON or
from a numerical integrator. So the real drone genuinely reacts to the
simulated ones and they react back. The only asymmetry is that the real one
has flight dynamics, radio latency and tracking error, which is exactly the
part worth testing.

The hardware drones are the ids named by hw_drone, which takes a
comma-separated list. Each must appear in the config's drone list; every other
drone in that list is simulated. cf_name and mocap_name are matched to it
positionally, so the three lists must be the same length and in the same order.

ALTITUDE. The algorithms are 2D -- z is pinned at the hover height -- and they
have no notion of one drone's physical extent, so nothing in the control law
prevents two real drones sharing a point in the xy plane. With more than one
real drone, takeoff_height therefore staggers them by height_step (0.40m) by
default, which makes a planar crossing harmless. Pass an explicit
comma-separated list to override. This is a guard, not a licence: check the
planned minimum separation before you fly, because 0.40m of vertical offset
does not make prop wash from the upper drone irrelevant to the lower one.

Prerequisites on the lab machine (and crazyflies.yaml / motion_capture.yaml
must list EVERY real drone, not just the first):
  ros2 launch crazyflie launch.py backend:=cflib     (terminal 1)

Usage, one real drone:
  ros2 launch drone_testbed hardware_hybrid.launch.py \
      config:=config/testbed_hybrid.yaml cf_name:=drone_1

Usage, two real drones:
  ros2 launch drone_testbed hardware_hybrid.launch.py \
      config:=config/testbed_fig4.yaml \
      hw_drone:=drone1,drone4 cf_name:=drone_1,drone_4 \
      mocap_name:=drone_1,drone_4
"""

import os
import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _split(value):
    """Comma-separated launch argument -> list, tolerant of spaces."""
    return [part.strip() for part in value.split(',') if part.strip()]


def _heights(value, step, count):
    """Per-drone hover altitudes.

    A single value with several real drones is read as "start here and stack
    upward", because two real drones at one altitude flying a planar law will
    eventually occupy the same point -- the law has no collision term. An
    explicit list of the right length is taken as given.
    """
    given = [float(v) for v in _split(value)]
    if len(given) == count:
        return given
    if len(given) == 1:
        return [given[0] + i * step for i in range(count)]
    raise RuntimeError(
        f'takeoff_height must be a single value or exactly {count} '
        f'comma-separated values, one per real drone; got {given}'
    )


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

    hw_drones = _split(cfg('hw_drone'))
    cf_names = _split(cfg('cf_name'))
    mocap_names = _split(cfg('mocap_name'))

    missing = [d for d in hw_drones if d not in drone_ids]
    if missing:
        raise RuntimeError(
            f'hw_drone names {missing} are not in {config_rel}, which lists '
            f'{drone_ids}. Every real drone must be one of the configured '
            f'drones or the algorithm will never see its state and will never '
            f'command anything.'
        )
    if len(set(hw_drones)) != len(hw_drones):
        raise RuntimeError(f'hw_drone lists a drone twice: {hw_drones}')
    if not (len(cf_names) == len(mocap_names) == len(hw_drones)):
        raise RuntimeError(
            f'hw_drone ({len(hw_drones)}), cf_name ({len(cf_names)}) and '
            f'mocap_name ({len(mocap_names)}) must be lists of the same '
            f'length, matched positionally: '
            f'hw_drone={hw_drones} cf_name={cf_names} mocap_name={mocap_names}'
        )

    heights = _heights(cfg('takeoff_height'), float(cfg('height_step')),
                       len(hw_drones))

    geofence = float(cfg('geofence'))
    nodes = []

    # --- the real drones -------------------------------------------------
    for hw_drone, cf_name, mocap_name, height in zip(
            hw_drones, cf_names, mocap_names, heights):
        # Node names must be unique per drone. Two nodes sharing a name is
        # legal in ROS 2 but makes every `ros2 node`/`ros2 param` command
        # ambiguous, which is exactly the wrong thing while debugging a flight.
        nodes.append(Node(
            package='drone_testbed', executable='mocap_state_node',
            name=f'mocap_state_node_{hw_drone}',
            parameters=[{'drone_id': hw_drone, 'mocap_name': mocap_name}],
            output='screen',
        ))
        nodes.append(Node(
            package='drone_testbed', executable='crazyflie_node',
            name=f'crazyflie_node_{hw_drone}',
            parameters=[{
                'drone_id': hw_drone,
                'cf_name': cf_name,
                'flight_duration': float(cfg('flight_duration')),
                'use_mocap_yaw': cfg('use_mocap_yaw').lower() in ('true', '1'),
                'geofence': geofence,
                'max_lead': float(cfg('max_lead')),
                'takeoff_height': height,
            }],
            output='screen',
        ))

    # --- the virtual ones -----------------------------------------------
    for drone in config['drones']:
        if drone['id'] in hw_drones:
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
        # Which configured drones are real; all others are simulated.
        # Comma-separated, and cf_name/mocap_name match it positionally.
        DeclareLaunchArgument('hw_drone', default_value='drone1'),
        DeclareLaunchArgument('cf_name', default_value='drone_1'),
        DeclareLaunchArgument('mocap_name', default_value='drone_1'),
        # A single value stacks upward by height_step, one level per real
        # drone; a comma-separated list of the right length is used as given.
        DeclareLaunchArgument('takeoff_height', default_value='1.0'),
        DeclareLaunchArgument('height_step', default_value='0.4'),
        DeclareLaunchArgument('flight_duration', default_value='60.0'),
        DeclareLaunchArgument('use_mocap_yaw', default_value='true'),
        DeclareLaunchArgument('geofence', default_value='1.5'),
        DeclareLaunchArgument('max_lead', default_value='0.3'),
        # Must exceed takeoff, or the simulated drones fly the formation
        # without the real ones. Each real drone is its own process building
        # its own Crazyswarm client and taking off on its own clock, so the
        # margin needs to cover the slowest of them, not the average.
        DeclareLaunchArgument('auto_start_delay', default_value='12.0'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('trail_seconds', default_value='40.0'),
        OpaqueFunction(function=_launch_setup),
    ])
