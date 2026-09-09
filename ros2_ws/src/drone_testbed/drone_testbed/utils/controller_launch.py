"""Select independent ROS controllers for Kuramoto, legacy manager otherwise."""
from launch_ros.actions import Node


def controller_nodes(config, config_path, auto_start_delay=2.0):
    if config.get('algorithm', {}).get('name') != 'KuramotoFormation':
        return [Node(package='drone_testbed', executable='algorithm_manager',
                     name='algorithm_manager', parameters=[{
                         'config_file': config_path, 'auto_start_delay': auto_start_delay}],
                     output='screen')]
    nodes = [Node(package='drone_testbed', executable='kuramoto_controller',
                  name=f'kuramoto_{d["id"]}', parameters=[{
                      'config_file': config_path, 'drone_id': d['id']}], output='screen')
             for d in config['drones']]
    nodes.append(Node(package='drone_testbed', executable='formation_lifecycle',
                      name='formation_lifecycle', parameters=[{
                          'auto_start_delay': auto_start_delay}], output='screen'))
    return nodes
