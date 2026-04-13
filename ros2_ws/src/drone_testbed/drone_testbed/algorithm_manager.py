"""Algorithm manager node -- central coordinator.

Subscribes to all drone states, runs the selected algorithm,
publishes acceleration commands to each drone. Supports live
algorithm swapping via a ROS2 service.

Topics:
  Subscribes: /<drone_id>/state      (Float64MultiArray [x, y, vx, vy])
  Publishes:  /<drone_id>/cmd_accel  (Float64MultiArray [ax, ay])
  Publishes:  /sim/control           (Int32)

Services:
  /set_algorithm      (std_srvs/SetString-like via String request)
  /reset_simulation   (std_srvs/Empty)
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Int32, String
from std_srvs.srv import Empty

from drone_testbed.utils.types import DroneState, ControlOutput
from drone_testbed.utils.config_loader import load_config
from drone_testbed.algorithms.registry import get_algorithm, list_algorithms
import drone_testbed.algorithms  # triggers registration


class AlgorithmManagerNode(Node):

    def __init__(self):
        super().__init__('algorithm_manager')

        self.declare_parameter('config_file', 'config/testbed.yaml')
        config_path = self.get_parameter('config_file').value
        self._config = load_config(config_path)

        sim_cfg = self._config['simulation']
        self._dt = 1.0 / sim_cfg.get('control_rate', 5.0)
        self._drone_ids = [d['id'] for d in self._config['drones']]

        # Current drone states
        self._states = {}

        # Subscribe to each drone's state
        self._state_subs = []
        for drone_id in self._drone_ids:
            sub = self.create_subscription(
                Float64MultiArray,
                f'/{drone_id}/state',
                lambda msg, did=drone_id: self._state_callback(did, msg),
                10,
            )
            self._state_subs.append(sub)

        # Publishers for each drone's command
        self._cmd_pubs = {}
        for drone_id in self._drone_ids:
            self._cmd_pubs[drone_id] = self.create_publisher(
                Float64MultiArray,
                f'/{drone_id}/cmd_accel',
                10,
            )

        # Sim control publisher
        self._control_pub = self.create_publisher(Int32, '/sim/control', 10)

        # Algorithm status publisher
        self._algo_status_pub = self.create_publisher(String, '/sim/algorithm_status', 10)

        # Load initial algorithm
        algo_cfg = self._config.get('algorithm', {})
        algo_name = algo_cfg.get('name', 'LeaderFollower')
        algo_params = algo_cfg.get('params', {})
        self._load_algorithm(algo_name, algo_params)

        # Services
        self.create_service(Empty, '/reset_simulation', self._reset_callback)

        # Use a string topic for algorithm switching (simpler than custom srv)
        self.create_subscription(
            String, '/set_algorithm', self._set_algorithm_callback, 10,
        )

        # Control timer
        self.create_timer(self._dt, self._control_loop)

        # Auto-start after a short delay to let drone nodes initialize
        self.create_timer(2.0, self._auto_start, callback_group=None)
        self._started = False

        self.get_logger().info(
            f'Algorithm manager started with {len(self._drone_ids)} drones, '
            f'algorithm={algo_name}'
        )
        self.get_logger().info(f'Available algorithms: {list_algorithms()}')

    def _load_algorithm(self, name: str, params: dict):
        self._algorithm = get_algorithm(name)
        self._algorithm.configure(params, self._drone_ids)
        self.get_logger().info(f'Loaded algorithm: {name}')

        status_msg = String()
        status_msg.data = name
        self._algo_status_pub.publish(status_msg)

    def _state_callback(self, drone_id: str, msg: Float64MultiArray):
        if len(msg.data) >= 4:
            self._states[drone_id] = DroneState.from_flat(drone_id, list(msg.data))

    def _control_loop(self):
        if not self._started:
            return
        if len(self._states) < len(self._drone_ids):
            return  # Wait for all drones to report state

        controls = self._algorithm.compute_controls(self._states, self._dt)

        for drone_id, ctrl in controls.items():
            if drone_id in self._cmd_pubs:
                msg = Float64MultiArray()
                msg.data = ctrl.to_flat()
                self._cmd_pubs[drone_id].publish(msg)

    def _auto_start(self):
        if not self._started:
            self._started = True
            msg = Int32()
            msg.data = 1  # start
            self._control_pub.publish(msg)
            self.get_logger().info('Simulation started')

    def _set_algorithm_callback(self, msg: String):
        algo_name = msg.data.strip()
        self.get_logger().info(f'Switching algorithm to: {algo_name}')
        try:
            algo_params = self._config.get('algorithm', {}).get('params', {})
            self._load_algorithm(algo_name, algo_params)
        except ValueError as e:
            self.get_logger().error(str(e))

    def _reset_callback(self, request, response):
        self._started = False
        self._algorithm.reset()

        # Tell all drones to reset
        msg = Int32()
        msg.data = 2  # reset
        self._control_pub.publish(msg)

        self.get_logger().info('Simulation reset')

        # Re-start after delay
        self.create_timer(2.0, self._auto_start)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = AlgorithmManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
