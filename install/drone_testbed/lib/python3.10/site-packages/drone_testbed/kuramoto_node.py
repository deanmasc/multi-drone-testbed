"""One local controller per process; only configured neighbors are observed."""
import math
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Float64, Int32, String
from std_srvs.srv import Empty
from drone_testbed.algorithms.kuramoto import KuramotoAgent
from drone_testbed.utils.config_loader import load_config
from drone_testbed.utils.types import DroneState


class KuramotoNode(Node):
    def __init__(self):
        super().__init__('kuramoto_controller')
        self.declare_parameter('config_file', 'config/testbed_kuramoto.yaml')
        self.declare_parameter('drone_id', 'drone1')
        config = load_config(self.get_parameter('config_file').value)
        self.drone_id = self.get_parameter('drone_id').value
        params = config['algorithm'].get('params', {})
        self.agent = KuramotoAgent(self.drone_id, params, [d['id'] for d in config['drones']])
        self.timeout = float(params.get('neighbor_timeout', .5))
        rate = float(config['simulation'].get('control_rate', 10.))
        if not math.isfinite(self.timeout) or self.timeout <= 0 or not math.isfinite(rate) or rate <= 0:
            raise ValueError('neighbor_timeout and control_rate must be positive and finite')
        self.states, self.phases = {}, {}
        self.running = False
        self.last_tick = self.now()
        self.cmd_pub = self.create_publisher(Float64MultiArray, f'/{self.drone_id}/cmd_accel', 10)
        self.phase_pub = self.create_publisher(Float64, f'/{self.drone_id}/phase', 10)
        self.subscriptions_local = []
        for d in [self.drone_id] + self.agent.neighbors:
            self.subscriptions_local.append(self.create_subscription(
                Float64MultiArray, f'/{d}/state', lambda msg, d=d: self.state_callback(d, msg), 10))
        for d in self.agent.neighbors:
            self.subscriptions_local.append(self.create_subscription(
                Float64, f'/{d}/phase', lambda msg, d=d: self.phase_callback(d, msg), 10))
        self.create_subscription(Int32, '/sim/control', self.control_callback, 10)
        self.create_timer(1. / rate, self.tick)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def state_callback(self, drone_id, msg):
        if len(msg.data) >= 4 and np.all(np.isfinite(msg.data[:4])):
            self.states[drone_id] = (DroneState.from_flat(drone_id, msg.data), self.now())

    def phase_callback(self, drone_id, msg):
        if math.isfinite(msg.data):
            self.phases[drone_id] = (msg.data, self.now())

    def control_callback(self, msg):
        if not self.running or msg.data != 1:
            self.last_tick = self.now()
        self.running = msg.data == 1
        if msg.data == 2:
            self.agent.reset()
            self.states.clear()
            self.phases.clear()
        if not self.running:
            self.cmd_pub.publish(Float64MultiArray(data=[0., 0.]))

    def tick(self):
        now = self.now()
        dt = now - self.last_tick
        self.last_tick = now
        own = self.states.get(self.drone_id)
        if self.running and own is not None and 0 < dt <= self.timeout and now - own[1] <= self.timeout:
            states = {d: s for d, (s, t) in self.states.items() if now - t <= self.timeout}
            phases = {d: p for d, (p, t) in self.phases.items() if now - t <= self.timeout}
            # Missing neighbors are omitted locally; no global all-agent barrier.
            ctrl = self.agent.step(own[0], states, phases, dt)
            self.cmd_pub.publish(Float64MultiArray(data=ctrl.to_flat()))
        else:
            self.cmd_pub.publish(Float64MultiArray(data=[0., 0.]))
        self.phase_pub.publish(Float64(data=self.agent.phase))


class FormationLifecycle(Node):
    """Start/reset/status only. Does not subscribe to state or compute commands."""
    def __init__(self):
        super().__init__('formation_lifecycle')
        self.declare_parameter('auto_start_delay', 2.)
        self.delay = float(self.get_parameter('auto_start_delay').value)
        self.start_at = self.get_clock().now().nanoseconds * 1e-9 + self.delay
        self.control = self.create_publisher(Int32, '/sim/control', 10)
        self.status = self.create_publisher(String, '/sim/algorithm_status', 10)
        self.create_service(Empty, '/reset_simulation', self.reset)
        self.create_timer(.5, self.tick)

    def reset(self, request, response):
        self.control.publish(Int32(data=2))
        self.start_at = self.get_clock().now().nanoseconds * 1e-9 + self.delay
        return response

    def tick(self):
        self.status.publish(String(data='KuramotoFormation'))
        if self.get_clock().now().nanoseconds * 1e-9 >= self.start_at:
            self.control.publish(Int32(data=1))


def _spin(cls, args):
    rclpy.init(args=args)
    node = cls()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main(args=None):
    _spin(KuramotoNode, args)


def lifecycle_main(args=None):
    _spin(FormationLifecycle, args)
