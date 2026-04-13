"""Simulated drone node -- one instance per drone.

Subscribes to acceleration commands, integrates double integrator dynamics,
publishes state. Parameterized by drone_id so the launch file can spawn N
instances from this single file.

Topics:
  Subscribes: /<drone_id>/cmd_accel  (Float64MultiArray [ax, ay])
  Publishes:  /<drone_id>/state      (Float64MultiArray [x, y, vx, vy])
  Subscribes: /sim/control           (Int32: 0=stop, 1=start, 2=reset)
"""

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Int32

from drone_testbed.dynamics import double_integrator


class DroneNode(Node):

    def __init__(self):
        super().__init__('drone_node')

        # Parameters
        self.declare_parameter('drone_id', 'drone1')
        self.declare_parameter('initial_position', [0.0, 0.0])
        self.declare_parameter('initial_velocity', [0.0, 0.0])
        self.declare_parameter('dt', 0.1)
        self.declare_parameter('sim_rate', 10.0)
        self.declare_parameter('bounds', 2.0)
        self.declare_parameter('max_velocity', 0.7)
        self.declare_parameter('max_acceleration', 0.5)

        self._drone_id = self.get_parameter('drone_id').value
        init_pos = self.get_parameter('initial_position').value
        init_vel = self.get_parameter('initial_velocity').value
        self._dt = self.get_parameter('dt').value
        sim_rate = self.get_parameter('sim_rate').value
        self._bounds = self.get_parameter('bounds').value
        self._max_vel = self.get_parameter('max_velocity').value
        self._max_acc = self.get_parameter('max_acceleration').value

        # State: [x, y, vx, vy]
        self._initial_state = np.array([
            init_pos[0], init_pos[1],
            init_vel[0], init_vel[1],
        ])
        self._state = self._initial_state.copy()
        self._cmd_accel = np.zeros(2)
        self._running = False

        # Publisher
        self._state_pub = self.create_publisher(
            Float64MultiArray,
            f'/{self._drone_id}/state',
            10,
        )

        # Subscribers
        self.create_subscription(
            Float64MultiArray,
            f'/{self._drone_id}/cmd_accel',
            self._cmd_callback,
            10,
        )
        self.create_subscription(Int32, '/sim/control', self._control_callback, 10)

        # Simulation timer
        timer_period = 1.0 / sim_rate
        self.create_timer(timer_period, self._sim_step)

        self.get_logger().info(
            f'Drone [{self._drone_id}] initialized at '
            f'pos=({init_pos[0]:.1f}, {init_pos[1]:.1f})'
        )

    def _cmd_callback(self, msg: Float64MultiArray):
        if len(msg.data) >= 2:
            self._cmd_accel = np.clip(
                np.array(msg.data[:2]),
                -self._max_acc,
                self._max_acc,
            )

    def _control_callback(self, msg: Int32):
        if msg.data == 0:
            self._running = False
            self._cmd_accel = np.zeros(2)
        elif msg.data == 1:
            self._running = True
        elif msg.data == 2:
            self._running = False
            self._state = self._initial_state.copy()
            self._cmd_accel = np.zeros(2)

    def _sim_step(self):
        # Publish state even when stopped (so visualizer can see initial positions)
        msg = Float64MultiArray()
        msg.data = self._state.tolist()
        self._state_pub.publish(msg)

        if not self._running:
            return

        # Integrate dynamics
        self._state = double_integrator.step(self._state, self._cmd_accel, self._dt)

        # Clamp velocity
        vx, vy = self._state[2], self._state[3]
        speed = np.sqrt(vx ** 2 + vy ** 2)
        if speed > self._max_vel:
            scale = self._max_vel / speed
            self._state[2] *= scale
            self._state[3] *= scale

        # Clamp position to arena bounds
        self._state[0] = np.clip(self._state[0], -self._bounds, self._bounds)
        self._state[1] = np.clip(self._state[1], -self._bounds, self._bounds)


def main(args=None):
    rclpy.init(args=args)
    node = DroneNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
