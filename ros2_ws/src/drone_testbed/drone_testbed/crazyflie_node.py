"""Hardware Crazyflie node -- single drone.

Replaces drone_node.py for real hardware. Instead of simulating physics
it commands a real Crazyflie drone via Crazyswarm2.

Receives acceleration commands from algorithm_manager, integrates them
into position/velocity setpoints, and streams cmdFullState to the drone.
Real position comes from the mocap_state_node (not this node).

Topics:
  Subscribes: /<drone_id>/cmd_accel   (Float64MultiArray [ax, ay])
  Subscribes: /<drone_id>/state       (Float64MultiArray [x, y, vx, vy])
                                       -- from mocap_state_node
  Subscribes: /sim/control            (Int32: 0=stop, 1=start, 2=reset)

Usage:
  ros2 launch drone_testbed hardware_single.launch.py
"""

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Int32

from crazyflie_py import Crazyswarm


TAKEOFF_HEIGHT = 0.5   # metres
TAKEOFF_DURATION = 3.0  # seconds
LAND_DURATION = 3.0
CONTROL_RATE = 25       # Hz -- how often we send cmdFullState


class CrazyflieNode(Node):

    def __init__(self, swarm):
        super().__init__('crazyflie_node')

        self.declare_parameter('drone_id', 'drone1')
        self.declare_parameter('cf_name', '/cf1')   # Crazyswarm robot name
        self.declare_parameter('max_velocity', 0.7)
        self.declare_parameter('max_acceleration', 0.5)

        self._drone_id = self.get_parameter('drone_id').value
        cf_name = self.get_parameter('cf_name').value
        self._max_vel = self.get_parameter('max_velocity').value
        self._max_acc = self.get_parameter('max_acceleration').value

        # Get the specific Crazyflie object from swarm
        self._cf = swarm.allcfs.crazyflies_by_name[cf_name]
        self._time_helper = swarm.timeHelper

        # Desired state (integrated from acceleration commands)
        self._desired_pos = np.zeros(3)   # [x, y, z]
        self._desired_vel = np.zeros(3)   # [vx, vy, vz]
        self._cmd_accel = np.zeros(2)     # [ax, ay] from algorithm

        # Real position from mocap (used to initialise desired_pos)
        self._real_pos = None
        self._running = False

        # Subscribers
        self.create_subscription(
            Float64MultiArray,
            f'/{self._drone_id}/cmd_accel',
            self._accel_callback,
            10,
        )
        self.create_subscription(
            Float64MultiArray,
            f'/{self._drone_id}/state',
            self._state_callback,
            10,
        )
        self.create_subscription(
            Int32, '/sim/control', self._control_callback, 10,
        )

        # Control timer
        self.create_timer(1.0 / CONTROL_RATE, self._control_loop)

        self.get_logger().info(
            f'Crazyflie node ready: {cf_name} → /{self._drone_id}'
        )

    def _accel_callback(self, msg: Float64MultiArray):
        if len(msg.data) >= 2:
            self._cmd_accel = np.clip(
                np.array(msg.data[:2]),
                -self._max_acc,
                self._max_acc,
            )

    def _state_callback(self, msg: Float64MultiArray):
        if len(msg.data) >= 2:
            self._real_pos = np.array([msg.data[0], msg.data[1], TAKEOFF_HEIGHT])

    def _control_callback(self, msg: Int32):
        if msg.data == 0:
            self._running = False
            self._cmd_accel = np.zeros(2)
        elif msg.data == 1:
            self._running = True
            # Seed desired pos from real mocap position
            if self._real_pos is not None:
                self._desired_pos = self._real_pos.copy()
        elif msg.data == 2:
            self._running = False
            self._cmd_accel = np.zeros(2)

    def _control_loop(self):
        if not self._running:
            return

        dt = 1.0 / CONTROL_RATE

        # Integrate acceleration into desired velocity and position
        acc3 = np.array([self._cmd_accel[0], self._cmd_accel[1], 0.0])
        self._desired_vel += acc3 * dt

        # Clamp velocity
        xy_speed = np.linalg.norm(self._desired_vel[:2])
        if xy_speed > self._max_vel:
            self._desired_vel[:2] *= self._max_vel / xy_speed

        self._desired_pos += self._desired_vel * dt

        # Keep z fixed at takeoff height
        self._desired_pos[2] = TAKEOFF_HEIGHT

        # Send full state command to Crazyflie
        self._cf.cmdFullState(
            pos=self._desired_pos,
            vel=self._desired_vel,
            acc=acc3,
            yaw=0.0,
            omega=np.zeros(3),
        )

    def takeoff(self):
        self.get_logger().info(f'Taking off to {TAKEOFF_HEIGHT}m...')
        self._cf.takeoff(targetHeight=TAKEOFF_HEIGHT, duration=TAKEOFF_DURATION)
        self._time_helper.sleep(TAKEOFF_DURATION + 0.5)

        # Seed desired pos at current real position after takeoff
        if self._real_pos is not None:
            self._desired_pos = self._real_pos.copy()
        self._desired_pos[2] = TAKEOFF_HEIGHT
        self.get_logger().info('Takeoff complete')

    def land(self):
        self.get_logger().info('Landing...')
        self._running = False
        self._cf.land(targetHeight=0.05, duration=LAND_DURATION)
        self._time_helper.sleep(LAND_DURATION + 0.5)
        self.get_logger().info('Landed')


def main(args=None):
    rclpy.init(args=args)

    swarm = Crazyswarm()

    node = CrazyflieNode(swarm)

    try:
        node.takeoff()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.land()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
