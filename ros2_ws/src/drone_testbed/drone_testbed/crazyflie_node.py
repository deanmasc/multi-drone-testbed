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

import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float64MultiArray, Int32

from motion_capture_tracking_interfaces.msg import NamedPoseArray

from crazyflie_py import Crazyswarm


TAKEOFF_HEIGHT = 1.0   # metres
TAKEOFF_DURATION = 4.0  # seconds
LAND_DURATION = 3.0
CONTROL_RATE = 25       # Hz -- how often we send cmdFullState


class CrazyflieNode(Node):

    def __init__(self, swarm):
        super().__init__('crazyflie_node')

        self.declare_parameter('drone_id', 'drone1')
        self.declare_parameter('cf_name', '/cf1')   # Crazyswarm robot name
        self.declare_parameter('max_velocity', 0.7)
        self.declare_parameter('max_acceleration', 0.5)
        # Seconds to fly after takeoff before landing. 0 = until Ctrl+C.
        self.declare_parameter('flight_duration', 0.0)
        # Must track the `tracking` mode in crazyflies.yaml. True for "vendor"
        # tracking of a multi-marker rigid body, where mocap orientation is real
        # and the firmware already knows its true heading -- hold the measured
        # yaw. False for single-marker/position-only setups, where the /poses
        # quaternion is a placeholder and the drone's own estimate starts at 0.
        self.declare_parameter('use_mocap_yaw', True)
        # Half-extent of the allowed x/y box around the world origin. The
        # streamed setpoint is an open integral of algorithm output, so without
        # this nothing stops it walking out of the capture volume -- and once
        # the drone leaves tracking, position feedback is gone entirely.
        self.declare_parameter('geofence', 1.5)
        # How far the streamed setpoint may get ahead of the drone's actual
        # position before we stop advancing it. Without this the setpoint is an
        # open integral: if the drone cannot follow -- still on the ground,
        # thrust-limited, snagged -- the tracking error grows, the algorithm
        # saturates its output, and the setpoint accelerates away on its own.
        self.declare_parameter('max_lead', 0.3)

        self._drone_id = self.get_parameter('drone_id').value
        cf_name = self.get_parameter('cf_name').value
        self._cf_name = cf_name
        self._max_vel = self.get_parameter('max_velocity').value
        self._max_acc = self.get_parameter('max_acceleration').value
        self._flight_duration = self.get_parameter('flight_duration').value
        self._use_mocap_yaw = self.get_parameter('use_mocap_yaw').value
        self._geofence = self.get_parameter('geofence').value
        self._max_lead = self.get_parameter('max_lead').value

        # Get the specific Crazyflie object from swarm
        self._cf = swarm.allcfs.crazyfliesByName[cf_name]
        self._time_helper = swarm.timeHelper

        # Desired state (integrated from acceleration commands)
        self._desired_pos = np.zeros(3)   # [x, y, z]
        self._desired_vel = np.zeros(3)   # [vx, vy, vz]
        self._cmd_accel = np.zeros(2)     # [ax, ay] from algorithm

        # Real position from mocap (used to initialise desired_pos)
        self._real_pos = None
        self._running = False

        # Low-level setpoint streaming state. Streaming cmdFullState preempts
        # the onboard high-level commander, so we stay on high-level hover
        # until the algorithm first commands motion, and we must hand control
        # back (notifySetpointsStop) before any takeoff/land call can work.
        self._streaming = False
        self._mocap_yaw = 0.0   # latest yaw from mocap; streamed as yaw-hold
        self._yaw_hold = 0.0

        self.create_subscription(
            NamedPoseArray,
            '/poses',
            self._poses_callback,
            qos_profile_sensor_data,
        )

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

    def _poses_callback(self, msg: NamedPoseArray):
        if not self._use_mocap_yaw:
            return  # single-marker tracking: no real orientation to read
        for named_pose in msg.poses:
            if named_pose.name != self._cf_name:
                continue
            q = named_pose.pose.orientation
            self._mocap_yaw = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z),
            )
            return

    def _control_callback(self, msg: Int32):
        if msg.data == 0:
            self._running = False
            self._cmd_accel = np.zeros(2)
            self._stop_streaming()
        elif msg.data == 1:
            self._running = True
        elif msg.data == 2:
            self._running = False
            self._cmd_accel = np.zeros(2)
            self._stop_streaming()

    def _stop_streaming(self):
        """Hand control back to the onboard high-level commander."""
        if not self._streaming:
            return
        self._streaming = False
        self._cf.notifySetpointsStop()
        # Brief takeoff re-engages the high-level hover at the current height
        # (same trick as the reference pursuit-evasion implementation).
        self._cf.takeoff(targetHeight=TAKEOFF_HEIGHT, duration=0.5)
        self.get_logger().info('Streaming stopped, back on high-level hover')

    def _control_loop(self):
        if not self._running:
            return

        # Stay on the onboard high-level hover until the algorithm actually
        # commands motion. Once cmdFullState streaming starts it preempts the
        # high-level commander and must be continuous, so we only engage on
        # the first nonzero acceleration -- and seed the setpoint from the
        # drone's real position and yaw to avoid a jump at engagement.
        if not self._streaming:
            if not np.any(self._cmd_accel):
                return
            if self._real_pos is None:
                return  # don't engage until mocap tells us where the drone is
            self._desired_pos = self._real_pos.copy()
            self._desired_pos[2] = TAKEOFF_HEIGHT
            self._desired_vel = np.zeros(3)
            self._yaw_hold = self._mocap_yaw
            self._streaming = True
            self.get_logger().info(
                f'Engaging setpoint streaming at {self._desired_pos.round(3)}, '
                f'yaw-hold {math.degrees(self._yaw_hold):.0f} deg'
            )

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

        # Leash the setpoint to the drone. This is the anti-windup: if the drone
        # is not following -- still on the ground, thrust-limited, snagged --
        # refuse to let the setpoint run away from it, and bleed off the
        # velocity that accumulated while it was falling behind.
        if self._real_pos is not None:
            lead = self._desired_pos[:2] - self._real_pos[:2]
            distance = float(np.linalg.norm(lead))
            if distance > self._max_lead:
                self._desired_pos[:2] = (
                    self._real_pos[:2] + lead * (self._max_lead / distance)
                )
                self._desired_vel[:2] *= 0.5
                self.get_logger().warn(
                    f'Setpoint leashed: drone is {distance:.2f}m behind it '
                    f'(limit {self._max_lead}m) -- is it actually flying?',
                    throttle_duration_sec=1.0,
                )

        # Geofence x/y. Zero the outward velocity component as well as clamping
        # the position, otherwise the integrator keeps winding up against the
        # limit and the drone lurches when it is finally free to move again.
        for axis in (0, 1):
            limit = math.copysign(self._geofence, self._desired_pos[axis])
            if abs(self._desired_pos[axis]) > self._geofence:
                self._desired_pos[axis] = limit
                if self._desired_vel[axis] * limit > 0:
                    self._desired_vel[axis] = 0.0
                self.get_logger().warn(
                    f'Geofence hit on {"xy"[axis]} at {self._geofence}m',
                    throttle_duration_sec=1.0,
                )

        # Send full state command to Crazyflie. Yaw holds the value captured at
        # engagement: the measured mocap yaw when orientation is real, otherwise
        # 0 to match the drone's own gyro-initialised estimate. Never mix the
        # two -- commanding 0 while mocap reports a nonzero yaw snap-turns it.
        self._cf.cmdFullState(
            pos=self._desired_pos,
            vel=self._desired_vel,
            acc=acc3,
            yaw=self._yaw_hold,
            omega=np.zeros(3),
        )

    def takeoff(self):
        # Hand control back to the high-level commander before we ask for
        # takeoff. If an earlier run died mid-stream (Ctrl+C, crash, unhandled
        # exception) the firmware is still in low-level setpoint mode and will
        # silently ignore takeoff() until the drone is power-cycled. Nothing in
        # this process can leave it that way when the algorithm never commands
        # motion, but a previous run that did stream will contaminate every
        # later run, so clear it unconditionally at startup.
        self._cf.notifySetpointsStop()

        self.get_logger().info(f'Taking off to {TAKEOFF_HEIGHT}m...')
        self._cf.takeoff(targetHeight=TAKEOFF_HEIGHT, duration=TAKEOFF_DURATION)
        self._time_helper.sleep(TAKEOFF_DURATION + 0.5)

        # Seed desired pos at current real position after takeoff
        if self._real_pos is not None:
            self._desired_pos = self._real_pos.copy()
        self._desired_pos[2] = TAKEOFF_HEIGHT
        self.get_logger().info('Takeoff complete')

    def fly(self):
        """Spin until the flight duration elapses (0 = until interrupted)."""
        if self._flight_duration <= 0.0:
            self.get_logger().info('Flying until Ctrl+C...')
            rclpy.spin(self)
            return

        self.get_logger().info(f'Flying for {self._flight_duration:.1f}s...')
        end = self._time_helper.time() + self._flight_duration
        while rclpy.ok() and self._time_helper.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def land(self):
        self.get_logger().info('Landing...')
        self._running = False
        # If we were streaming setpoints, the high-level commander is
        # preempted and would silently ignore land() -- hand control back first.
        if self._streaming:
            self._streaming = False
            self._cf.notifySetpointsStop()
        self._cf.land(targetHeight=0.05, duration=LAND_DURATION)
        self._time_helper.sleep(LAND_DURATION + 0.5)
        self.get_logger().info('Landed')


def main(args=None):
    swarm = Crazyswarm()

    node = CrazyflieNode(swarm)

    try:
        node.takeoff()
        node.fly()
    except KeyboardInterrupt:
        pass
    finally:
        node.land()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
