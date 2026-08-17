"""Hardware Crazyflie node -- single drone.

Replaces drone_node.py for real hardware. Instead of simulating physics
it commands a real Crazyflie drone via Crazyswarm2.

Receives acceleration commands from algorithm_manager, integrates them
into position/velocity setpoints, and streams cmdFullState to the drone.
Real position comes from the mocap_state_node (not this node).

Topics:
  Subscribes: /<drone_id>/cmd_accel     (Float64MultiArray [ax, ay])
  Subscribes: /<drone_id>/cmd_pos       (Float64MultiArray [x, y, vx, vy])
                                         -- optional explicit setpoint
  Subscribes: /<drone_id>/state         (Float64MultiArray [x, y, vx, vy])
                                         -- from mocap_state_node
  Subscribes: /poses                    (NamedPoseArray, for mocap yaw)
  Subscribes: /sim/control              (Int32: 0=stop, 1=start, 2=reset)
  Publishes:  /<drone_id>/setpoint      (Float64MultiArray [x, y, vx, vy, z])
  Publishes:  /<drone_id>/flight_status (String)

Usage:
  ros2 launch drone_testbed hardware_single.launch.py
"""

import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float64MultiArray, Int32, String

from motion_capture_tracking_interfaces.msg import NamedPoseArray

from crazyflie_py import Crazyswarm


TAKEOFF_HEIGHT = 1.0   # metres
TAKEOFF_DURATION = 4.0  # seconds
LAND_DURATION = 4.0
CONTROL_RATE = 25       # Hz -- how often we send cmdFullState

# How stale an explicit setpoint may be before we stop trusting it and fall
# back to integrating cmd_accel. Algorithms publish at their control rate
# (10Hz for Square), so this tolerates several missed messages.
SETPOINT_TIMEOUT = 0.5  # seconds

# How long the drone's mocap state may go missing before we abort. Losing mocap
# is not a benign stall: crazyflie_server stops feeding external position to the
# firmware, the onboard Kalman filter dead-reckons on IMU alone, and its estimate
# drifts away from reality within a second or two. The controller then chases a
# setpoint using a position it is increasingly wrong about, which is what tumbles
# the drone. Landing blind is far better than flying blind.
MOCAP_TIMEOUT = 0.5  # seconds

# Extra distance the drone itself may be outside the geofence before aborting.
# The setpoint is clamped at the fence, so normal tracking error puts the drone
# slightly past it; this is the margin that separates "tracking lag" from "no
# longer under control".
GEOFENCE_BREACH_MARGIN = 0.3  # metres


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
        self._real_pos_time = 0.0
        self._running = False

        # Set when something makes continued flight unsafe. fly() breaks out of
        # its loop and main() lands, rather than hovering on regardless.
        self._abort_reason = None

        # Low-level setpoint streaming state. Streaming cmdFullState preempts
        # the onboard high-level commander, so we stay on high-level hover
        # until the algorithm first commands motion, and we must hand control
        # back (notifySetpointsStop) before any takeoff/land call can work.
        self._streaming = False
        self._mocap_yaw = 0.0   # latest yaw from mocap; streamed as yaw-hold
        self._yaw_hold = 0.0

        # Explicit setpoint from a trajectory-following algorithm, if it offers
        # one. Preferred over integrating cmd_accel, which reconstructs the
        # intended path and accumulates bias with nothing to correct it.
        self._cmd_setpoint = None    # ([x, y], [vx, vy])
        self._setpoint_time = 0.0

        # Telemetry flags, refreshed each control tick purely so the live
        # visualizer can show *why* the setpoint moved the way it did. A
        # deviation that comes from the leash or the geofence intervening is a
        # different problem from one where the drone simply is not tracking.
        self._leashed = False
        self._fenced = False
        self._setpoint_source = 'none'
        # Until takeoff seeds _desired_pos it is still zeros, which would draw a
        # phantom setpoint at the origin. Only publish once it means something.
        self._setpoint_valid = False

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
            Float64MultiArray,
            f'/{self._drone_id}/cmd_pos',
            self._setpoint_callback,
            10,
        )
        self.create_subscription(
            Int32, '/sim/control', self._control_callback, 10,
        )

        # Telemetry for the live visualizer. This is the setpoint we actually
        # stream to the firmware -- after the leash, geofence and speed clamp
        # have had their say -- which is the only honest thing to compare the
        # drone's real position against. /cmd_pos is the algorithm's intent and
        # may differ; the gap between the two is itself worth seeing.
        self._setpoint_pub = self.create_publisher(
            Float64MultiArray, f'/{self._drone_id}/setpoint', 10,
        )
        self._status_pub = self.create_publisher(
            String, f'/{self._drone_id}/flight_status', 10,
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
            self._real_pos_time = self.get_clock().now().nanoseconds * 1e-9

    def _setpoint_callback(self, msg: Float64MultiArray):
        if len(msg.data) >= 4:
            self._cmd_setpoint = (
                np.array(msg.data[0:2]), np.array(msg.data[2:4]),
            )
            self._setpoint_time = self.get_clock().now().nanoseconds * 1e-9

    def _check_safe_to_continue(self) -> bool:
        """Abort if the drone itself is lost or out of bounds.

        Clamping the setpoint is not enough on its own: it stops us *asking* for
        something out of bounds, but the drone keeps flying either way. These
        are the conditions where continuing does more damage than landing.
        """
        if self._abort_reason is not None:
            return False

        now = self.get_clock().now().nanoseconds * 1e-9
        if self._real_pos is None or (now - self._real_pos_time) > MOCAP_TIMEOUT:
            self._abort_reason = 'lost mocap tracking'
        else:
            limit = self._geofence + GEOFENCE_BREACH_MARGIN
            if np.any(np.abs(self._real_pos[:2]) > limit):
                self._abort_reason = (
                    f'drone left the geofence at '
                    f'({self._real_pos[0]:+.2f}, {self._real_pos[1]:+.2f})'
                )

        if self._abort_reason is None:
            return True

        self.get_logger().error(f'ABORT: {self._abort_reason} -- landing')
        self._running = False
        self._cmd_accel = np.zeros(2)
        if self._streaming:
            # Hand control back so the land() in main() is not silently ignored.
            self._streaming = False
            self._cf.notifySetpointsStop()
        return False

    def _setpoint_is_fresh(self) -> bool:
        if self._cmd_setpoint is None:
            return False
        now = self.get_clock().now().nanoseconds * 1e-9
        return (now - self._setpoint_time) < SETPOINT_TIMEOUT

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

    def _flight_status(self) -> str:
        """One-line description of what this node is doing right now."""
        if self._abort_reason is not None:
            return f'ABORT: {self._abort_reason}'
        if not self._running:
            return 'idle'
        if not self._streaming:
            return 'hover (high-level)'

        status = f'streaming ({self._setpoint_source})'
        if self._leashed:
            status += ' [leashed]'
        if self._fenced:
            status += ' [geofence]'
        return status

    def _publish_telemetry(self):
        """Publish the commanded setpoint and status for the live visualizer.

        Called after the setpoint update on every tick, including the ticks that
        return early, so the GUI keeps showing the commanded hover point while
        the algorithm is quiet rather than freezing on the last streamed value --
        and so the value it shows is the one just streamed, not the tick before.
        """
        if self._setpoint_valid:
            msg = Float64MultiArray()
            msg.data = [
                float(self._desired_pos[0]), float(self._desired_pos[1]),
                float(self._desired_vel[0]), float(self._desired_vel[1]),
                float(self._desired_pos[2]),
            ]
            self._setpoint_pub.publish(msg)

        status = String()
        status.data = self._flight_status()
        self._status_pub.publish(status)

    def _control_loop(self):
        self._update_setpoint()
        self._publish_telemetry()

    def _update_setpoint(self):
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

        # Only meaningful once we are the ones flying the drone.
        if not self._check_safe_to_continue():
            return

        dt = 1.0 / CONTROL_RATE
        acc3 = np.array([self._cmd_accel[0], self._cmd_accel[1], 0.0])

        if self._setpoint_is_fresh():
            # The algorithm knows exactly where the drone should be, so use
            # that. Integrating cmd_accel instead reconstructs the same path
            # from its correction signal, and since nothing pulls the result
            # back toward the true target, any bias compounds lap after lap.
            target_pos, target_vel = self._cmd_setpoint
            self._desired_pos[:2] = target_pos
            self._desired_vel[:2] = target_vel
            self._setpoint_source = 'explicit'
        else:
            # Reactive algorithm (or the setpoint went stale): all we have is a
            # push direction, so integrate it into a path.
            self._desired_vel += acc3 * dt
            self._desired_pos += self._desired_vel * dt
            self._setpoint_source = 'integrated'

        # Clamp velocity
        xy_speed = np.linalg.norm(self._desired_vel[:2])
        if xy_speed > self._max_vel:
            self._desired_vel[:2] *= self._max_vel / xy_speed

        # Keep z fixed at takeoff height
        self._desired_pos[2] = TAKEOFF_HEIGHT

        # Leash the setpoint to the drone. This is the anti-windup: if the drone
        # is not following -- still on the ground, thrust-limited, snagged --
        # refuse to let the setpoint run away from it, and bleed off the
        # velocity that accumulated while it was falling behind.
        self._leashed = False
        if self._real_pos is not None:
            lead = self._desired_pos[:2] - self._real_pos[:2]
            distance = float(np.linalg.norm(lead))
            if distance > self._max_lead:
                self._leashed = True
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
        self._fenced = False
        for axis in (0, 1):
            limit = math.copysign(self._geofence, self._desired_pos[axis])
            if abs(self._desired_pos[axis]) > self._geofence:
                self._fenced = True
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
        self._setpoint_valid = True
        self.get_logger().info('Takeoff complete')

    def fly(self):
        """Spin until the flight duration elapses, or a safety check aborts.

        Always spin_once rather than rclpy.spin() even when flying until Ctrl+C:
        spin() never returns, so an abort raised in the control callback would
        have no way to bring the drone down.
        """
        untimed = self._flight_duration <= 0.0
        if untimed:
            self.get_logger().info('Flying until Ctrl+C...')
            end = float('inf')
        else:
            self.get_logger().info(f'Flying for {self._flight_duration:.1f}s...')
            end = self._time_helper.time() + self._flight_duration

        while rclpy.ok() and self._time_helper.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self._abort_reason is not None:
                self.get_logger().error('Flight aborted, landing now')
                return

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
