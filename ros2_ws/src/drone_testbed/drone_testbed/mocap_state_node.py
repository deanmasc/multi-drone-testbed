"""Motion capture state node.

Reads drone position from the VICON mocap system and publishes it
as a drone state topic compatible with the algorithm manager.

Velocity is estimated by a least-squares fit over a short sliding window
rather than by differencing consecutive samples. That matters more than it
sounds. A raw first difference amplifies position noise by sqrt(2)/dt, which
at 100Hz is a factor of ~141: sub-millimetre VICON jitter -- excellent
tracking -- becomes tens of millimetres per second of velocity noise. Any
algorithm with a velocity gain then multiplies that again. TrochoidalConsensus
at beta=6 turned 0.5mm of jitter into 0.42 m/s^2 of commanded acceleration,
against a trajectory whose peak demand was 0.385 -- the noise was larger than
the signal, and the real drone visibly juddered.

Fitting a line to the last N samples instead reduces the noise by roughly
sqrt(12/(N(N^2-1)))/dt, about 13x better at N=10, in exchange for a lag of
(N-1)/2 samples. Fitting against the actual timestamps (rather than assuming a
fixed dt) also makes it immune to the jitter and dropped frames that made the
old wall-clock division occasionally produce enormous spikes.

Topics:
  Subscribes: /poses                  (NamedPoseArray from VICON)
  Publishes:  /<drone_id>/state       (Float64MultiArray [x, y, vx, vy])
"""

import math
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float64MultiArray

from motion_capture_tracking_interfaces.msg import NamedPoseArray

import numpy as np


class MocapStateNode(Node):

    def __init__(self):
        super().__init__('mocap_state_node')

        self.declare_parameter('drone_id', 'drone1')
        self.declare_parameter('mocap_name', 'cf1')  # name as it appears in VICON
        # Samples in the velocity fit. Larger is smoother but laggier: the
        # estimate is effectively centred (window-1)/2 samples in the past, so
        # at 100Hz a window of 10 costs about 45ms of phase. Below 3 this
        # degenerates to a plain difference and the noise comes back.
        self.declare_parameter('velocity_window', 10)
        # Samples older than this are dropped from the fit, so a tracking
        # dropout cannot leave the window straddling a gap and report a
        # velocity averaged across the missing stretch.
        self.declare_parameter('velocity_max_age', 0.25)  # seconds

        self._drone_id = self.get_parameter('drone_id').value
        self._mocap_name = self.get_parameter('mocap_name').value
        self._window = max(2, int(self.get_parameter('velocity_window').value))
        self._max_age = float(self.get_parameter('velocity_max_age').value)

        self._samples = deque(maxlen=self._window)   # (t, x, y)

        self._state_pub = self.create_publisher(
            Float64MultiArray,
            f'/{self._drone_id}/state',
            10,
        )

        self.create_subscription(
            NamedPoseArray,
            '/poses',
            self._poses_callback,
            qos_profile_sensor_data,
        )

        # Noise gain relative to a raw first difference, so the log says what
        # this window is actually buying.
        n = self._window
        gain = math.sqrt(12.0 / (n * (n * n - 1))) / math.sqrt(2.0)
        self.get_logger().info(
            f'Mocap state node: tracking "{self._mocap_name}" → '
            f'/{self._drone_id}/state'
        )
        self.get_logger().info(
            f'Velocity: least-squares over {n} samples '
            f'({gain:.3f}x the noise of a raw difference, '
            f'~{(n - 1) / 2:.1f} samples of lag)'
        )

    def _velocity(self):
        """Least-squares slope of position against time over the window.

        Solving against the real timestamps rather than a nominal dt means
        irregular arrival -- which is normal, and was previously able to divide
        by a near-zero interval -- just weights the fit slightly differently
        instead of producing a spike.
        """
        if len(self._samples) < 2:
            return np.zeros(2)

        newest = self._samples[-1][0]
        pts = [s for s in self._samples if newest - s[0] <= self._max_age]
        if len(pts) < 2:
            return np.zeros(2)

        t = np.array([p[0] for p in pts])
        t = t - t.mean()
        spread = float(t @ t)
        if spread < 1e-12:      # every sample carries the same timestamp
            return np.zeros(2)

        xy = np.array([[p[1], p[2]] for p in pts])
        return (t @ (xy - xy.mean(axis=0))) / spread

    def _poses_callback(self, msg: NamedPoseArray):
        now = self.get_clock().now().nanoseconds * 1e-9

        for named_pose in msg.poses:
            if named_pose.name != self._mocap_name:
                continue

            pos = np.array([
                named_pose.pose.position.x,
                named_pose.pose.position.y,
            ])

            self._samples.append((now, pos[0], pos[1]))
            vel = self._velocity()

            state_msg = Float64MultiArray()
            state_msg.data = [pos[0], pos[1], vel[0], vel[1]]
            self._state_pub.publish(state_msg)
            return


def main(args=None):
    rclpy.init(args=args)
    node = MocapStateNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
