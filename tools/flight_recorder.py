#!/usr/bin/env python3
"""Record everything a flight post-mortem needs, into one plain text table.

Every diagnosis on this testbed so far has been inference from x/y at 10Hz,
because that is all the algorithm logs. Two signals that would settle far more
have never been recorded:

  the drone's OWN estimate  (/<cf>/pose, already published at 10Hz by
                             firmware_logging in crazyflies.yaml)
  orientation               (in /poses, dropped by mocap_state_node)

The first is the important one. Everything the drone does is driven by where it
*believes* it is. If that belief tracks VICON through an excursion, then the
drone flew where it meant to and the fault is in what we commanded. If it
diverges, the drone was flying blind and the fault is upstream in the estimator
or the mocap feed. Those are opposite problems and nothing recorded so far can
tell them apart.

Output is one fixed-rate row per sample holding the latest value of every
stream, so it loads with a single numpy.loadtxt and needs no ROS to analyse.
Missing streams are written as nan rather than dropping the row.

Usage (terminal 3, alongside the normal two):
    python3 tools/flight_recorder.py
    python3 tools/flight_recorder.py --drone-id drone1 --cf-name drone_1 \
        --out ~/flight_$(date +%H%M%S).txt
"""

import argparse
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float64MultiArray, String
from geometry_msgs.msg import PoseStamped

from motion_capture_tracking_interfaces.msg import NamedPoseArray


NAN = float('nan')

COLUMNS = [
    't',
    'vic_x', 'vic_y', 'vic_z', 'vic_roll', 'vic_pitch', 'vic_yaw',
    'est_x', 'est_y', 'est_z', 'est_yaw',
    'st_x', 'st_y', 'st_vx', 'st_vy',
    'cmd_x', 'cmd_y',
    'set_x', 'set_y',
]


def rpy_deg(q):
    """Roll, pitch, yaw in degrees from a quaternion."""
    roll = math.atan2(2.0 * (q.w * q.x + q.y * q.z),
                      1.0 - 2.0 * (q.x * q.x + q.y * q.y))
    sp = max(-1.0, min(1.0, 2.0 * (q.w * q.y - q.z * q.x)))
    yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                     1.0 - 2.0 * (q.y * q.y + q.z * q.z))
    return math.degrees(roll), math.degrees(math.asin(sp)), math.degrees(yaw)


class FlightRecorder(Node):

    def __init__(self, drone_id, cf_name, out_path, rate):
        super().__init__('flight_recorder')
        self._cf_name = cf_name
        self._v = {c: NAN for c in COLUMNS}
        self._status = 'none'
        # Which streams ever produced a message. Reported at exit, because a
        # topic that never connected is the failure most likely to waste a lab
        # session -- far better to see it in the first second than afterwards.
        self._seen = {'poses': 0, 'pose(onboard)': 0, 'state': 0,
                      'cmd_pos': 0, 'setpoint': 0, 'status': 0}
        self._t0 = None

        self._f = open(out_path, 'w', buffering=1)   # line buffered: Ctrl+C safe
        self._f.write(
            '# Flight recorder. Positions m, angles deg, time s from first sample.\n'
            f'# vic_* = VICON ({cf_name}).  est_* = the drone\'s OWN estimate.\n'
            '# st_* = /state as the algorithm sees it.  cmd_* = algorithm intent.\n'
            '# set_* = setpoint actually streamed.  Missing = nan.\n'
            '# ' + ' '.join(COLUMNS) + ' status\n'
        )
        self._out_path = out_path

        self.create_subscription(NamedPoseArray, '/poses',
                                 self._poses_cb, qos_profile_sensor_data)
        self.create_subscription(PoseStamped, f'/{cf_name}/pose',
                                 self._onboard_cb, 10)
        self.create_subscription(Float64MultiArray, f'/{drone_id}/state',
                                 self._state_cb, 10)
        self.create_subscription(Float64MultiArray, f'/{drone_id}/cmd_pos',
                                 self._cmd_cb, 10)
        self.create_subscription(Float64MultiArray, f'/{drone_id}/setpoint',
                                 self._set_cb, 10)
        self.create_subscription(String, f'/{drone_id}/flight_status',
                                 self._status_cb, 10)

        self.create_timer(1.0 / rate, self._sample)
        self.create_timer(2.0, self._report)

        print(f'Recording to {out_path} at {rate}Hz. Ctrl+C to stop.')
        print(f'  VICON body "{cf_name}", onboard estimate /{cf_name}/pose, '
              f'algorithm topics /{drone_id}/*\n')

    def _now(self):
        t = self.get_clock().now().nanoseconds * 1e-9
        if self._t0 is None:
            self._t0 = t
        return t - self._t0

    def _poses_cb(self, msg):
        for np_ in msg.poses:
            if np_.name != self._cf_name:
                continue
            self._seen['poses'] += 1
            p = np_.pose.position
            r, pi, ya = rpy_deg(np_.pose.orientation)
            self._v.update(vic_x=p.x, vic_y=p.y, vic_z=p.z,
                           vic_roll=r, vic_pitch=pi, vic_yaw=ya)
            return

    def _onboard_cb(self, msg):
        self._seen['pose(onboard)'] += 1
        p = msg.pose.position
        _, _, ya = rpy_deg(msg.pose.orientation)
        self._v.update(est_x=p.x, est_y=p.y, est_z=p.z, est_yaw=ya)

    def _state_cb(self, msg):
        if len(msg.data) >= 4:
            self._seen['state'] += 1
            self._v.update(st_x=msg.data[0], st_y=msg.data[1],
                           st_vx=msg.data[2], st_vy=msg.data[3])

    def _cmd_cb(self, msg):
        if len(msg.data) >= 2:
            self._seen['cmd_pos'] += 1
            self._v.update(cmd_x=msg.data[0], cmd_y=msg.data[1])

    def _set_cb(self, msg):
        if len(msg.data) >= 2:
            self._seen['setpoint'] += 1
            self._v.update(set_x=msg.data[0], set_y=msg.data[1])

    def _status_cb(self, msg):
        self._seen['status'] += 1
        self._status = msg.data.replace(' ', '_') or 'none'

    def _sample(self):
        self._v['t'] = self._now()
        self._f.write(
            ' '.join(f'{self._v[c]:+.4f}' for c in COLUMNS)
            + f' {self._status}\n'
        )

    def _report(self):
        missing = [k for k, n in self._seen.items() if n == 0]
        if missing:
            print(f'  waiting on: {", ".join(missing)}')

    def close(self):
        self._f.close()
        print(f'\nWrote {self._out_path}')
        for k, n in self._seen.items():
            mark = 'OK ' if n else '-- '
            print(f'  {mark}{k:16s} {n} messages')
        if not self._seen['pose(onboard)']:
            print('\n  /%s/pose never arrived -- that is the drone\'s own estimate,\n'
                  '  the most useful signal here. Check the topic name with\n'
                  '  `ros2 topic list | grep pose` while the stack is running.'
                  % self._cf_name)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--drone-id', default='drone1', help='algorithm-side id')
    ap.add_argument('--cf-name', default='drone_1', help='VICON / crazyswarm name')
    ap.add_argument('--out', default='flight_record.txt')
    ap.add_argument('--rate', type=float, default=50.0)
    a = ap.parse_args()

    rclpy.init()
    node = FlightRecorder(a.drone_id, a.cf_name, a.out, a.rate)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
