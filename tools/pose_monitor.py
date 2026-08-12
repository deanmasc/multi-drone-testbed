#!/usr/bin/env python3
"""Watch a VICON rigid body on /poses and flag anything a controller would hate.

The flight logs written by the algorithms only carry x/y, because that is all
mocap_state_node forwards. Orientation never reaches them -- which is a problem,
because a rigid body whose orientation flips mid-flight makes the firmware apply
every position correction in a rotated frame, and the drone spirals away doing
exactly what it was told. This records what those logs cannot see.

Flags three things:
  YAW JUMP  orientation resolved to a different solution between frames, which
            usually means the marker pattern is too symmetric to disambiguate
  POS JUMP  the body was misidentified, or reacquired after a dropout
  DROPOUT   tracking lost; the firmware dead-reckons on IMU and drifts

Run it with nothing flying to check the rigid body, and again during a flight to
see what the pose was doing at the moment things went wrong.

Usage (after sourcing ROS, with crazyflie launch.py already running):
    python3 tools/pose_monitor.py
    python3 tools/pose_monitor.py --name drone_2 | tee /tmp/pose.log
"""

import argparse
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from motion_capture_tracking_interfaces.msg import NamedPoseArray


# Levels at which the pose stopped describing a physically possible movement.
YAW_JUMP_DEG = 20.0     # deg between consecutive samples
POS_JUMP_M = 0.10       # metres between consecutive samples
DROPOUT_S = 0.15        # seconds with no sample for this body
PRINT_HZ = 5.0          # steady-state print rate; anomalies always print


def rpy_from_quaternion(q):
    """Roll, pitch, yaw in degrees."""
    roll = math.atan2(2.0 * (q.w * q.x + q.y * q.z),
                      1.0 - 2.0 * (q.x * q.x + q.y * q.y))
    sin_pitch = max(-1.0, min(1.0, 2.0 * (q.w * q.y - q.z * q.x)))
    pitch = math.asin(sin_pitch)
    yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                     1.0 - 2.0 * (q.y * q.y + q.z * q.z))
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


def angle_diff_deg(a: float, b: float) -> float:
    """Smallest signed difference a-b, wrapped to [-180, 180]."""
    return (a - b + 180.0) % 360.0 - 180.0


class PoseMonitor(Node):

    def __init__(self, name: str):
        super().__init__('pose_monitor')
        self._name = name
        self._prev = None
        self._last_print = 0.0
        self._samples = 0
        self._anomalies = 0
        self._worst_yaw_jump = 0.0
        self._seen = False

        self.create_subscription(
            NamedPoseArray, '/poses', self._callback, qos_profile_sensor_data,
        )
        self.create_timer(1.0, self._check_alive)

        print(f'Watching "{name}" on /poses. Ctrl+C to stop.\n')
        print(f'{"time":>8}  {"x":>7} {"y":>7} {"z":>7}  '
              f'{"roll":>6} {"pitch":>6} {"yaw":>7}   notes')

    def _check_alive(self):
        if not self._seen:
            print(f'  ...no pose named "{self._name}" yet -- check the object '
                  f'exists in Tracker, is enabled, and its markers are visible')

    def _callback(self, msg: NamedPoseArray):
        now = self.get_clock().now().nanoseconds * 1e-9

        for named_pose in msg.poses:
            if named_pose.name != self._name:
                continue

            self._seen = True
            self._samples += 1
            p = named_pose.pose.position
            roll, pitch, yaw = rpy_from_quaternion(named_pose.pose.orientation)

            notes = []
            if self._prev is not None:
                pt, px, py, pz, pyaw = self._prev
                dt = now - pt
                dyaw = angle_diff_deg(yaw, pyaw)
                dpos = math.dist((p.x, p.y, p.z), (px, py, pz))

                if dt > DROPOUT_S:
                    notes.append(f'DROPOUT {dt * 1000:.0f}ms')
                if abs(dyaw) > YAW_JUMP_DEG:
                    notes.append(f'YAW JUMP {dyaw:+.0f} deg')
                    self._worst_yaw_jump = max(self._worst_yaw_jump, abs(dyaw))
                if dpos > POS_JUMP_M:
                    notes.append(f'POS JUMP {dpos * 100:.0f}cm')

            self._prev = (now, p.x, p.y, p.z, yaw)

            if notes:
                self._anomalies += 1
            elif now - self._last_print < 1.0 / PRINT_HZ:
                return
            self._last_print = now

            print(f'{now % 1000:8.2f}  {p.x:+7.3f} {p.y:+7.3f} {p.z:+7.3f}  '
                  f'{roll:+6.1f} {pitch:+6.1f} {yaw:+7.1f}   {"  ".join(notes)}')
            return

    def summary(self):
        print(f'\n{self._samples} samples, {self._anomalies} anomalies.')
        if not self._seen:
            print(f'Never saw "{self._name}". Wrong name, or not tracked.')
        elif self._worst_yaw_jump:
            print(f'Worst yaw jump {self._worst_yaw_jump:.0f} deg. Anything near '
                  f'90 or 180 means the marker pattern is ambiguous -- Tracker is '
                  f'resolving it to more than one orientation. Fix the markers '
                  f'before flying again.')
        elif self._anomalies:
            print('Anomalies present -- the pose is not trustworthy enough to fly on.')
        else:
            print('No anomalies over this window.')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--name', default='drone_1',
                        help='rigid body name as published on /poses')
    args = parser.parse_args()

    rclpy.init()
    node = PoseMonitor(args.name)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.summary()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
