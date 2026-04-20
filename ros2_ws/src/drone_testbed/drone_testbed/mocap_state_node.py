"""Motion capture state node.

Reads drone position from the VICON mocap system and publishes it
as a drone state topic compatible with the algorithm manager.

Velocity is estimated by finite differencing consecutive positions.

Topics:
  Subscribes: /poses                  (NamedPoseArray from VICON)
  Publishes:  /<drone_id>/state       (Float64MultiArray [x, y, vx, vy])
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

from motion_capture_tracking_interfaces.msg import NamedPoseArray

import numpy as np


class MocapStateNode(Node):

    def __init__(self):
        super().__init__('mocap_state_node')

        self.declare_parameter('drone_id', 'drone1')
        self.declare_parameter('mocap_name', 'cf1')  # name as it appears in VICON

        self._drone_id = self.get_parameter('drone_id').value
        self._mocap_name = self.get_parameter('mocap_name').value

        self._prev_pos = None
        self._prev_time = None

        self._state_pub = self.create_publisher(
            Float64MultiArray,
            f'/{self._drone_id}/state',
            10,
        )

        self.create_subscription(
            NamedPoseArray,
            '/poses',
            self._poses_callback,
            10,
        )

        self.get_logger().info(
            f'Mocap state node: tracking "{self._mocap_name}" → /{self._drone_id}/state'
        )

    def _poses_callback(self, msg: NamedPoseArray):
        now = self.get_clock().now().nanoseconds * 1e-9

        for named_pose in msg.poses:
            if named_pose.name != self._mocap_name:
                continue

            pos = np.array([
                named_pose.pose.position.x,
                named_pose.pose.position.y,
            ])

            # Estimate velocity by finite differencing
            if self._prev_pos is not None and self._prev_time is not None:
                dt = now - self._prev_time
                if dt > 0:
                    vel = (pos - self._prev_pos) / dt
                else:
                    vel = np.zeros(2)
            else:
                vel = np.zeros(2)

            self._prev_pos = pos
            self._prev_time = now

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
