"""Simulation visualizer -- matplotlib 2D top-down view.

Subscribes to all drone states and displays them in real-time.
Shows arena boundary, drone markers with ID labels, velocity arrows,
and position trails.

Topics:
  Subscribes: /<drone_id>/state           (Float64MultiArray [x, y, vx, vy])
  Subscribes: /sim/algorithm_status       (String)
"""

import threading
import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
from collections import deque

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String

from drone_testbed.utils.config_loader import load_config


# Colors for up to 10 drones
DRONE_COLORS = [
    '#e74c3c', '#3498db', '#2ecc71', '#f39c12', '#9b59b6',
    '#1abc9c', '#e67e22', '#34495e', '#e91e63', '#00bcd4',
]

TRAIL_LENGTH = 50


class SimVisualizerNode(Node):

    def __init__(self):
        super().__init__('sim_visualizer')

        self.declare_parameter('config_file', 'config/testbed.yaml')
        config_path = self.get_parameter('config_file').value
        config = load_config(config_path)

        self._bounds = config['simulation'].get('bounds', 2.0)
        self._drone_ids = [d['id'] for d in config['drones']]

        # State storage (thread-safe)
        self._lock = threading.Lock()
        self._positions = {d: np.zeros(2) for d in self._drone_ids}
        self._velocities = {d: np.zeros(2) for d in self._drone_ids}
        self._trails = {d: deque(maxlen=TRAIL_LENGTH) for d in self._drone_ids}
        self._algorithm_name = config.get('algorithm', {}).get('name', '?')

        # Subscribe to drone states
        for drone_id in self._drone_ids:
            self.create_subscription(
                Float64MultiArray,
                f'/{drone_id}/state',
                lambda msg, did=drone_id: self._state_callback(did, msg),
                10,
            )

        # Subscribe to algorithm status
        self.create_subscription(
            String, '/sim/algorithm_status',
            self._algo_status_callback, 10,
        )

        # Set up matplotlib figure
        self._fig, self._ax = plt.subplots(1, 1, figsize=(8, 8))
        self._setup_plot()

        # Update timer (~3Hz via ROS, but actual plot update uses matplotlib timer)
        self._timer = self._fig.canvas.new_timer(interval=300)
        self._timer.add_callback(self._update_plot)
        self._timer.start()

        self.get_logger().info(
            f'Visualizer started for {len(self._drone_ids)} drones'
        )

    def _state_callback(self, drone_id: str, msg: Float64MultiArray):
        if len(msg.data) >= 4:
            with self._lock:
                self._positions[drone_id] = np.array(msg.data[0:2])
                self._velocities[drone_id] = np.array(msg.data[2:4])
                self._trails[drone_id].append(msg.data[0:2])

    def _algo_status_callback(self, msg: String):
        with self._lock:
            self._algorithm_name = msg.data

    def _setup_plot(self):
        ax = self._ax
        b = self._bounds
        ax.set_xlim(-b - 0.5, b + 0.5)
        ax.set_ylim(-b - 0.5, b + 0.5)
        ax.set_aspect('equal')
        ax.grid(True, alpha=0.3)
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')

        # Arena boundary
        arena = Circle((0, 0), b, fill=False, linestyle='--',
                        color='gray', linewidth=2)
        ax.add_patch(arena)

    def _update_plot(self):
        ax = self._ax

        with self._lock:
            positions = {k: v.copy() for k, v in self._positions.items()}
            velocities = {k: v.copy() for k, v in self._velocities.items()}
            trails = {k: list(v) for k, v in self._trails.items()}
            algo_name = self._algorithm_name

        ax.cla()
        self._setup_plot()

        ax.set_title(f'Multi-Drone Testbed  |  Algorithm: {algo_name}', fontsize=14)

        for i, drone_id in enumerate(self._drone_ids):
            color = DRONE_COLORS[i % len(DRONE_COLORS)]
            pos = positions[drone_id]
            vel = velocities[drone_id]

            # Draw trail
            trail = trails[drone_id]
            if len(trail) > 1:
                trail_arr = np.array(trail)
                alphas = np.linspace(0.1, 0.5, len(trail_arr))
                for j in range(len(trail_arr) - 1):
                    ax.plot(
                        trail_arr[j:j + 2, 0], trail_arr[j:j + 2, 1],
                        color=color, alpha=alphas[j], linewidth=1.5,
                    )

            # Draw drone marker
            ax.plot(pos[0], pos[1], 'o', color=color, markersize=12,
                    markeredgecolor='black', markeredgewidth=1)

            # Draw ID label
            ax.annotate(
                drone_id, (pos[0], pos[1]),
                textcoords='offset points', xytext=(10, 10),
                fontsize=9, color=color, fontweight='bold',
            )

            # Draw velocity arrow
            speed = np.linalg.norm(vel)
            if speed > 0.01:
                ax.arrow(
                    pos[0], pos[1], vel[0] * 0.5, vel[1] * 0.5,
                    head_width=0.05, head_length=0.03,
                    fc=color, ec=color, alpha=0.7,
                )

        self._fig.canvas.draw_idle()
        self._fig.canvas.flush_events()


def main(args=None):
    rclpy.init(args=args)
    node = SimVisualizerNode()

    # Run ROS spinning in a background thread so matplotlib can use the main thread
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
