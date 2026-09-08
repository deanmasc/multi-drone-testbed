"""Live flight visualizer -- real-time 2D view of commanded vs actual position.

The simulation visualizer answers "where are the drones?". This one answers
"is the drone going where we told it to?", which is the question that matters
on hardware. It plots three things per drone and the gaps between them:

  actual     -- where the drone really is, from mocap  (/<id>/state)
  commanded  -- the setpoint crazyflie_node streams to the firmware, after the
                leash, geofence and speed clamp have modified it
                (/<id>/setpoint)
  intended   -- the raw setpoint the algorithm asked for, published only by
                trajectory-following algorithms  (/<id>/cmd_pos)

`actual` vs `commanded` is the drone's true tracking error -- the thing the
onboard controller is actually failing or succeeding at. `commanded` vs
`intended` shows the safety layer intervening: when they separate, the leash or
the geofence is rewriting the algorithm's request, which looks like a control
bug from the outside but is not one.

Works in simulation too, where nothing publishes /<id>/setpoint; the commanded
artists simply stay hidden and it degrades to the sim view plus time series.

Topics:
  Subscribes: /<drone_id>/state          (Float64MultiArray [x, y, vx, vy])
  Subscribes: /<drone_id>/setpoint       (Float64MultiArray [x, y, vx, vy, z])
  Subscribes: /<drone_id>/cmd_pos        (Float64MultiArray [x, y, vx, vy])
  Subscribes: /<drone_id>/flight_status  (String)
  Subscribes: /sim/algorithm_status      (String)

Usage:
  ros2 run drone_testbed live_visualizer --ros-args \
      -p config_file:=config/testbed_vicon_test.yaml -p geofence:=1.5
"""

import threading
from collections import deque

import numpy as np

import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String

from drone_testbed.utils.config_loader import load_config


# Colors for up to 10 drones
DRONE_COLORS = [
    '#e74c3c', '#3498db', '#2ecc71', '#f39c12', '#9b59b6',
    '#1abc9c', '#e67e22', '#34495e', '#e91e63', '#00bcd4',
]

# Mocap runs at ~100Hz, so a few thousand samples covers a generous window
# without letting the deques grow without bound over a long flight.
HISTORY_MAXLEN = 6000

# Beyond this the /state topic is treated as dead rather than merely quiet.
# Matches MOCAP_TIMEOUT in crazyflie_node -- the point at which that node
# aborts -- so the GUI goes red at the same moment the drone starts landing.
STALE_TIMEOUT = 0.5  # seconds


class DroneTrace:
    """Rolling history for one drone, plus the artists that draw it."""

    def __init__(self):
        self.actual = deque(maxlen=HISTORY_MAXLEN)     # (t, x, y)
        self.commanded = deque(maxlen=HISTORY_MAXLEN)  # (t, x, y)
        self.intended = deque(maxlen=HISTORY_MAXLEN)   # (t, x, y)
        self.error = deque(maxlen=HISTORY_MAXLEN)      # (t, |actual-reference|)
        self.last_actual_time = None
        self.status = 'waiting...'
        # Which stream the error is measured against -- 'commanded' on hardware,
        # 'intent' in simulation where nothing publishes /setpoint.
        self.error_ref = 'commanded'


class LiveVisualizerNode(Node):

    def __init__(self):
        super().__init__('live_visualizer')

        self.declare_parameter('config_file', 'config/testbed.yaml')
        # Should match the geofence crazyflie_node was launched with, so the box
        # drawn here is the box actually being enforced.
        self.declare_parameter('geofence', 1.5)
        # Likewise the leash: it is the threshold crazyflie_node actually acts
        # on, so it is drawn as the reference line on the error plot. A default
        # baked in here would quietly lie the moment max_lead is changed.
        self.declare_parameter('max_lead', 0.3)
        # Seconds of path to keep on the XY view. The time series keep the full
        # history window instead, which is what you scroll back through.
        self.declare_parameter('trail_seconds', 8.0)
        self.declare_parameter('history_seconds', 40.0)
        self.declare_parameter('plot_rate', 10.0)  # Hz

        config_path = self.get_parameter('config_file').value
        config = load_config(config_path)

        self._geofence = self.get_parameter('geofence').value
        self._max_lead = self.get_parameter('max_lead').value
        self._trail_seconds = self.get_parameter('trail_seconds').value
        self._history_seconds = self.get_parameter('history_seconds').value
        plot_rate = self.get_parameter('plot_rate').value

        self._drone_ids = [d['id'] for d in config['drones']]
        self._algorithm_name = config.get('algorithm', {}).get('name', '?')

        # Everything below is touched from both the ROS executor thread and the
        # matplotlib timer on the main thread.
        self._lock = threading.Lock()
        self._traces = {d: DroneTrace() for d in self._drone_ids}
        self._t0 = None

        for drone_id in self._drone_ids:
            self.create_subscription(
                Float64MultiArray, f'/{drone_id}/state',
                lambda msg, d=drone_id: self._state_cb(d, msg), 10,
            )
            self.create_subscription(
                Float64MultiArray, f'/{drone_id}/setpoint',
                lambda msg, d=drone_id: self._setpoint_cb(d, msg), 10,
            )
            self.create_subscription(
                Float64MultiArray, f'/{drone_id}/cmd_pos',
                lambda msg, d=drone_id: self._cmd_pos_cb(d, msg), 10,
            )
            self.create_subscription(
                String, f'/{drone_id}/flight_status',
                lambda msg, d=drone_id: self._status_cb(d, msg), 10,
            )

        self.create_subscription(
            String, '/sim/algorithm_status', self._algo_status_cb, 10,
        )

        self._build_figure()

        self._timer = self._fig.canvas.new_timer(
            interval=int(1000.0 / plot_rate)
        )
        self._timer.add_callback(self._update_plot)
        self._timer.start()

        self.get_logger().info(
            f'Live visualizer started for {len(self._drone_ids)} drones '
            f'(geofence {self._geofence}m)'
        )

    # ---------------- ROS callbacks ----------------

    def _now(self) -> float:
        """Seconds since the first message of the run."""
        t = self.get_clock().now().nanoseconds * 1e-9
        if self._t0 is None:
            self._t0 = t
        return t - self._t0

    def _state_cb(self, drone_id: str, msg: Float64MultiArray):
        if len(msg.data) < 2:
            return
        with self._lock:
            t = self._now()
            trace = self._traces[drone_id]
            trace.actual.append((t, msg.data[0], msg.data[1]))
            trace.last_actual_time = t
            self._recompute_error(trace, t)

    def _setpoint_cb(self, drone_id: str, msg: Float64MultiArray):
        if len(msg.data) < 2:
            return
        with self._lock:
            t = self._now()
            trace = self._traces[drone_id]
            trace.commanded.append((t, msg.data[0], msg.data[1]))
            self._recompute_error(trace, t)

    def _cmd_pos_cb(self, drone_id: str, msg: Float64MultiArray):
        if len(msg.data) < 2:
            return
        with self._lock:
            self._traces[drone_id].intended.append(
                (self._now(), msg.data[0], msg.data[1])
            )

    def _status_cb(self, drone_id: str, msg: String):
        with self._lock:
            self._traces[drone_id].status = msg.data

    def _algo_status_cb(self, msg: String):
        with self._lock:
            self._algorithm_name = msg.data

    @staticmethod
    def _recompute_error(trace: DroneTrace, t: float):
        """Log tracking error whenever both sides of the comparison exist.

        The two topics arrive at different rates (mocap ~100Hz, setpoint 25Hz),
        so rather than interpolating we just pair each new sample against the
        other stream's latest value. At these rates the timing skew is well
        under the errors being measured.
        """
        if not trace.actual:
            return

        # Prefer the streamed setpoint. Fall back to the algorithm's intent so
        # the error plot still works in simulation, where crazyflie_node is not
        # running and /setpoint is never published.
        if trace.commanded:
            trace.error_ref = 'commanded'
            reference = trace.commanded[-1]
        elif trace.intended:
            trace.error_ref = 'intent'
            reference = trace.intended[-1]
        else:
            return

        _, ax_, ay_ = trace.actual[-1]
        _, cx, cy = reference
        trace.error.append((t, float(np.hypot(ax_ - cx, ay_ - cy))))

    # ---------------- figure setup ----------------

    def _build_figure(self):
        self._fig = plt.figure(figsize=(15, 9))
        self._fig.canvas.manager.set_window_title('Drone Testbed -- Live Flight')
        self._fig.patch.set_facecolor('white')

        gs = self._fig.add_gridspec(
            3, 2, width_ratios=[1.3, 1.0],
            hspace=0.45, wspace=0.2,
            left=0.06, right=0.97, top=0.92, bottom=0.07,
        )
        self._ax_xy = self._fig.add_subplot(gs[:, 0])
        self._ax_err = self._fig.add_subplot(gs[0, 1])
        self._ax_x = self._fig.add_subplot(gs[1, 1])
        self._ax_y = self._fig.add_subplot(gs[2, 1])

        self._setup_xy_axes()
        self._setup_time_axes()

        # Persistent artists, updated in place each frame. Rebuilding the axes
        # every tick (as the sim visualizer does) is fine at 3Hz but visibly
        # stutters at the rate you want for watching a live flight.
        self._artists = {}
        for i, drone_id in enumerate(self._drone_ids):
            color = DRONE_COLORS[i % len(DRONE_COLORS)]
            self._artists[drone_id] = self._make_artists(drone_id, color)

        self._ax_xy.legend(loc='upper right', fontsize=8, framealpha=0.9)

        self._status_text = self._fig.text(
            0.06, 0.015, '', fontsize=9, family='monospace',
            va='bottom', ha='left',
        )
        self._title = self._fig.suptitle('', fontsize=14, fontweight='bold')

    def _setup_xy_axes(self):
        ax = self._ax_xy
        g = self._geofence
        span = g + 0.6
        ax.set_xlim(-span, span)
        ax.set_ylim(-span, span)
        ax.set_aspect('equal')
        ax.grid(True, alpha=0.3)
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_title('Top-down view', fontsize=11)

        # The geofence the setpoint is clamped to, and the extra margin the
        # drone itself may drift into before crazyflie_node aborts.
        ax.add_patch(Rectangle(
            (-g, -g), 2 * g, 2 * g, fill=False,
            linestyle='--', edgecolor='#c0392b', linewidth=1.5,
            label='geofence',
        ))
        margin = g + 0.3  # GEOFENCE_BREACH_MARGIN in crazyflie_node
        ax.add_patch(Rectangle(
            (-margin, -margin), 2 * margin, 2 * margin, fill=False,
            linestyle=':', edgecolor='#c0392b', linewidth=1.0, alpha=0.5,
            label='abort boundary',
        ))
        ax.axhline(0, color='gray', linewidth=0.6, alpha=0.5)
        ax.axvline(0, color='gray', linewidth=0.6, alpha=0.5)

    def _setup_time_axes(self):
        self._ax_err.set_ylabel('|error| (m)')
        self._ax_err.set_title(
            'Tracking error: actual vs commanded', fontsize=11,
        )
        self._ax_x.set_ylabel('X (m)')
        self._ax_x.set_title('X: actual (solid) vs commanded (dashed)', fontsize=10)
        self._ax_y.set_ylabel('Y (m)')
        self._ax_y.set_title('Y: actual (solid) vs commanded (dashed)', fontsize=10)
        self._ax_y.set_xlabel('time (s)')

        for ax in (self._ax_err, self._ax_x, self._ax_y):
            ax.grid(True, alpha=0.3)

        # The leash length is the threshold crazyflie_node acts on, so it is the
        # natural reference line for judging whether an error is significant.
        self._ax_err.axhline(
            self._max_lead, color='#e67e22', linestyle=':', linewidth=1.2,
            label=f'leash (max_lead {self._max_lead}m)',
        )
        self._ax_err.legend(loc='upper left', fontsize=8)

    def _make_artists(self, drone_id: str, color: str) -> dict:
        show_label = drone_id == self._drone_ids[0]

        def lbl(text):
            return text if show_label else None

        a = {}
        # --- XY view ---
        a['actual_trail'], = self._ax_xy.plot(
            [], [], '-', color=color, linewidth=2.0, alpha=0.85,
            label=lbl('actual'),
        )
        a['cmd_trail'], = self._ax_xy.plot(
            [], [], '--', color=color, linewidth=1.3, alpha=0.55,
            label=lbl('commanded'),
        )
        a['intended_trail'], = self._ax_xy.plot(
            [], [], ':', color='#7f8c8d', linewidth=1.2, alpha=0.7,
            label=lbl('algorithm intent'),
        )
        a['deviation'], = self._ax_xy.plot(
            [], [], '-', color='#c0392b', linewidth=1.6, alpha=0.9,
            label=lbl('deviation'),
        )
        a['actual_dot'], = self._ax_xy.plot(
            [], [], 'o', color=color, markersize=13,
            markeredgecolor='black', markeredgewidth=1.2, zorder=5,
        )
        a['cmd_dot'], = self._ax_xy.plot(
            [], [], 'X', color=color, markersize=11, alpha=0.75,
            markeredgecolor='black', markeredgewidth=0.8, zorder=4,
        )
        a['label'] = self._ax_xy.annotate(
            drone_id, (0, 0), textcoords='offset points', xytext=(12, 10),
            fontsize=11, color=color, fontweight='bold',
        )

        # --- time series ---
        a['err_line'], = self._ax_err.plot(
            [], [], '-', color=color, linewidth=1.4,
        )
        a['x_actual'], = self._ax_x.plot([], [], '-', color=color, linewidth=1.4)
        a['x_cmd'], = self._ax_x.plot(
            [], [], '--', color=color, linewidth=1.2, alpha=0.6,
        )
        a['y_actual'], = self._ax_y.plot([], [], '-', color=color, linewidth=1.4)
        a['y_cmd'], = self._ax_y.plot(
            [], [], '--', color=color, linewidth=1.2, alpha=0.6,
        )
        return a

    # ---------------- drawing ----------------

    @staticmethod
    def _window(samples, t_min):
        """Return (t, a, b) arrays for samples newer than t_min."""
        if not samples:
            return None
        arr = np.asarray(samples, dtype=float)
        arr = arr[arr[:, 0] >= t_min]
        if arr.size == 0:
            return None
        return arr

    def _update_plot(self):
        with self._lock:
            traces = {
                d: (
                    list(t.actual), list(t.commanded), list(t.intended),
                    list(t.error), t.last_actual_time, t.status, t.error_ref,
                )
                for d, t in self._traces.items()
            }
            algo_name = self._algorithm_name
            now = 0.0 if self._t0 is None else self._now()

        trail_min = now - self._trail_seconds
        hist_min = now - self._history_seconds
        status_lines = []

        for drone_id in self._drone_ids:
            (actual, commanded, intended, error,
             last_t, status, error_ref) = traces[drone_id]
            a = self._artists[drone_id]

            act_w = self._window(actual, trail_min)
            cmd_w = self._window(commanded, trail_min)
            int_w = self._window(intended, trail_min)

            # --- XY trails and markers ---
            if act_w is not None:
                a['actual_trail'].set_data(act_w[:, 1], act_w[:, 2])
                a['actual_dot'].set_data([act_w[-1, 1]], [act_w[-1, 2]])
                # Move the anchor only. set_position() on an Annotation with
                # textcoords='offset points' rewrites the *offset* rather than
                # the anchor, which would collapse the (12, 10) offset to a
                # fraction of a point and bury the label under the marker.
                a['label'].xy = (act_w[-1, 1], act_w[-1, 2])
            else:
                a['actual_trail'].set_data([], [])
                a['actual_dot'].set_data([], [])

            if cmd_w is not None:
                a['cmd_trail'].set_data(cmd_w[:, 1], cmd_w[:, 2])
                a['cmd_dot'].set_data([cmd_w[-1, 1]], [cmd_w[-1, 2]])
            else:
                a['cmd_trail'].set_data([], [])
                a['cmd_dot'].set_data([], [])

            if int_w is not None:
                a['intended_trail'].set_data(int_w[:, 1], int_w[:, 2])
            else:
                a['intended_trail'].set_data([], [])

            # The line joining where it is to where it was told to be. Seeing
            # the gap itself is more legible than reading two coordinates.
            if act_w is not None and cmd_w is not None:
                a['deviation'].set_data(
                    [act_w[-1, 1], cmd_w[-1, 1]],
                    [act_w[-1, 2], cmd_w[-1, 2]],
                )
            else:
                a['deviation'].set_data([], [])

            # --- time series ---
            err_w = self._window(error, hist_min)
            if err_w is not None:
                a['err_line'].set_data(err_w[:, 0], err_w[:, 1])
            else:
                a['err_line'].set_data([], [])

            act_h = self._window(actual, hist_min)
            cmd_h = self._window(commanded, hist_min)
            if act_h is not None:
                a['x_actual'].set_data(act_h[:, 0], act_h[:, 1])
                a['y_actual'].set_data(act_h[:, 0], act_h[:, 2])
            else:
                a['x_actual'].set_data([], [])
                a['y_actual'].set_data([], [])
            if cmd_h is not None:
                a['x_cmd'].set_data(cmd_h[:, 0], cmd_h[:, 1])
                a['y_cmd'].set_data(cmd_h[:, 0], cmd_h[:, 2])
            else:
                a['x_cmd'].set_data([], [])
                a['y_cmd'].set_data([], [])

            status_lines.append(self._status_line(
                drone_id, act_w, err_w, last_t, now, status, error_ref,
            ))

        # Scroll the time axes; the XY axes are fixed to the geofence.
        t_lo = max(0.0, hist_min)
        t_hi = max(now, t_lo + 1.0)
        for ax in (self._ax_err, self._ax_x, self._ax_y):
            ax.set_xlim(t_lo, t_hi)
        self._ax_err.set_ylim(0, max(0.4, self._max_error(traces, hist_min) * 1.2))
        for ax in (self._ax_x, self._ax_y):
            ax.set_ylim(-self._geofence - 0.3, self._geofence + 0.3)

        self._title.set_text(
            f'Multi-Drone Testbed -- LIVE   |   Algorithm: {algo_name}'
        )
        self._status_text.set_text('\n'.join(status_lines))

        self._fig.canvas.draw_idle()

    @staticmethod
    def _max_error(traces, t_min) -> float:
        peak = 0.0
        for entry in traces.values():
            for t, e in entry[3]:
                if t >= t_min and e > peak:
                    peak = e
        return peak

    def _status_line(
        self, drone_id, act_w, err_w, last_t, now, status, error_ref,
    ) -> str:
        if act_w is None:
            return f'{drone_id}: no mocap data yet'

        pos = f'pos ({act_w[-1, 1]:+.2f}, {act_w[-1, 2]:+.2f})'

        if err_w is not None:
            errs = err_w[:, 1]
            err = (
                f'err {errs[-1]:.3f}m vs {error_ref}  '
                f'(mean {errs.mean():.3f}  peak {errs.max():.3f})'
            )
        else:
            err = 'err --  (no setpoint yet)'

        stale = ''
        if last_t is not None and (now - last_t) > STALE_TIMEOUT:
            stale = f'  !! MOCAP STALE {now - last_t:.1f}s'

        return f'{drone_id}: {pos}  {err}  [{status}]{stale}'


def main(args=None):
    rclpy.init(args=args)
    node = LiveVisualizerNode()

    # matplotlib owns the main thread; ROS spins behind it.
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
