#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Vector3
from haply_msgs.msg import HaplyControl, Inverse3State

import threading
import time
from collections import deque
import matplotlib  # don't import pyplot here; do it in the main thread


class HaplyAutoControlPublisher(Node):
    """ROS2 Node that alternates between two predefined positions and plots one coordinate (1=x / 2=y / 3=z) over time."""

    def __init__(self):
        super().__init__('spring_damper_test')

        # --- Parameters ---
        self.declare_parameter("interval", 2.0)
        self.declare_parameter("plot_window", 30.0)
        self.declare_parameter("axis", 1)  # 1=x, 2=y, 3=z

        self.interval = float(self.get_parameter("interval").value)
        self.plot_window = float(self.get_parameter("plot_window").value)
        axis_param = int(self.get_parameter("axis").value)

        # Map numeric axis to string + accessor
        if axis_param == 1:
            self.axis = "x"
            self.extract_fn = lambda pos: float(pos.x)
        elif axis_param == 2:
            self.axis = "y"
            self.extract_fn = lambda pos: float(pos.y)
        elif axis_param == 3:
            self.axis = "z"
            self.extract_fn = lambda pos: float(pos.z)
        else:
            self.get_logger().warn(f"Invalid axis '{axis_param}', defaulting to 1 (x)")
            self.axis = "x"
            self.extract_fn = lambda pos: float(pos.x)

        # --- Pub/Sub ---
        self.publisher = self.create_publisher(HaplyControl, 'haply_target', 10)
        self.create_subscription(Inverse3State, 'inverse3_state', self.inverse_state_callback, 10)

        # --- Predefined positions (alternating) ---
        self.positions = [
            Point(x=-0.08, y=-0.15, z=-0.03),
            Point(x= 0.23, y=-0.16, z=0.15),
        ]
        self.current_index = 0

        # Reference lines along the chosen axis
        self.ref_lines = [self.extract_fn(p) for p in self.positions]

        # Timer to publish targets
        self.timer = self.create_timer(self.interval, self.publish_next_position)

        # Buffers for plotting
        self.t0 = time.time()
        self.lock = threading.Lock()
        self.t_buf = deque()
        self.val_buf = deque()

        self.get_logger().info(
            f"Initialized. Alternating every {self.interval:.1f}s. "
            f"Plotting '{self.axis}(t)' with ref lines {self.ref_lines}, window={self.plot_window:.1f}s."
        )

    # ------------------ ROS logic ------------------

    def publish_next_position(self):
        """Publish the next predefined target position."""
        target_position = self.positions[self.current_index]

        msg = HaplyControl()
        msg.use_position = True
        msg.target_position = target_position
        msg.force = Vector3(x=0.0, y=0.0, z=0.0)

        self.publisher.publish(msg)
        self.get_logger().info(
            f"Published target: x={target_position.x:.3f}, "
            f"y={target_position.y:.3f}, z={target_position.z:.3f}"
        )

        self.current_index = (self.current_index + 1) % len(self.positions)

    def inverse_state_callback(self, msg: Inverse3State):
        """Collect chosen axis value over time for plotting."""
        t = time.time() - self.t0
        val = self.extract_fn(msg.position)

        with self.lock:
            self.t_buf.append(t)
            self.val_buf.append(val)
            # Drop old samples
            t_min = t - self.plot_window
            while self.t_buf and self.t_buf[0] < t_min:
                self.t_buf.popleft()
                self.val_buf.popleft()

    # ------------------ Plotting ------------------

    def run_plot(self):
        """Run the live plotting loop in the main thread."""
        try:
            matplotlib.use("TkAgg")
            import matplotlib.pyplot as plt

            plt.ion()
            fig, ax = plt.subplots()

            (line_meas,) = ax.plot([], [], label=f"{self.axis}(t)")
            ref_lines = []
            for r in self.ref_lines:
                (ref_line,) = ax.plot([], [], linestyle="--", label=f"{self.axis}_ref={r}")
                ref_lines.append((ref_line, r))

            ax.set_xlabel("Time [s]")
            ax.set_ylabel(f"{self.axis} position")
            ax.set_title(f"Inverse3 {self.axis}(t)")
            ax.legend(loc="best")
            ax.grid(True)

            while rclpy.ok():
                with self.lock:
                    t_dat = list(self.t_buf)
                    v_dat = list(self.val_buf)

                if t_dat:
                    t_now = t_dat[-1]
                    ax.set_xlim(t_now - self.plot_window, t_now)

                    line_meas.set_data(t_dat, v_dat)

                    y_vals = v_dat + [r for _, r in ref_lines]
                    if y_vals:
                        y_min = min(y_vals)
                        y_max = max(y_vals)
                        pad = 0.05 * max(1.0, (y_max - y_min))
                        ax.set_ylim(y_min - pad, y_max + pad)

                    x0, x1 = ax.get_xlim()
                    for ref_line, r in ref_lines:
                        ref_line.set_data([x0, x1], [r, r])

                plt.pause(0.05)

            plt.ioff()
            plt.show(block=False)
        except Exception as e:
            self.get_logger().error(f"Plotting error: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = HaplyAutoControlPublisher()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        node.run_plot()
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        spin_thread.join()


if __name__ == "__main__":
    main()
