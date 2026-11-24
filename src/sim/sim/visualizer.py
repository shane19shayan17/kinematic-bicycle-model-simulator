"""Matplotlib visualization for the simulated point and its footprint."""

from collections import deque
import math

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Circle


class SimulatorVisualizer:
    """Draw the 2D plane, moving point, heading, and historical footprint."""

    def __init__(
        self,
        background_color: str,
        point_radius: float,
        point_color: str,
        x_axis_scale: float,
        y_axis_scale: float,
        footprint_color: str,
        footprint_line_width: float,
        max_points: int,
    ):
        """
        Initialize the figure and persistent drawing objects.

        Axis scales are positive half-ranges: x is [-x_scale, x_scale],
        and y is [-y_scale, y_scale].
        """
        self.trajectory_x = deque(maxlen=max_points)
        self.trajectory_y = deque(maxlen=max_points)

        self.figure, self.axes = plt.subplots()
        self.figure.canvas.manager.set_window_title("ROS 2 2D Bicycle Simulator")
        self.axes.set_facecolor(background_color)
        self.figure.patch.set_facecolor(background_color)

        self.axes.set_xlim(-x_axis_scale, x_axis_scale)
        self.axes.set_ylim(-y_axis_scale, y_axis_scale)
        self.axes.set_aspect("equal", adjustable="box")
        self.axes.set_xlabel("X position [m]")
        self.axes.set_ylabel("Y position [m]")
        self.axes.set_title("Kinematic Bicycle Model")
        self.axes.grid(True, alpha=0.35)
        self.axes.axhline(0.0, color="black", linewidth=0.8, alpha=0.6)
        self.axes.axvline(0.0, color="black", linewidth=0.8, alpha=0.6)

        self.footprint_line, = self.axes.plot(
            [], [], color=footprint_color,
            linewidth=footprint_line_width, label="Footprint",
        )
        self.point = Circle(
            (0.0, 0.0), radius=point_radius, color=point_color, zorder=5
        )
        self.axes.add_patch(self.point)
        self.heading_line, = self.axes.plot([], [], color="black", linewidth=1.4)
        self.axes.legend(loc="upper right")

        self.latest_state = (0.0, 0.0, 0.0)
        self.animation = None

    def update_state(self, x: float, y: float, heading: float) -> None:
        """Receive the latest pose from the ROS thread."""
        self.latest_state = (x, y, heading)

    def _draw_frame(self, _frame):
        """Update Matplotlib artists; called by the GUI animation timer."""
        x, y, heading = self.latest_state
        self.trajectory_x.append(x)
        self.trajectory_y.append(y)
        self.footprint_line.set_data(self.trajectory_x, self.trajectory_y)
        self.point.center = (x, y)

        arrow_length = max(
            self.point.get_radius() * 5.0,
            min(
                self.axes.get_xlim()[1] - self.axes.get_xlim()[0],
                self.axes.get_ylim()[1] - self.axes.get_ylim()[0],
            ) * 0.025,
        )
        self.heading_line.set_data(
            [x, x + arrow_length * math.cos(heading)],
            [y, y + arrow_length * math.sin(heading)],
        )
        return self.footprint_line, self.point, self.heading_line

    def run(self, interval_ms: float) -> None:
        """Open the interactive Matplotlib window and run its event loop."""
        self.animation = FuncAnimation(
            self.figure,
            self._draw_frame,
            interval=interval_ms,
            blit=False,
            cache_frame_data=False,
        )
        plt.show()
