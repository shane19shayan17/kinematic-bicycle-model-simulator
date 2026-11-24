"""ROS 2 node that simulates a point using a kinematic bicycle model."""

import threading

import numpy as np
import rclpy
from rclpy.node import Node

from sim_interfaces.msg import CmdVel, VehicleState

from .bicycle_model import BicycleModel, VehicleState as ModelState
from .visualizer import SimulatorVisualizer


class SimulatorNode(Node):
    """ROS 2 interface, simulation timer, noise model, and visualization."""

    def __init__(self):
        """Declare configuration, create the model, and set up ROS interfaces."""
        super().__init__("sim")
        self._declare_parameters()
        self._read_parameters()
        self._validate_parameters()

        self.rng = (
            np.random.default_rng(self.random_seed)
            if self.random_seed >= 0 else np.random.default_rng()
        )

        initial_state = ModelState(
            x=self.initial_x,
            y=self.initial_y,
            heading=self.initial_heading,
        )
        self.model = BicycleModel(self.wheelbase, initial_state)

        # The ROS executor and GUI use separate threads. Protect shared state.
        self.state_lock = threading.Lock()
        self.command_velocity = 0.0
        self.command_steering = 0.0

        self.command_subscription = self.create_subscription(
            CmdVel, "/cmd_vel", self.command_callback, 10
        )
        self.sensor_publisher = self.create_publisher(
            VehicleState, "/sensors", 10
        )
        self.actuator_publisher = self.create_publisher(
            VehicleState, "/actuator_feedback", 10
        )

        self.dt = 1.0 / self.frequency
        self.timer = self.create_timer(self.dt, self.simulation_callback)

        self.visualizer = SimulatorVisualizer(
            background_color=self.background_color,
            point_radius=self.point_radius,
            point_color=self.point_color,
            x_axis_scale=self.x_axis_scale,
            y_axis_scale=self.y_axis_scale,
            footprint_color=self.footprint_color,
            footprint_line_width=self.footprint_line_width,
            max_points=self.footprint_max_points,
        )
        self.visualizer.update_state(
            self.initial_x, self.initial_y, self.initial_heading
        )

        self.get_logger().info(
            f"Simulator started at {self.frequency:.1f} Hz; "
            "subscribing to /cmd_vel."
        )

    def _declare_parameters(self):
        """Declare parameters so YAML and ROS command-line overrides work."""
        defaults = {
            "background_color": "white",
            "point_radius": 0.01,
            "point_color": "red",
            "x_axis_scale": 10.0,
            "y_axis_scale": 10.0,
            "footprint_color": "royalblue",
            "footprint_line_width": 1.5,
            "footprint_max_points": 20000,
            "simulator_frequency": 30.0,
            "wheelbase": 0.32,
            "max_steering_angle": 0.60,
            "initial_x": 0.0,
            "initial_y": 0.0,
            "initial_heading": 0.0,
            "state_noise_x_std": 0.01,
            "state_noise_y_std": 0.01,
            "state_noise_heading_std": 0.01,
            "actuator_noise_percent": 0.0,
            "random_seed": -1,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def _read_parameters(self):
        """Read ROS parameters into named Python attributes."""
        get = lambda name: self.get_parameter(name).value

        self.background_color = get("background_color")
        self.point_radius = float(get("point_radius"))
        self.point_color = get("point_color")
        self.x_axis_scale = float(get("x_axis_scale"))
        self.y_axis_scale = float(get("y_axis_scale"))
        self.footprint_color = get("footprint_color")
        self.footprint_line_width = float(get("footprint_line_width"))
        self.footprint_max_points = int(get("footprint_max_points"))

        self.frequency = float(get("simulator_frequency"))
        self.wheelbase = float(get("wheelbase"))
        self.max_steering_angle = float(get("max_steering_angle"))

        self.initial_x = float(get("initial_x"))
        self.initial_y = float(get("initial_y"))
        self.initial_heading = float(get("initial_heading"))

        self.state_noise_x_std = float(get("state_noise_x_std"))
        self.state_noise_y_std = float(get("state_noise_y_std"))
        self.state_noise_heading_std = float(get("state_noise_heading_std"))
        self.actuator_noise_percent = float(get("actuator_noise_percent"))
        self.random_seed = int(get("random_seed"))

    def _validate_parameters(self):
        """Reject invalid physical, display, and noise parameters."""
        positive_values = {
            "simulator_frequency": self.frequency,
            "wheelbase": self.wheelbase,
            "max_steering_angle": self.max_steering_angle,
            "point_radius": self.point_radius,
            "x_axis_scale": self.x_axis_scale,
            "y_axis_scale": self.y_axis_scale,
        }
        for name, value in positive_values.items():
            if value <= 0.0:
                raise ValueError(f"{name} must be greater than zero")

        noise_values = (
            self.state_noise_x_std,
            self.state_noise_y_std,
            self.state_noise_heading_std,
            self.actuator_noise_percent,
        )
        if any(value < 0.0 for value in noise_values):
            raise ValueError("Noise parameters cannot be negative")
        if self.footprint_max_points < 2:
            raise ValueError("footprint_max_points must be at least 2")

    def command_callback(self, message: CmdVel):
        """Store the newest velocity and steering command."""
        with self.state_lock:
            self.command_velocity = float(message.linear_velocity)
            self.command_steering = float(message.steering_angle)

    def _apply_actuator_noise(self, command: float) -> float:
        """Apply multiplicative Gaussian noise with configured relative sigma."""
        if command == 0.0 or self.actuator_noise_percent == 0.0:
            return command
        relative_sigma = self.actuator_noise_percent / 100.0
        return command * (1.0 + self.rng.normal(0.0, relative_sigma))

    def _make_state_message(
        self, x: float, y: float, heading: float,
        velocity: float, steering: float,
    ) -> VehicleState:
        """Create a timestamped VehicleState message in the map frame."""
        message = VehicleState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "map"
        message.x = float(x)
        message.y = float(y)
        message.heading = float(heading)
        message.linear_velocity = float(velocity)
        message.steering_angle = float(steering)
        return message

    def simulation_callback(self):
        """Apply actuator noise, integrate, publish outputs, and update the GUI."""
        with self.state_lock:
            applied_velocity = self._apply_actuator_noise(self.command_velocity)
            applied_steering = self._apply_actuator_noise(self.command_steering)

            # Enforce the steering actuator's mechanical angle limit.
            applied_steering = float(np.clip(
                applied_steering,
                -self.max_steering_angle,
                self.max_steering_angle,
            ))

            state = self.model.step(
                velocity=applied_velocity,
                steering_angle=applied_steering,
                dt=self.dt,
            )
            true_x, true_y, true_heading = state.x, state.y, state.heading

        # Noise affects the sensor estimate only, never the internal true state.
        estimated_x = true_x + self.rng.normal(0.0, self.state_noise_x_std)
        estimated_y = true_y + self.rng.normal(0.0, self.state_noise_y_std)
        estimated_heading = BicycleModel.wrap_angle(
            true_heading + self.rng.normal(0.0, self.state_noise_heading_std)
        )

        self.sensor_publisher.publish(self._make_state_message(
            estimated_x, estimated_y, estimated_heading,
            applied_velocity, applied_steering,
        ))
        self.actuator_publisher.publish(self._make_state_message(
            true_x, true_y, true_heading,
            applied_velocity, applied_steering,
        ))

        # Only the GUI reads this pose; it does not modify model state.
        self.visualizer.update_state(true_x, true_y, true_heading)


def main(args=None):
    """Run ROS callbacks in a worker thread and the Matplotlib GUI on main."""
    rclpy.init(args=args)
    node = None
    spin_thread = None

    try:
        node = SimulatorNode()
        spin_thread = threading.Thread(
            target=rclpy.spin, args=(node,), daemon=True
        )
        spin_thread.start()

        # GUI frameworks generally require their event loop on the main thread.
        node.visualizer.run(interval_ms=1000.0 / node.frequency)

    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if spin_thread is not None and spin_thread.is_alive():
            spin_thread.join(timeout=1.0)


if __name__ == "__main__":
    main()
