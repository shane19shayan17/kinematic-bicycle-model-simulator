"""Kinematic bicycle model used by the 2D simulator."""

from dataclasses import dataclass
import math


@dataclass
class VehicleState:
    """Ground-truth planar vehicle state and applied actuator values."""

    x: float = 0.0
    y: float = 0.0
    heading: float = 0.0
    linear_velocity: float = 0.0
    steering_angle: float = 0.0


class BicycleModel:
    """Integrate a no-slip kinematic bicycle model with forward Euler."""

    def __init__(self, wheelbase: float, initial_state: VehicleState):
        """
        Create the model.

        Args:
            wheelbase: Distance between front and rear axle, in meters.
            initial_state: Initial ground-truth state.
        """
        if wheelbase <= 0.0:
            raise ValueError("wheelbase must be greater than zero")
        self.wheelbase = float(wheelbase)
        self.state = initial_state

    def step(self, velocity: float, steering_angle: float, dt: float) -> VehicleState:
        """
        Advance the vehicle by one fixed simulation step using forward Euler.

        Args:
            velocity: Applied forward velocity in m/s.
            steering_angle: Applied front-wheel steering angle in radians.
            dt: Simulation time step in seconds.

        Returns:
            Updated ground-truth state.
        """
        if dt <= 0.0:
            raise ValueError("dt must be greater than zero")

        theta = self.state.heading
        x_dot = velocity * math.cos(theta)
        y_dot = velocity * math.sin(theta)
        theta_dot = (velocity / self.wheelbase) * math.tan(steering_angle)

        self.state.x += x_dot * dt
        self.state.y += y_dot * dt
        self.state.heading = self.wrap_angle(theta + theta_dot * dt)
        self.state.linear_velocity = velocity
        self.state.steering_angle = steering_angle
        return self.state

    @staticmethod
    def wrap_angle(angle: float) -> float:
        """Wrap an angle to [-pi, pi)."""
        return (angle + math.pi) % (2.0 * math.pi) - math.pi
