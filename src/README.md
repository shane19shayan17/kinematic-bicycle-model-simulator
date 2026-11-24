# ROS 2 2D Kinematic Bicycle Simulator

A small ROS 2 workspace containing:
- `sim_interfaces`: custom messages shared by the simulator and other nodes.
- `sim`: Python ROS 2 node named `sim`, Matplotlib 2D view, YAML config, and launch file.

Designed for ROS 2 Humble / Ubuntu 22.04 and Jazzy / Ubuntu 24.04.

## Model

State: `x [m]`, `y [m]`, `heading [rad]`.
Inputs: `linear_velocity [m/s]`, `steering_angle [rad]`.
Positive steering means a left turn.

```text
xdot       = v * cos(heading)
ydot       = v * sin(heading)
heading_dot = v / wheelbase * tan(steering_angle)
```

Forward Euler integration uses a fixed `dt = 1 / simulator_frequency`.

The ground-truth state is internal. `/sensors` publishes a noisy pose;
`/actuator_feedback` publishes ground-truth pose and the applied actuator
values. Sensor noise values are Gaussian standard deviations in meters/radians.
`actuator_noise_percent` is the Gaussian standard deviation relative to the
magnitude of each nonzero command. For example, 10 means sigma is 10% of the
command. Under this multiplicative model, zero commands remain zero.

## Packages and topics

| Package | Purpose |
|---|---|
| `sim_interfaces` | `CmdVel` and `VehicleState` custom messages |
| `sim` | Python simulator node and visualization |

| Topic | Type | Meaning |
|---|---|---|
| `/cmd_vel` | `sim_interfaces/msg/CmdVel` | Commanded speed and steering |
| `/sensors` | `sim_interfaces/msg/VehicleState` | Noisy estimated pose |
| `/actuator_feedback` | `sim_interfaces/msg/VehicleState` | Ground-truth pose and applied commands |

`CmdVel.msg`:
```text
float64 linear_velocity
float64 steering_angle
```

`VehicleState.msg`:
```text
std_msgs/Header header
float64 x
float64 y
float64 heading
float64 linear_velocity
float64 steering_angle
```

## Build

```bash
mkdir -p ~/bicycle_ws/src
# Extract/copy sim_interfaces/ and sim/ into ~/bicycle_ws/src/
source /opt/ros/$ROS_DISTRO/setup.bash
cd ~/bicycle_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

Install Matplotlib if needed:

```bash
sudo apt install python3-matplotlib
```

## Run

```bash
ros2 launch sim sim.launch.py
```

A Matplotlib window opens. In another terminal:

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
source ~/bicycle_ws/install/setup.bash

# Straight motion
ros2 topic pub -r 10 /cmd_vel sim_interfaces/msg/CmdVel \
"{linear_velocity: 1.0, steering_angle: 0.0}"

# Curved motion
ros2 topic pub -r 10 /cmd_vel sim_interfaces/msg/CmdVel \
"{linear_velocity: 1.0, steering_angle: 0.35}"

# Stop
ros2 topic pub --once /cmd_vel sim_interfaces/msg/CmdVel \
"{linear_velocity: 0.0, steering_angle: 0.0}"
```

Inspect:

```bash
ros2 topic echo /sensors
ros2 topic echo /actuator_feedback
ros2 topic info /cmd_vel
ros2 interface show sim_interfaces/msg/CmdVel
```

The last command remains active if its publisher stops. Publish zero velocity
to stop the simulated point.

## YAML configuration

Edit `sim/config/sim.yaml` and relaunch.

- `background_color`: Matplotlib color, default white.
- `point_radius`: point radius in meters, default 0.01.
- `point_color`: Matplotlib color, default red.
- `simulator_frequency`: update rate in Hz.
- `x_axis_scale`, `y_axis_scale`: positive half-ranges; visible axes are
  `[-scale, +scale]` meters.
- `state_noise_x_std`, `state_noise_y_std`: position noise standard deviation in m.
- `state_noise_heading_std`: heading noise standard deviation in rad.
- `actuator_noise_percent`: multiplicative Gaussian actuator noise, in percent.
- `wheelbase`: bicycle wheelbase in meters.
- `max_steering_angle`: steering saturation in radians.
- `footprint_max_points`: maximum stored trajectory points.

Example: at a 2 m/s command and 10% actuator noise, velocity noise sigma is
0.2 m/s.

## Design notes

- `BicycleModel` owns the ground-truth state and integration.
- `SimulatorNode` owns ROS interfaces, timer, noise, and publication.
- `SimulatorVisualizer` owns the Matplotlib drawing and trajectory.
- ROS spins on a worker thread; Matplotlib runs on the main thread.
- The footprint is the historical path of the point, not a vehicle outline.
- This is a kinematic model: it does not model tire slip, acceleration,
  steering lag, collisions, or vehicle dimensions.

## Troubleshooting

**No window:** Run in a desktop session. Headless SSH/container sessions need
X forwarding or a virtual display.

**Matplotlib missing:** Install `python3-matplotlib` in the Python environment
used by ROS 2.

**Interface package missing:** Build both packages in the same workspace and
source `install/setup.bash`.

**Topic type unavailable:** Source the workspace and check
`ros2 interface show sim_interfaces/msg/CmdVel`.
