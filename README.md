# RoboSub

Autonomous underwater vehicle (AUV) control stack for the RoboSub pre-qualification course. Runs as a pygame simulator on any desktop and is architected for a drop-in port to ROS2 on a Raspberry Pi 5.

---

## Architecture

### Design principle

The control stack is middleware-agnostic. The `Submarine` class and all task/subtask code have zero ROS or pygame dependencies — they consume a `SensorSuite` and return `ThrusterCommands`. Swapping the simulator for real hardware only requires replacing the node that fills `SensorSuite` and consumes `ThrusterCommands`.

### Package layout

```
src/robosub/
├── config/
│   └── pid_params.yaml          # All tunable gains (loaded at startup)
├── launch/
│   └── robosub.launch.py        # Launches simulator + submarine nodes
└── robosub/
    ├── mission.py               # Mission plan — edit here to change behavior
    ├── nodes/
    │   ├── simulator_node.py    # ROS2 wrapper around the pygame simulator
    │   └── submarine_node.py    # ROS2 wrapper around the control stack
    ├── simulator/
    │   └── simulator.py         # Pygame 3D physics + rendering
    └── sub/
        ├── submarine.py         # Brain: PID controllers, mission execution
        ├── data_structures.py   # SensorSuite, ThrusterCommands, Vision
        ├── config.py            # Physics and world constants
        ├── vision.py            # HSV blob detection
        ├── utils.py             # angle_diff and other helpers
        └── tasks/
            ├── task_base.py         # Task base class
            ├── subtask_base.py      # Subtask base class
            ├── common_subtasks.py   # Reusable subtask library
            ├── gate_task.py         # Navigate through the red gate
            ├── orbit_turn_task.py   # Orbit the green pole and clear it
            ├── stabilize_task.py    # Dive to depth and hold
            └── shutdown_task.py     # End of mission
```

### Control loop

Each tick the active `Task` calls one `Subtask`, which receives the current `SensorSuite` and `Vision` data and returns `(SubtaskStatus, ThrusterCommands)`. When a subtask completes the task advances to the next one; when a task completes the `Submarine` advances to the next task in the mission plan.

```
SensorSuite → Submarine.update() → current Task → current Subtask → ThrusterCommands
```

### Yaw control

All yaw is a PD controller: `yaw = clip(P * error - D * gyro_z, -1, 1)`.

- Visual servo subtasks (align, approach, orbit): `P = yaw_gain` (per-task parameter), `D = yaw_d_gain` from YAML.
- Heading-hold (`get_heading_commands`): `P = hover_yaw_p_gain`, `D = yaw_d_gain`.
- `yaw_d_gain = 3.0` is the tuned value; increasing it damps oscillation, excessively large values cause saturation and instability.

### Vision

`Vision.update()` runs HSV blob detection once per tick. Tasks call `is_pole_visible()`, `get_pole_center_x()`, `is_gate_visible()`, etc. — they never touch raw images. The green pole is identified by finding the largest taller-than-wide green blob; the gate is two matching-height red blobs with horizontal overlap.

### Gains

All PID gains live in `config/pid_params.yaml` and are loaded at node startup. The file is re-read on each run — no rebuild needed to change gains.

```yaml
align_yaw_p_gain: 0.4    # Visual servo P (rad/s per normalised pixel error)
hover_yaw_p_gain: 0.02   # Heading-hold P (command per degree of error)
yaw_d_gain:       3.0    # Yaw damping (command per rad/s of gyro_z)
depth_p_gain:     2.5
depth_i_gain:     0.15
depth_d_gain:     4.0
```

---

## Running the simulator

### Prerequisites

- Python 3.10+
- ROS2 Jazzy (or Humble)
- `colcon`, `pygame`, `numpy`, `opencv-python`, `cv_bridge`

### Build

```bash
cd robosub_ws
colcon build --symlink-install
source install/setup.bash
```

`--symlink-install` means edits to Python source take effect immediately without a rebuild. Only `setup.py` / `package.xml` changes require a rebuild.

### Run

```bash
ros2 launch robosub robosub.launch.py
```

This starts two nodes:

| Node | What it does |
|---|---|
| `simulator_node` | Runs the pygame physics sim, publishes sensor topics, subscribes to thruster commands |
| `submarine_node` | Runs the control stack, subscribes to sensor topics, publishes thruster commands |

### Simulator controls

| Key | Action |
|---|---|
| `Space` | Pause / resume |
| `R` | Reset simulation |
| `Q` | Quit |

### Tuning without a rebuild

Edit `config/pid_params.yaml`, then press `R` in the simulator window to reset. The node reloads gains on `Submarine.__init__()`, which is called by `submarine_node` at startup. To pick up YAML changes mid-session, restart the submarine node:

```bash
ros2 run robosub submarine_node
```

### Changing the mission

Edit `robosub/mission.py`. Task parameters (approach distance, orbit sway power, etc.) are all keyword arguments in `create_mission()`. No rebuild needed with `--symlink-install`.

---

## ROS2 topics

| Topic | Type | Direction | Description |
|---|---|---|---|
| `/camera/image_raw` | `sensor_msgs/Image` | sim → sub | BGR8 320×240 camera frame |
| `/sensors/depth` | `std_msgs/Float32` | sim → sub | Depth in meters |
| `/sensors/heading` | `std_msgs/Float32` | sim → sub | Heading in degrees (0–360) |
| `/sensors/pitch` | `std_msgs/Float32` | sim → sub | Pitch in degrees |
| `/sensors/imu` | `sensor_msgs/Imu` | sim → sub | Angular velocity + linear acceleration |
| `/sensors/velocity` | `geometry_msgs/Twist` | sim → sub | World-frame velocity (m/s) |
| `/thruster_commands` | `std_msgs/Float32MultiArray` | sub → sim | `[hfl, hfr, hal, har, vf, vr]` normalized -1..1 |
| `/sub/status` | `std_msgs/String` | sub → sim | `"TaskName\|state_name"` for the HUD |
| `/sim/control` | `std_msgs/String` | sim → sub | `pause`, `resume`, `reset`, `quit` |

---

## Porting to Raspberry Pi 5 (hardware)

The `submarine_node` requires **no changes** for hardware operation. Only the `simulator_node` needs to be replaced with a `hardware_node` that publishes the same topics.

### Hardware node responsibilities

Write a new node (`robosub/nodes/hardware_node.py`) that:

1. **Publishes** all sensor topics listed above from real hardware:
   - `/camera/image_raw` — USB or CSI camera via `cv_bridge`
   - `/sensors/depth` — Bar30 pressure sensor (Blue Robotics) via I2C
   - `/sensors/heading`, `/sensors/pitch`, `/sensors/imu` — MPU6050 or BNO055 IMU via I2C
   - `/sensors/velocity` — Zero (or DVL/optical flow if available; the control stack degrades gracefully without it)

2. **Subscribes** to `/thruster_commands` and drives the ESCs:
   - Six BlueRobotics T200 thrusters via ESC PWM signals
   - Map normalized -1..1 to PWM microseconds (1100–1900 µs typical)
   - Consider a hardware arming/disarming sequence

3. **Publishes** `/sim/control` if you want hardware kill-switch events to pause the submarine node.

### Launch file for hardware

Create a `hardware.launch.py` that substitutes `hardware_node` for `simulator_node`:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='robosub', executable='hardware_node',  name='hardware_node',  output='screen'),
        Node(package='robosub', executable='submarine_node', name='submarine_node', output='screen'),
    ])
```

### Pi 5 setup notes

- Install ROS2 Jazzy from the official ARM64 Debian packages.
- Install `python3-opencv`, `python3-numpy`, `python3-smbus2` (for I2C), `ros-jazzy-cv-bridge`.
- The `submarine_node` has no pygame dependency and runs headless.
- Load `config/pid_params.yaml` as ROS2 parameters via the launch file for runtime tuning with `ros2 param set`.
- A hardware watchdog (cut thrusters if `/thruster_commands` goes silent) is strongly recommended.

### Velocity without a DVL

`SensorSuite.velocity_x/y/z` is used by the damping and heading-hold controllers. On hardware without velocity sensing, populate all three with `0.0`. The PD yaw controller and depth PID still work; the sway damping term in `get_heading_commands` will simply have no effect.
