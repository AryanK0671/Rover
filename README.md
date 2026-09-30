# Rover Description & Simulation

ROS 2 Humble package for a 6-wheel rocker-bogie rover in Gazebo Fortress (Ignition Gazebo 6).

---

## Prerequisites

- **OS**: Ubuntu 22.04 LTS
- **ROS 2**: Humble Hawksbill
- **Gazebo**: Gazebo Fortress (`ign gazebo` / `gz sim` 6.18+)

---

## Installation & Setup

1. **Install Dependencies**:
   ```bash
   bash src/requirements.sh
   ```

2. **Install ROS Dependencies**:
   ```bash
   source /opt/ros/humble/setup.bash
   rosdep install --from-paths src --ignore-src -r -y
   ```

3. **Build Workspace**:
   ```bash
   colcon build --symlink-install
   source install/setup.bash
   ```

4. **Run Automated Tests**:
   ```bash
   colcon test --event-handlers console_direct+
   ```

> *Note: Gazebo resource paths (`IGN_GAZEBO_RESOURCE_PATH` and `GZ_SIM_RESOURCE_PATH`) are automatically configured by the launch files, so manual `.bashrc` path exports are not required.*

---

## Running the Simulation

### 1. Launch Simulation with Gazebo GUI
```bash
ros2 launch rover_description sim.launch.py
```

### 2. Launch Simulation with RViz Visualization
```bash
ros2 launch rover_description sim.launch.py rviz:=true
```

### 3. Launch Simulation in Headless Mode (Server Only / CI)
```bash
ros2 launch rover_description sim.launch.py headless:=true
```

### Launch Flags
- `headless:=true` : Run Gazebo server only without rendering GUI.
- `rviz:=true` : Launch RViz pre-configured for rover model, obstacles, and PointCloud2 visualization.
- `cmd_vel:=true` (default: `true`) : Automatically run the Twist `/cmd_vel` bridge node.
- `points:=true` (default: `true`) : Run the `scan_to_points` LaserScan-to-PointCloud2 converter node.
- `obstacle:=true` (default: `true`) : Run the `spawn_obstacle` marker publisher node for RViz.

---

## Obstacles & PointCloud2 Testing

### Viewing Obstacles & PointCloud in RViz
Launch the complete simulation with RViz:
```bash
ros2 launch rover_description sim.launch.py rviz:=true
```
In RViz, you will see:
- **Rover Model** centered on `base_link`.
- **Obstacle Markers** (`/obstacle_marker`, namespace `obstacles`): default 1m orange box at (3.0m, 0.0m) and cyan cylinder at (3.5m, 1.8m).
- **PointCloud2** (`/scan/points`): 3D laser return points projected along the front faces of the obstacles.

### Spawning Custom Obstacles Dynamically
To spawn additional obstacles into Gazebo simulation and display them in RViz:
```bash
# Spawn a box at x=2.5m, y=-1.0m
ros2 run rover_description spawn_obstacle.py --name side_box -x 2.5 -y -1.0 --spawn-gazebo

# Spawn a cylinder with custom dimensions and color
ros2 run rover_description spawn_obstacle.py --name pillar -x 4.0 -y 1.2 --shape cylinder --sx 0.6 --sz 1.5 --color cyan --spawn-gazebo

# Standalone RViz test mode (publishes synthetic PointCloud2 test pattern without Gazebo)
ros2 run rover_description spawn_obstacle.py --test-cloud
```

---

## Teleoperation & Control

### Option A: Standard ROS 2 `/cmd_vel` Twist Control (Default)
The simulation launch automatically starts `cmd_vel_to_rover.py`, so the rover accepts standard `geometry_msgs/Twist` on `/cmd_vel` out of the box (e.g. from Nav2 or standard teleop):

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### Option B: Custom Keyboard Teleop (Direct Wheel Speed)
Drive the rover directly with keyboard keybindings:
```bash
ros2 run rover_description rover_teleop_key.py
```
- `i` / `,` : Drive forward / backward
- `j` / `l` : Turn in-place left / right
- `u` / `o` : Drive forward + curve left / right
- `m` / `.` : Drive backward + curve left / right
- `k` / `SPACE` : Stop
- `q` / `z` : Linear speed up / down
- `e` / `c` : Turn speed up / down

---

## Sensors & Diagnostics

- **LiDAR Scan**: Published on `/scan` (`sensor_msgs/msg/LaserScan`, 720 samples, 10 Hz, 360° unoccluded FOV).
- **IMU Data**: Published on `/imu/data` (`sensor_msgs/msg/Imu`, 100 Hz) reporting 3-axis orientation, angular velocity, and linear acceleration.
- **Convert LaserScan to PointCloud2**:
  ```bash
  ros2 run rover_description scan_to_points.py
  ```
  Publishes 3D point cloud on `/scan/points` (`sensor_msgs/msg/PointCloud2`).

---

## Coordinate Conventions (REP-103 & REP-105)

The robot root link `base_link` strictly adheres to standard ROS conventions:
- **+X**: Forward (driving direction towards obstacle course)
- **+Y**: Left
- **+Z**: Up
- **Origin**: Centered on the 6-wheel wheelbase footprint at axle ground height.

---

## Robot Model & PointCloud Verification (RViz Standalone)

To inspect the URDF model and verify PointCloud2 rendering in RViz without starting Gazebo:
```bash
ros2 launch rover_description display.launch.py
```
- `gui:=true` (default) : Launch joint state publisher GUI sliders.
- `rviz:=true` (default) : Launch RViz visualization.
- `obstacle:=true` (default) : Spawns 3D obstacle markers (`/obstacle_marker`, `/obstacle_marker_array`) and synthetic point cloud (`/scan/points`) directly in RViz.

---

## Obstacle Spawner & PointCloud Testing CLI

You can also run the obstacle spawner directly:
```bash
# Publish obstacle markers and synthetic PointCloud2 for RViz testing:
ros2 run rover_description spawn_obstacle.py --test-cloud

# Custom obstacle size and position:
ros2 run rover_description spawn_obstacle.py -x 3.5 -y 0.0 --sx 1.2 --sy 1.2 --sz 1.5 --shape box --color orange --test-cloud

# Dynamically inject an obstacle into active Gazebo Fortress simulation:
ros2 run rover_description spawn_obstacle.py --name pylon -x 2.5 -y 1.2 --shape cylinder --color cyan --spawn-gazebo
```
