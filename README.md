# Rover Description

## Clone Repository

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

git clone <GITHUB_REPO_URL>
```

## Install Dependencies

```bash
cd ~/ros2_ws

bash src/requirements.sh
```

## Configure Gazebo Resource Path

Add the following line to your `~/.bashrc` file:

```bash
export IGN_GAZEBO_RESOURCE_PATH=$HOME/<your_workspace>/src:$IGN_GAZEBO_RESOURCE_PATH
```

Reload your shell:

```bash
source ~/.bashrc
```

## Install ROS Dependencies

```bash
source /opt/ros/humble/setup.bash

rosdep install --from-paths src --ignore-src -r -y
```

## Build

```bash
colcon build --symlink-install

source install/setup.bash
```

## Launch Simulation

```bash
ros2 launch rover_description sim.launch.py
```
