# SDK DEPLOY

## Installation

### Prerequisites

- **ROS 2 Humble** installed and sourced ([install guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html))

### System dependencies:
  ```bash
  sudo apt install libevdev-dev
  rosdep install --from-paths src --ignore-src -r -y
  ```

### Build

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --cmake-args -DBUILD_PLATFORM=x86
source install/setup.bash
```

### Python venv (for simulation only)

The MuJoCo simulation requires Python packages not available via rosdep. Set up a venv once:

```bash
python3 -m venv venv
touch venv/COLCON_IGNORE
source venv/bin/activate
pip install -r requirements.txt
```

Activate the venv before running simulation commands.


## RTAB-Map SLAM + Nav2 Autonomous Navigation

Uses RTAB-Map (2D lidar ICP mode) for SLAM, then Nav2 for autonomous goal navigation.

### 1. Build a Map (SLAM)

```bash
# Terminal 1 — MuJoCo simulation
source install/setup.bash
source venv/bin/activate
ros2 run lite3_sdk_deploy mujoco_simulation_ros2.py

# Terminal 2 — RL controller (keyboard control for driving around to map)
source install/setup.bash
ros2 run lite3_sdk_deploy rl_deploy

# Terminal 3 — RTAB-Map SLAM
source install/setup.bash
ros2 launch lite3_sdk_deploy rtabmap.launch.py

# Terminal 4 — RViz2
source install/setup.bash
rviz2
```

Stand the robot with "z" then put into RL control mode with "c". Drive the robot around with keyboard controls (wasd) to build the map.

The map is saved automatically to `~/.ros/rtabmap.db`. RTAB-Map will reload it in localization mode.

### 2. Navigate with Nav2

After building a map, use RTAB-Map in localization mode with Nav2 for autonomous navigation:

```bash
# Terminal 1 — MuJoCo simulation
source install/setup.bash
source venv/bin/activate
ros2 run lite3_sdk_deploy mujoco_simulation_ros2.py

# Terminal 2 — RL controller with twist input
source install/setup.bash
ros2 run lite3_sdk_deploy rl_deploy --twist

# Terminal 3 — RTAB-Map in localization mode
source install/setup.bash
ros2 launch lite3_sdk_deploy rtabmap.launch.py localization:=true

# Terminal 4 — Navigation (planner + controller)
source install/setup.bash
ros2 launch nav2_bringup navigation_launch.py \
  params_file:=$(pwd)/src/Lite3_sdk_deploy/config/nav2_params.yaml

# Terminal 5 — RViz2
source install/setup.bash
rviz2
```

In RViz2:
1. Add **Map** display (topic `/map`, durability: Transient Local)
2. Set **2D Goal Pose** to send a navigation goal

## Twist Control (Simulation)

### Run

```bash
# Terminal 1 — MuJoCo simulation (publishes odom, TF, /scan)
ros2 run lite3_sdk_deploy mujoco_simulation_ros2.py

# Terminal 2 — RL controller with twist input (auto stands up + enters RL mode after ~5s)
ros2 run lite3_sdk_deploy rl_deploy --twist
```

### Manual Velocity Commands

After the robot stands and enters RL mode (~5 seconds), send velocity commands:

```bash
# Move forward (0.3 m/s)
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.3, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" -r 10

# Strafe left (0.3 m/s)
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.3, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" -r 10

# Turn left (0.3 rad/s)
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.3}}" -r 10

# Stop
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" --once
```

| Twist field    | Robot motion          | Max value |
|----------------|-----------------------|-----------|
| `linear.x`    | Forward / backward    | 0.7 m/s   |
| `linear.y`    | Strafe left / right   | 0.5 m/s   |
| `angular.z`   | Turn left / right     | 0.7 rad/s |




## SDK Overview
This repository contains the robotics control SDK, currently supporting Lite3 and M20.

> [!NOTE]
> Damage caused by using SDK is not covered under warranty!

 The repository is structured as follows:
- `src/drdds`: The drdds communication format used by the SDK.
### Lite3
- `src/Lite3_sdk_deploy`: The source code for the Lite3 SDK deploy.
- `src/lite3_sdk_service`: The source code for Lite3 SDK mode switching and frequency modification services.
- `src/lite3_transfer`: The source code for Lite3 UDP-ROS2 message transfer.  

Before using the Lite3 SDK, please refer to the [Lite3 SDK Service Guide](README_lite3_sdk_service.md). For the Lite3 SDK deployment process, please refer to the [Lite3 SDK Deployment Guide](src/Lite3_sdk_deploy/README.md).
### M20
- `src/M20_sdk_deploy`: The source code for the M20 SDK deploy.  

For the M20 SDK deployment process, please refer to the [M20 SDK Deployment Guide](/src/M20_sdk_deploy/README.md).  
## Contributors
See the [Contributors](Contributors.md) page for a list of contributors.

## SDK总览
本仓库包含机器人控制SDK，当前支持Lite3和M20平台。

> [!NOTE]
> 因使用 SDK 造成的设备损坏不在保修范围内！

仓库结构如下：
- `src/drdds`：SDK使用的drdds通信格式。
### Lite3
- `src/Lite3_sdk_deploy`：Lite3 SDK部署的源代码。
- `src/lite3_sdk_service`：Lite3 SDK模式切换、频率修改服务的源代码。
- `src/lite3_transfer`：Lite3 UDP-ROS2消息转换的源代码。

使用Lite3 SDK前请查看[Lite3 SDK服务说明](README_lite3_sdk_service.md)，Lite3 SDK部署流程请查看[Lite3 SDK部署说明](src/Lite3_sdk_deploy/README.md)。
### M20
- `src/M20_sdk_deploy`：M20 SDK部署的源代码。

M20 SDK部署流程请查看[M20 SDK部署说明](/src/M20_sdk_deploy/README.md)。
## 贡献者
请参阅[贡献者](Contributors.md)页面查看贡献者列表。


