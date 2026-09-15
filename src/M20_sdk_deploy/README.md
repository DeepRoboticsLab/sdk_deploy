# M20 SDK Deploy

[![Discord](https://img.shields.io/badge/-Discord-5865F2?style=flat&logo=Discord&logoColor=white)](https://discord.gg/gdM9mQutC8)

## 1. Overview

This repository uses ROS2 to implement the entire Sim-to-sim and Sim-to-real workflow. Therefore, ROS2 must first be installed on your computer, such as installing [ROS2 Humble](https://docs.ros.org/en/humble/index.html) on Ubuntu 22.04. We've also released an introduction [video](https://www.youtube.com/watch?v=FNaxsDBtD7A), please check it out! Please go through the whole process on a Ubuntu system.
```mermaid
graph LR
    A["/rl_deploy"] -->|/JOINTS_CMD| B["/mujoco_simulation"]
    B -->|/IMU_DATA| A
    B -->|/JOINTS_DATA| A
```
The following listings show the low-level ROS 2 communication; control commands are described in the control interfaces section below.

```bash
# ros2 topic list
/BATTERY_DATA
/IMU_DATA
/JOINTS_CMD
/JOINTS_DATA
/parameter_events
/rosout

# ros2 node info /mujoco_simulation 
/mujoco_simulation
  Subscribers:
    /JOINTS_CMD: drdds/msg/JointsDataCmd
  Publishers:
    /IMU_DATA: drdds/msg/ImuData
    /JOINTS_DATA: drdds/msg/JointsData
    /parameter_events: rcl_interfaces/msg/ParameterEvent
    /rosout: rcl_interfaces/msg/Log
  Service Servers:
    /mujoco_simulation/describe_parameters: rcl_interfaces/srv/DescribeParameters
    /mujoco_simulation/get_parameter_types: rcl_interfaces/srv/GetParameterTypes
    /mujoco_simulation/get_parameters: rcl_interfaces/srv/GetParameters
    /mujoco_simulation/list_parameters: rcl_interfaces/srv/ListParameters
    /mujoco_simulation/set_parameters: rcl_interfaces/srv/SetParameters
    /mujoco_simulation/set_parameters_atomically: rcl_interfaces/srv/SetParametersAtomically
  Service Clients:

  Action Servers:

  Action Clients:

# ros2 node info /rl_deploy 
/rl_deploy
  Subscribers:
    /BATTERY_DATA: drdds/msg/BatteryData
    /IMU_DATA: drdds/msg/ImuData
    /JOINTS_DATA: drdds/msg/JointsData
    /parameter_events: rcl_interfaces/msg/ParameterEvent
  Publishers:
    /JOINTS_CMD: drdds/msg/JointsDataCmd
    /parameter_events: rcl_interfaces/msg/ParameterEvent
    /rosout: rcl_interfaces/msg/Log
  Service Servers:
    /rl_deploy/describe_parameters: rcl_interfaces/srv/DescribeParameters
    /rl_deploy/get_parameter_types: rcl_interfaces/srv/GetParameterTypes
    /rl_deploy/get_parameters: rcl_interfaces/srv/GetParameters
    /rl_deploy/list_parameters: rcl_interfaces/srv/ListParameters
    /rl_deploy/set_parameters: rcl_interfaces/srv/SetParameters
    /rl_deploy/set_parameters_atomically: rcl_interfaces/srv/SetParametersAtomically
  Service Clients:

  Action Servers:

  Action Clients:

```

## 2. Contribution

Everyone is welcome to contribute to this repo. If you discover a bug or optimize our training config, just submit a pull request and we will look into it.

## 3. Sim-to-sim

```bash
pip install "numpy < 2.0" mujoco scipy
git clone https://github.com/DeepRoboticsLab/sdk_deploy.git

# Compile
cd sdk_deploy
source /opt/ros/<ros-distro>/setup.bash
colcon build --packages-up-to m20_sdk_deploy --cmake-args -DBUILD_PLATFORM=x86
```

Start the simulator in Terminal 2 before starting the controller in Terminal 1.
For simultaneous robots, use separate DDS domains for all their joint and IMU topics.

```bash
# Run (Open 2 terminals)
# Terminal 1
export ROS_DOMAIN_ID=1
source install/setup.bash
ros2 run m20_sdk_deploy rl_deploy

# Terminal 2 
export ROS_DOMAIN_ID=1
source install/setup.bash
python3 src/M20_sdk_deploy/interface/robot/simulation/mujoco_simulation_ros2.py
```

### 3.1. Control

For Sim-to-sim tests, use only the keyboard or ROS 2 interface. Refer to [Control interfaces](#5-control-interfaces) for selection instructions and commands.

<span style="color: red;">**Note:**</span>
> - Right click simulator window and select "always on top"
> - When the robot dog stands up, it may become stuck due to self-collision in the simulation. This is not a bug; please try again.

## 4. Sim-to-Real

This process is almost identical to simulation-simulation. You only need to add the step of connecting to Wi-Fi to transfer data, and then modify the compilation instructions.Real-robot control supports keyboard, gamepad, and ROS 2 input. Select the desired `RemoteCommandType` in `main.cpp` and rebuild; the current selection is `kRos2`.

**Please first use the OTA upgrade function in the handle settings to upgrade the hardware to version 1.1.8. We require a sdk authentication code to activate the sdk mode. Please contact our technical support team to get this unique code for each robot.**

```bash

# computer and gamepad should both connect to WiFi
# WiFi: CA9B********
# Passward: 12345678 (If wrong, contact technical support)
# Note: If you are connected via the second WiFi, use 10.21.41.1 instead of 10.21.31.103
#       for ssh/scp below. It is the same robot, only the IP differs per WiFi network.

# scp to transfer files to M20 (open a terminal on your local computer)
# Include the updated drdds source to generate RobotStatus in the workspace.
ssh user@10.21.31.103 'mkdir -p ~/M20_sdk_deploy/src'
scp -r ~/sdk_deploy/src/drdds ~/sdk_deploy/src/M20_sdk_deploy user@10.21.31.103:~/M20_sdk_deploy/src

# ssh connect for remote development, 
ssh user@10.21.31.103
cd M20_sdk_deploy
source /opt/ros/foxy/setup.bash #source ROS2 env
# Build drdds and the SDK into this workspace; keep the firmware installation
# under /opt/ros/foxy unchanged. Source this workspace when running the new SDK.
colcon build --packages-up-to m20_sdk_deploy --cmake-args -DBUILD_PLATFORM=arm

sudo su # Root
source /opt/ros/foxy/setup.bash #source ROS2 env
source /opt/robot/scripts/setup_ros2.sh
# Use the gamepad to enable SDK mode. Need authorization code, please contact technical support team.

# Run
source install/setup.bash
ros2 run m20_sdk_deploy rl_deploy

# exit sdk mode：
# Use the gamepad to enable SDK mode.
```

## 5. Control interfaces

Use the build and deployment steps above before selecting an input method; the
current selection in `main.cpp` is `kRos2`.

### 5.1. Keyboard

To use keyboard control, set `RemoteCommandType::kKeyBoard` (`0`) in [main.cpp](main.cpp), rebuild, and restart the SDK.

Press the following keys in the terminal running `rl_deploy`:

- z： default position
- c： rl control default position
- x： lie down
- wasd：forward/leftward/backward/rightward
- qe：counter clockwise/clockwise

### 5.2. Gamepad

To use gamepad control, set `RemoteCommandType::kGamepad` (`1`) in [main.cpp](main.cpp), rebuild, and restart the SDK.

*(Note: When using the gamepad control function, please ensure that the Gamepad APP version is V1.5.11 or higher.)*
![2838b054246d4700247b36207243258f](https://github.com/user-attachments/assets/ed4e8340-c1fe-4202-916a-84e80e537b7f)

- L1： default position
- L2： rl control default position
- R1： lie down
- R2： joint damping
- Left joystick：forward/leftward/backward/rightward
- Right joystick：clockwise/counter clockwise

### 5.3. ROS 2

To use ROS 2 control, set `RemoteCommandType::kRos2` (`2`) in [main.cpp](main.cpp), rebuild, and restart the SDK.

Start the controller with `ros2 run m20_sdk_deploy rl_deploy`; in another
terminal, source the same ROS/workspace environment and use the same DDS domain.
For the Sim-to-sim setup above, run `export ROS_DOMAIN_ID=1` in this command terminal too.

**Stand up:**

```bash
ros2 topic pub --once /rl_deploy/command_state std_msgs/msg/String '{data: stand}'
```

**Enter policy control after standing has finished:**

```bash
ros2 topic pub --once /rl_deploy/command_state std_msgs/msg/String '{data: rl_control}'
```

**Simulation movement example (forward at a requested 0.1 m/s):**

```bash
ros2 topic pub --rate 50 /rl_deploy/cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.1, y: 0.0}, angular: {z: 0.0}}'
```

Use `linear.x` for forward/backward (m/s), `linear.y` for left/right (m/s), and
`angular.z` for counterclockwise/clockwise yaw (rad/s); other axes must be zero.
Nonzero velocity is used only in policy control. Press Ctrl+C to stop publishing;
after 0.25 seconds without velocity input, the effective command becomes zero.

**Lie down:**

```bash
ros2 topic pub --once /rl_deploy/command_state std_msgs/msg/String '{data: lie_down}'
```

**Request joint damping:**

```bash
ros2 topic pub --once /rl_deploy/command_state std_msgs/msg/String '{data: damping}'
```

**Read current state, commands, joint data, and base orientation/angular velocity:**

```bash
ros2 topic echo /rl_deploy/status
# In a separate terminal, check the effective command rate:
ros2 topic hz /rl_deploy/effective_cmd_vel
```

`/rl_deploy/status` publishes [drdds/msg/RobotStatus](../drdds/msg/RobotStatus.msg)
at 10 Hz; effective commands are published at 50 Hz. Default velocity limits are
0.7 m/s forward, 0.5 m/s sideways, and 0.7 rad/s yaw. For posture-only operation,
start the controller with all three limits at zero:

```bash
ros2 run m20_sdk_deploy rl_deploy --ros-args \
  -p max_forward_velocity:=0.0 -p max_side_velocity:=0.0 -p max_yaw_velocity:=0.0
```

Simulation command and stand/lie checks passed, but the existing policy shows
negligible sideways motion, yaw overshoot, and drift at zero input. Hardware
checks covered stand/lie only.
