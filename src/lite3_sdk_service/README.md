# lite3_sdk_service

[![Discord](https://img.shields.io/badge/-Discord-5865F2?style=flat&logo=Discord&logoColor=white)](https://discord.gg/gdM9mQutC8)

## Overview

`lite3_sdk_service` is a standalone UDP-based control service for managing the `lite3_transfer` ROS2 node. It provides remote control capabilities to start and stop the transfer node, adjust its publish rate, and trigger emergency stop via UDP commands.

`lite3_transfer` is the low-level hardware and ROS2 bridge layer. It runs the Lite3 hardware interface node and the `retroid_gamepad` publisher node.

This deployment flow is intended for the Lite3 robot host. It requires Wi-Fi/network configuration, ROS2 installation, source package upload, local build, and systemd setup. Please **follow the steps in order**.

## Node Information

### lite3_sdk_service

```bash
# ros2 node info /lite3_sdk_service
/lite3_sdk_service
  Subscribers:
    /parameter_events: rcl_interfaces/msg/ParameterEvent
  Publishers:
    /rosout: rcl_interfaces/msg/Log
  Service Clients:
    /EMERGENCY_STOP: drdds/srv/StdSrvInt32
    /SDK_MODE: drdds/srv/StdSrvInt32
  Service Servers:
    /lite3_sdk_service/describe_parameters: rcl_interfaces/srv/DescribeParameters
    /lite3_sdk_service/get_parameter_types: rcl_interfaces/srv/GetParameterTypes
    /lite3_sdk_service/get_parameters: rcl_interfaces/srv/GetParameters
    /lite3_sdk_service/list_parameters: rcl_interfaces/srv/ListParameters
    /lite3_sdk_service/set_parameters: rcl_interfaces/srv/SetParameters
    /lite3_sdk_service/set_parameters_atomically: rcl_interfaces/srv/SetParametersAtomically
```

### lite3_transfer

```bash
# ros2 node info /lite3
/lite3_transfer
  Subscribers:
    /JOINTS_CMD: drdds/msg/JointsDataCmd
    /parameter_events: rcl_interfaces/msg/ParameterEvent
  Publishers:
    /IMU_DATA: drdds/msg/ImuData
    /JOINTS_DATA: drdds/msg/JointsData
    /EMERGENCY_STOP_SIGNAL: drdds/msg/StdMsgInt32
    /parameter_events: rcl_interfaces/msg/ParameterEvent
    /rosout: rcl_interfaces/msg/Log
  Service Servers:
    /SDK_MODE: drdds/srv/StdSrvInt32
    /EMERGENCY_STOP: drdds/srv/StdSrvInt32
    /lite3/describe_parameters: rcl_interfaces/srv/DescribeParameters
    /lite3/get_parameter_types: rcl_interfaces/srv/GetParameterTypes
    /lite3/get_parameters: rcl_interfaces/srv/GetParameters
    /lite3/list_parameters: rcl_interfaces/srv/ListParameters
    /lite3/set_parameters: rcl_interfaces/srv/SetParameters
    /lite3/set_parameters_atomically: rcl_interfaces/srv/SetParametersAtomically
```

### retroid_gamepad

```bash
# ros2 node info /retroid_gamepad
/retroid_gamepad
  Subscribers:
    /parameter_events: rcl_interfaces/msg/ParameterEvent
  Publishers:
    /GAMEPAD_DATA: drdds/msg/GamepadData
    /parameter_events: rcl_interfaces/msg/ParameterEvent
    /rosout: rcl_interfaces/msg/Log
  Service Servers:
    /retroid_gamepad/describe_parameters: rcl_interfaces/srv/DescribeParameters
    /retroid_gamepad/get_parameter_types: rcl_interfaces/srv/GetParameterTypes
    /retroid_gamepad/get_parameters: rcl_interfaces/srv/GetParameters
    /retroid_gamepad/list_parameters: rcl_interfaces/srv/ListParameters
    /retroid_gamepad/set_parameters: rcl_interfaces/srv/SetParameters
    /retroid_gamepad/set_parameters_atomically: rcl_interfaces/srv/SetParametersAtomically
```

## Prerequisites
All configurations must be performed on the motion host.

```bash
# Connect the computer and gamepad to your robot's Wi-Fi.
# Replace these placeholders with your robot's connection details.
ROBOT_USER=your_robot_user
ROBOT_HOST=your_robot_address
ssh "${ROBOT_USER}@${ROBOT_HOST}"
# Authenticate using the credentials configured on your robot.
```

### Network configuration

```bash
# Search for available Wi-Fi networks and add Wi-Fi
sudo nmcli dev wifi list
sudo nmcli dev wifi connect "name" password "password" ifname wlan0
```
After connecting to Wi-Fi, please attempt to ping external networks. If ping fails, follow these steps:

Navigate to `~/etc/netplan/config.yaml`
```bash
sudo vim ~/etc/netplan/config.yaml
```
Delete the gateway configuration. The following is an example; please delete according to your actual situation.
```bash
network:
    version: 2
    renderer: NetworkManager
    ethernets:
        eth1:
            addresser:
                - 192.168.1.120/24
                - 192.168.137.120/24
            gateway4: 192.168.137.120   # Delete this line.
            nameservers:
                addresser: [223.5.5.5]
```
Apply changes after saving
```bash
sudo netplan apply
```
After apply, you may encounter issues connecting to Wi-Fi of Lite3. Please try again after a short wait or restart your device. Use the ping command again to verify if you can connect to external networks.

### Install ROS2

The Lite3 host system uses Ubuntu 20.04 in the current workflow. Install ROS2 Foxy unless your environment has already been standardized differently.

Reference:

- [ROS2 Foxy documentation](https://docs.ros.org/en/foxy/index.html)

### Configure `network.toml`

```bash
vim ~/jy_exe/conf/network.toml
```

Modify `network.toml` to to this content.

```toml
ip = '192.168.2.1'
target_port = 43897
local_port = 43893

ips = ['192.168.1.103']
ports = [43897]
```

## SDK Service Features

- **Node Management**: Start and stop `lite3_transfer` remotely
- **Rate Control**: Adjust the publish rate of `lite3_transfer`
- **Emergency Stop**: Trigger the Lite3 emergency stop chain through `/EMERGENCY_STOP`
- **Auto-start Support**: Start `lite3_sdk_service` automatically via systemd

## Service Commands

The service accepts the following commands via UDP on port `12122`.

### Start or stop transfer

```text
on/off
```

- **Response**: `success` or `failure`

### Set publish rate

```text
<number>
```

- **Response**: `success`, `failure`, or `invalid`
- **Description**: Sets the publish rate of `/JOINTS_DATA` and `/IMU_DATA`
- **Valid values**: Must be `> 0`, `<= 200`, and a divisor of `200`
  - Valid examples: `1`, `2`, `4`, `5`, `8`, `10`, `20`, `25`, `40`, `50`, `100`, `125`, `200`

### Emergency stop

```text
estop
```
- **Response**: `success` or `failure`

## Service Deploy

### Upload source packages

```bash
# On the development computer, set ROBOT_USER and ROBOT_HOST as above.
ssh "${ROBOT_USER}@${ROBOT_HOST}" "mkdir -p ~/sdk_service/src"
scp -r ~/sdk_deploy/src/drdds ~/sdk_deploy/src/lite3_transfer ~/sdk_deploy/src/lite3_sdk_service "${ROBOT_USER}@${ROBOT_HOST}:~/sdk_service/src"
```

### Run the deployment script

Use `setup_service.sh` to build the required packages and configure the systemd service automatically.

```bash
cd ~/sdk_service/src/lite3_sdk_service
chmod +x setup_service.sh
./setup_service.sh
```

Optional parameters:

```bash
./setup_service.sh -r foxy -d 0
```

- `-r <distro>`: ROS2 distro, default `foxy`
- `-d <domain>`: `ROS_DOMAIN_ID`, default `0`

The script performs:

1. environment checks
2. `colcon build --packages-up-to lite3_sdk_service`
3. creation of `/etc/systemd/system/lite3_sdk_service.service`
4. `systemctl enable` and `systemctl restart`

## Build and Run Manually
If the script deployment fails, you can follow steps below to deploy.

### Compile

```bash
cd ~/sdk_service
source /opt/ros/foxy/setup.bash
colcon build --packages-up-to lite3_sdk_service
```
Run it to test if compile successfully.
```bash
source install/setup.bash
ros2 run lite3_sdk_service sdk_service
```

## Auto-start on Boot (Systemd)

### Create service file manually

```bash
sudo vim /etc/systemd/system/lite3_sdk_service.service
```

Example (replace `your_robot_user` and its home directory with your robot account):

```ini
[Unit]
Description=Lite3 SDK Control Service
After=network.target

[Service]
Type=simple
User=your_robot_user
Environment=ROS_DOMAIN_ID=0
ExecStart=/bin/bash -lc 'source /opt/ros/foxy/setup.bash && \
                         source /home/your_robot_user/sdk_service/install/setup.bash && \
                         exec ros2 run lite3_sdk_service sdk_service'
Restart=on-failure

[Install]
WantedBy=multi-user.target
```

### Enable and manage the service

```bash
# start the service
sudo systemctl daemon-reload
sudo systemctl enable lite3_sdk_service
sudo systemctl start lite3_sdk_service
sudo systemctl status lite3_sdk_service
```
```bash
# other commands to manage
sudo systemctl stop lite3_sdk_service
sudo systemctl restart lite3_sdk_service
sudo systemctl disable lite3_sdk_service
```

## Troubleshooting

### Port already in use

If you see `bind: Address already in use`:

```bash
sudo lsof -i:12122
sudo kill <pid>
```

### Service not responding

Check:

```bash
sudo systemctl status lite3_sdk_service
sudo journalctl -u lite3_sdk_service -n 50
```

### Service repeatedly exits with `std::bad_alloc`

Check `sudo journalctl -u lite3_sdk_service -n 50`. A crash while decoding
`rmw_dds_common` discovery messages can be caused by incompatible ROS 2 nodes
sharing the network and domain; rebuilding the SDK packages does not remove
that traffic. Stop the incompatible participants or separate their DDS domains
from the robot. The standard onboard service uses domain `0`; a clean shell
also defaults to `0`, so no manual `ROS_DOMAIN_ID` export is required.

If the incompatible traffic comes from the external Wi-Fi network, disconnect
`wlan0` while keeping the robot hotspot (`p2p0`) available for SSH and the gamepad:

```bash
sudo nmcli device disconnect wlan0
sudo systemctl restart lite3_sdk_service
```

To prevent reconnection after reboot, disable autoconnect for each external
Wi-Fi profile with `sudo nmcli connection modify "<external Wi-Fi profile>" connection.autoconnect no`.
Keep autoconnect enabled for the robot hotspot. This avoids custom DDS profiles
and preserves the default ROS domain and normal cross-host communication on
the robot hotspot.

`systemctl is-enabled lite3_sdk_service` confirms boot configuration only.
Also check `systemctl status lite3_sdk_service` for a stable running process,
not `activating (auto-restart)`. Enable SDK mode after the service is stable,
then verify that `lite3_transfer` stays running and publishes joint/IMU data.

### Transfer node does not start

- Ensure `lite3_transfer` was built successfully
- Ensure ROS2 environment files are valid
- Ensure UDP commands are sent to the correct robot-host IP and port

### Emergency stop does not work

- Ensure `lite3_transfer` is already running before sending `estop`
- Check that `/EMERGENCY_STOP` exists on `lite3_transfer`
- Check:

```bash
ros2 service list | grep EMERGENCY_STOP
sudo journalctl -u lite3_sdk_service -n 50
```

- If deploy-side damping does not happen, verify that `/EMERGENCY_STOP_SIGNAL` has an active subscriber

## License

BSD 3-Clause License

## Author

Haokai Dai (haokaidai.zju@gmail.com)
