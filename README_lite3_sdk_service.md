# Lite3_sdk_service
[![Discord](https://img.shields.io/badge/-Discord-5865F2?style=flat&logo=Discord&logoColor=white)](https://discord.gg/gdM9mQutC8)  
## Note
<span style="color: red;">**This is the companion service program for Lite3 SDK deployment. It is only available for Lite3 Venture Edition.**</span>  
## Overview
This interface is used for the Lite3 SDK deploy companion startup. It converts UDP messages into ROS2 messages used by the SDK and configures automatic startup at boot.  
`lite3_transfer` package will run lite3 and retroid_gamepad nodes. The nodes information is as follows: 
```bash
# ros2 node info /lite3
/lite3_transfer
  Subscribers:
    /JOINTS_CMD: drdds/msg/JointsDataCmd
    /parameter_events: rcl_interfaces/msg/ParameterEvent
  Publishers:
    /IMU_DATA: drdds/msg/ImuData
    /JOINTS_DATA: drdds/msg/JointsData
    /parameter_events: rcl_interfaces/msg/ParameterEvent
    /rosout: rcl_interfaces/msg/Log
  Service Servers:
    /SDK_MODE: drdds/srv/StdSrvInt32
    /lite3/describe_parameters: rcl_interfaces/srv/DescribeParameters
    /lite3/get_parameter_types: rcl_interfaces/srv/GetParameterTypes
    /lite3/get_parameters: rcl_interfaces/srv/GetParameters
    /lite3/list_parameters: rcl_interfaces/srv/ListParameters
    /lite3/set_parameters: rcl_interfaces/srv/SetParameters
    /lite3/set_parameters_atomically: rcl_interfaces/srv/SetParametersAtomically
  Service Clients:

  Action Servers:

  Action Clients:

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
Service Clients:

Action Servers:

Action Clients:
```
`lite3_sdk_service` package is a standalone UDP-based control service for managing the `lite3_transfer` ROS2 node.  
It provides remote control capabilities to start/stop the transfer node and adjust its publish rate via UDP commands.
The nodes information is as follows: 
```bash
# ros2 node info /lite3_sdk_service
/lite3_sdk_service
  Subscribers:
    /parameter_events: rcl_interfaces/msg/ParameterEvent
  Publishers:
    /rosout: rcl_interfaces/msg/Log
  Service Clients:
    /SDK_MODE: drdds/srv/StdSrvInt32
  Service Servers:
    /lite3_sdk_service/describe_parameters: rcl_interfaces/srv/DescribeParameters
    /lite3_sdk_service/get_parameter_types: rcl_interfaces/srv/GetParameterTypes
    /lite3_sdk_service/get_parameters: rcl_interfaces/srv/GetParameters
    /lite3_sdk_service/list_parameters: rcl_interfaces/srv/ListParameters
    /lite3_sdk_service/set_parameters: rcl_interfaces/srv/SetParameters
    /lite3_sdk_service/set_parameters_atomically: rcl_interfaces/srv/SetParametersAtomically
```
## Preparation
Using the lite3 sdk service requires network configuration and installation of ROS2. Please **follow the steps in sequence**.
### Modify ip
modify this file jy_exe/conf/network.toml to this content:
```bash
ip = '192.168.2.1'
target_port = 43897
local_port = 43893

ips = ['192.168.1.103']
ports = [43897]
```
### Network configuration
```bash
# Search for available Wi-Fi networks and add Wi-Fi
sudo nmcli dev wifi list
sudo nmcli dev wifi connect "name" password "password" ifname wlan0
```
After connecting to Wi-Fi, please attempt to ping external networks. If ping fails, follow these steps:  
Navigate to ~/etc/netplan/config.yaml
```bash
# Delete the gateway configuration. The following is an example; please delete according to your actual situation.
network:
    version: 2
    renderer: NetworkManager
    ethernets:
        eth1:
            addresser:
                - 192.168.1.120/24
                - 192.168.137.120/24
            gateway4: 192.168.137.120   # Delete this gateway.
            nameservers:
                addresser: [223.5.5.5]
# Apply changes after saving
sudo netplan apply
```
After apply, you may encounter issues connecting to Wi-Fi of Lite3. Please try again after a short wait or restart your device. Use the ping command again to verify if you can connect to external networks.
### Install ROS2
The Lite3 computer system runs Ubuntu 20.04. Please refer to [ROS2 Foxy](https://docs.ros.org/en/foxy/index.html) for ROS2 installation.
## Transmit and compile packages
Before SCP transfer, you can first remove the SDK deploy folder to reduce transfer time.
### Compile packages
```bash
# scp to transfer files to lite3 (open a terminal on your local computer) password is ' (a single quote)
ssh ysc@192.168.2.1 'mkdir -p ~/lite3_sdk_service/src' && \
scp -r ~/sdk_deploy/src/drdds ~/sdk_deploy/src/lite3_sdk_service ~/sdk_deploy/src/lite3_transfer ysc@192.168.2.1:~/lite3_sdk_service/src


# ssh connect for remote development
ssh ysc@192.168.2.1
cd lite3_sdk_service
source /opt/ros/foxy/setup.bash
colcon build --packages-up-to lite3_sdk_service
```
You can run these packages to test whether they compile and install correctly.
```bash
# run lite3 sdk service
cd lite3_sdk_service
source install/setup.bash
export ROS_DOMAIN_ID=0  # Optional, defaults to 0 if not set
ros2 run lite3_sdk_service sdk_service
```
## Auto-start on Boot (Systemd)
### 1. Create Systemd Service File
```bash
# add a new service
cd /etc/systemd/system/
sudo vim lite3_sdk_service.service
```

```bash
# add the following content
[Unit]
Description=Lite3 SDK Control UDP Service
After=network.target

[Service]
Type=simple
User=ysc  # Replace with your username
Environment=ROS_DOMAIN_ID=0
ExecStart=/bin/bash -lc 'source /opt/ros/foxy/setup.bash && \
                         source /home/ysc/transfer/install/setup.bash && \
                         exec /home/ysc/transfer/install/lite3_sdk_service/lib/lite3_sdk_service/sdk_service'
Restart=on-failure

[Install]
WantedBy=multi-user.target
```

### 2. Enable and Start Service

```bash
# Reload systemd
sudo systemctl daemon-reload

# Enable auto-start on boot
sudo systemctl enable lite3_sdk_service

# Start service immediately
sudo systemctl start lite3_sdk_service
```

### 3. Service Management Commands

```bash
# Start service
sudo systemctl start lite3_sdk_service

# Stop service
sudo systemctl stop lite3_sdk_service

# Restart service
sudo systemctl restart lite3_sdk_service

# Disable auto-start
sudo systemctl disable lite3_sdk_service

# Check status
sudo systemctl status lite3_sdk_service

# View logs
sudo journalctl -u lite3_sdk_service -f
```
## Troubleshooting
### Service Not Responding

1. Check if service is running: `sudo systemctl status lite3_sdk_service`
2. Check logs: `sudo journalctl -u lite3_sdk_service -n 50`
3. Verify ROS2 domain: Ensure `ROS_DOMAIN_ID=0` matches other nodes

### Node Won't Start

- Ensure `lite3_transfer` package is built and available
- Check ROS2 environment is properly sourced
- Verify network connectivity if running remotely
