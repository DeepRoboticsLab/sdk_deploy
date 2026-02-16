# lite3 transfer

[![Discord](https://img.shields.io/badge/-Discord-5865F2?style=flat&logo=Discord&logoColor=white)](https://discord.gg/gdM9mQutC8)

## Overview
Transfer package will run lite3 and retroid_gamepad nodes. The nodes information is as follows: 
```bash
# Run lite3 node in real
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

# Run retroid_gamepad node in real
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
This process requires connecting to the Wi-Fi of Lite3 computer, configuring the external network and gateway, and downloading ROS2. Please **follow the steps in sequence**.

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
            gateway4: 192.168.137.120   # Delete this line.
            nameservers:
                addresser: [223.5.5.5]
# Apply changes after saving
sudo netplan apply
```
After apply, you may encounter issues connecting to Wi-Fi of Lite3. Please try again after a short wait or restart your device. Use the ping command again to verify if you can connect to external networks.
### Install ROS2
The Lite3 computer system runs Ubuntu 20.04. Please refer to [ROS2 Foxy](https://docs.ros.org/en/foxy/index.html) for ROS2 installation.
### Compile and run lite3 transfer
```bash
# scp to transfer files to lite3 (open a terminal on your local computer) password is ' (a single quote)
scp -r ~/sdk_deploy/src/drdds ysc@192.168.2.1:~/transfer/src
scp -r ~/sdk_deploy/src/lite3_transfer ysc@192.168.2.1:~/transfer/src

# ssh connect for remote development
ssh ysc@192.168.2.1
cd transfer
source /opt/ros/foxy/setup.bash
colcon build --packages-up-to lite3_transfer
```

```bash
# run ros2 message transfer
cd transfer
source install/setup.bash
export ROS_DOMAIN_ID=1
ros2 run lite3_transfer lite3_transfer
```
### (Optional) Node auto-start on boot

```bash
# add a new service
cd /etc/systemd/system/
sudo vim lite3-transfer.service

# add the following content
[Unit]
Description=Lite3 Transfer ROS2 Node
After=multi-user.target

[Service]
Type=simple
User=ysc
WorkingDirectory=/home/ysc/transfer
Environment="ROS_DOMAIN_ID=1"
ExecStart=/bin/bash -c 'source /opt/ros/foxy/setup.bash && source /home/ysc/transfer/install/setup.bash && ros2 run lite3_transfer lite3_transfer'
Restart=on-failure
RestartSec=5

StandardOutput=journal
StandardError=journal
SyslogIdentifier=lite3-transfer

[Install]
WantedBy=multi-user.target
```
There are some service commands.  
If auto-start is enabled, you won't need to manually run the transfer package when using the Lite3 SDK. You only need to run the sdk_deploy.
```bash
# start service
sudo systemctl start lite3-transfer.service

# enable auto-start
sudo systemctl enable lite3-transfer.service

# disable auto-start
sudo systemctl disable lite3-transfer.service
```