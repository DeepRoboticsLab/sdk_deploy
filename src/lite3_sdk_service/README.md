# lite3_sdk_service

[![Discord](https://img.shields.io/badge/-Discord-5865F2?style=flat&logo=Discord&logoColor=white)](https://discord.gg/gdM9mQutC8)

## Overview

`lite3_sdk_service` is a standalone UDP-based control service for managing the `lite3_transfer` ROS2 node. It provides remote control capabilities to start/stop the transfer node and adjust its publish rate via UDP commands.

The service listens on UDP port **12122** and responds to simple text commands.

### Node Information

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

## Features

- **UDP Command Interface**: Simple text-based commands via UDP port 12122
- **Node Management**: Start and stop `lite3_transfer` node remotely
- **Rate Control**: Dynamically adjust publish rate of `lite3_transfer` node
- **Graceful Shutdown**: Properly exits SDK mode before stopping to avoid damping mode
- **Auto-start Support**: Systemd service for boot-time startup

## UDP Commands

The service accepts the following commands via UDP (port 12122):

### Start Node
```
on
```
- **Response**: `success` or `failure`
- **Description**: Starts the `lite3_transfer` node by executing `ros2 run lite3_transfer lite3_transfer`

### Stop Node
```
off
```
- **Response**: `success` or `failure`
- **Description**: Gracefully stops the `lite3_transfer` node by:
  1. Sending command `0` to `/SDK_MODE` service to exit SDK mode
  2. Waiting for proper mode switch
  3. Sending SIGINT to allow clean shutdown
  4. Force killing if necessary (with timeout)

### Set Publish Rate
```
<number>
```
- **Response**: `success`, `failure`, or `invalid`
- **Description**: Sets the publish rate of `/JOINTS_DATA` and `/IMU_DATA` topics
- **Valid Values**: Must be > 0, ≤ 200, and a divisor of 200
  - Valid examples: `1`, `2`, `4`, `5`, `8`, `10`, `20`, `25`, `40`, `50`, `100`, `125`, `200`
- **Implementation**: Calls `/SDK_MODE` ROS2 service with the specified rate

### Invalid Commands
Any other command will return `invalid`.

## Build and Run

### Prerequisites

- ROS2 (Foxy or later)
- `drdds` package (must be built first)

### Compile

```bash
cd <workspace>
source /opt/ros/foxy/setup.bash  # or your ROS2 distro
colcon build --packages-select lite3_sdk_service
```

### Run

```bash
cd <workspace>
source install/setup.bash
export ROS_DOMAIN_ID=0  # Optional, defaults to 0 if not set
ros2 run lite3_sdk_service sdk_service
```

The service will start listening on UDP port 12122.

## Testing UDP Commands

You can test the service using various UDP clients:

### Using `nc` (netcat)
```bash
# Send commands
echo "on" | nc -u 127.0.0.1 12122
echo "off" | nc -u 127.0.0.1 12122
echo "20" | nc -u 127.0.0.1 12122
```

### Using Python
```python
import socket

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.sendto(b"on", ("127.0.0.1", 12122))
response, addr = sock.recvfrom(1024)
print(f"Response: {response.decode()}")
```

## Auto-start on Boot (Systemd)

To enable the service to start automatically on boot:

### 1. Create Systemd Service File

```bash
sudo vim /etc/systemd/system/lite3_sdk_service.service
```

Add the following content (adjust paths as needed):

```ini
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

# Check status
sudo systemctl status lite3_sdk_service

# View logs
sudo journalctl -u lite3_sdk_service -f
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
```

## Architecture

The service is built with a clean architecture:

- **Base Class** (`service/sdk_service.hpp`): Pure virtual interface defining service operations
- **Implementation** (`service/lite3_sdk_service.hpp`): Lite3-specific implementation
- **Main** (`main.cpp`): Simple entry point that initializes and runs the service

### Key Components

- **UDP Server**: Listens on port 12122 for commands
- **Process Management**: Handles starting/stopping `lite3_transfer` node via `fork()` and process signals
- **ROS2 Service Client**: Communicates with `/SDK_MODE` service to control publish rate
- **Error Handling**: Comprehensive error handling and logging

## Troubleshooting

### Port Already in Use

If you see `bind: Address already in use`, another instance is running:

```bash
# Find and kill existing process
sudo lsof -i:12122
sudo kill <pid>
```

### Service Not Responding

1. Check if service is running: `sudo systemctl status lite3_sdk_service`
2. Check logs: `sudo journalctl -u lite3_sdk_service -n 50`
3. Verify ROS2 domain: Ensure `ROS_DOMAIN_ID=0` matches other nodes

### Node Won't Start

- Ensure `lite3_transfer` package is built and available
- Check ROS2 environment is properly sourced
- Verify network connectivity if running remotely

### Invalid Rate Values

Only values that are divisors of 200 and ≤ 200 are valid:
- Valid: 1, 2, 4, 5, 8, 10, 20, 25, 40, 50, 100, 125, 200
- Invalid: 3, 7, 15, 201, etc.

## Dependencies

- `rclcpp`: ROS2 C++ client library
- `drdds`: Custom message and service definitions

## License

BSD 3-Clause License

## Author

Haokai Dai (haokaidai.zju@gmail.com)
