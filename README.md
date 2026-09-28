# SDK DEPLOY

## 1. SDK Overview

This repository contains the robotics control SDK, currently supporting Lite3, M20, and S10.

> [!NOTE]
> Damage caused by using SDK is not covered under warranty!

 The repository is structured as follows:

- `src/drdds`: The drdds communication format used by the SDK.

### 1.1. Lite3

- `src/Lite3_sdk_deploy`: The source code for the Lite3 SDK deploy.
- `src/lite3_sdk_service`: The source code for Lite3 SDK mode switching and frequency modification services.
- `src/lite3_transfer`: The source code for Lite3 UDP-ROS2 message transfer.  

Before using the Lite3 SDK, please refer to the [Lite3 SDK Service Guide](src/lite3_sdk_service/README.md). For the Lite3 SDK deployment process, please refer to the [Lite3 SDK Deployment Guide](src/Lite3_sdk_deploy/README.md).

### 1.2. M20

- `src/M20_sdk_deploy`: The source code for the M20 SDK deploy.  

For the M20 SDK deployment process, please refer to the [M20 SDK Deployment Guide](src/M20_sdk_deploy/README.md).

### 1.3. S10

- `src/S10_sdk_deploy`: The source code for the S10 SDK deploy.  

For the S10 SDK deployment process, please refer to the [S10 SDK Deployment Guide](src/S10_sdk_deploy/README.md).

## 2. Contributors

See the [Contributors](Contributors.md) page for a list of contributors.

## 3. SDK总览

本仓库包含机器人控制SDK，当前支持Lite3、M20和S10平台。

> [!NOTE]
> 因使用 SDK 造成的设备损坏不在保修范围内！

仓库结构如下：

- `src/drdds`：SDK使用的drdds通信格式。

### 3.1. Lite3

- `src/Lite3_sdk_deploy`：Lite3 SDK部署的源代码。
- `src/lite3_sdk_service`：Lite3 SDK模式切换、频率修改服务的源代码。
- `src/lite3_transfer`：Lite3 UDP-ROS2消息转换的源代码。

使用Lite3 SDK前请查看[Lite3 SDK服务说明](src/lite3_sdk_service/README.md)，Lite3 SDK部署流程请查看[Lite3 SDK部署说明](src/Lite3_sdk_deploy/README.md)。

### 3.2. M20

- `src/M20_sdk_deploy`：M20 SDK部署的源代码。

M20 SDK部署流程请查看[M20 SDK部署说明](src/M20_sdk_deploy/README.md)。

### 3.3. S10

- `src/S10_sdk_deploy`：S10 SDK部署的源代码。

S10 SDK部署流程请查看[S10 SDK部署说明](src/S10_sdk_deploy/README.md)。

## 4. 贡献者

请参阅[贡献者](Contributors.md)页面查看贡献者列表。

## 5. Control interfaces

### 5.1. Keyboard

Select `RemoteCommandType::kKeyBoard` (`0`) in the robot's `main.cpp`, rebuild, and restart to control it with keys.

### 5.2. Gamepad

Select `RemoteCommandType::kRetroidGamepad` for Lite3 or `RemoteCommandType::kGamepad` for M20 (`1`) in `main.cpp`, rebuild, and restart to use the gamepad.

### 5.3. ROS 2

Select `RemoteCommandType::kRos2` (`2`) in the robot's `main.cpp`, rebuild, and restart to send ROS 2 state and velocity commands.

Both entry points currently select ROS 2. The implementation is inside each
SDK's `interface/user_command/` folder, and status uses
[drdds/msg/RobotStatus.msg](src/drdds/msg/RobotStatus.msg) at 10 Hz with effective
commands at 50 Hz. Key bindings and ROS 2 command examples are in the
[Lite3 control interfaces](src/Lite3_sdk_deploy/README.md#6-control-interfaces) and
[M20 control interfaces](src/M20_sdk_deploy/README.md#5-control-interfaces).
