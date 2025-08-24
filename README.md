# Haply ROS2 Interface

This package provides a ROS2 interface for the **Haply Inverse3** and **VerseGrip Stylus** haptic devices. It includes drivers, visualization tools, and control nodes to operate and test the devices individually or together within a ROS2 environment.

---

## Contents

- [Prerequisites](#prerequisites)
  - [Platform Recommendation](#platform-recommendation)
  - [ROS2 Version](#ros2-version)
  - [Required Python Packages](#required-python-packages)
  - [Communication Protocol](#communication-protocol)
  - [Other Recommendations](#other-recommendations)
- [Getting Started](#getting-started)
  - [Cloning the Repository](#cloning-the-repository)
  - [Set Up Haply Device](#set-up-haply-device)
  - [Connecting USB Devices to Linux (WSL 2)](#connecting-usb-devices-to-linux-wsl-2)
- [Available Nodes](#available-nodes)
  - [`haply_driver_node`](#haply_driver_node)
  - [`inverse3_driver_node`](#inverse3_driver_node)
  - [`handle_driver_node`](#handle_driver_node)
  - [Demo Nodes](#demo-nodes)

## Prerequisites

### Platform Recommendation

This interface was developed and tested on **Ubuntu 22.04** running under **WSL**. It is therefore **recommended to use the same environment** for compatibility and stability.

If you don't have WSL installed yet, you can find detailed installation instructions here:  
[WSL Installation Guide](https://learn.microsoft.com/en-us/windows/wsl/install)

### ROS2 Version

This interface was developed and tested using **ROS2 Humble**. It is strongly recommended to use this version to ensure compatibility and stability.

You can find more information on how to install ROS2 Humble here:  
[ROS2 Installation Guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)

### Required Python Packages

Below are the external Python packages that must be installed manually:

- **Name:** `websockets` 
  ```bash
  pip install websockets
  ```

- **Name:** `orjson` 
  ```bash
  pip install orjson
  ```

- **Name:** `rclpy` 
  ```bash
  pip install rclpy
  ```

### Communication Protocol

This interface uses a WebSocket-based communication protocol to interact with the devices.

To enable this, the **Haply Inverse SDK Service** must be installed. This service acts as a WebSocket server and is required for the interface to function properly.

For installation and testing instructions, please refer to the official documentation:

[Haply Inverse SDK Service Guide](https://docs.haply.co/inverseSDK/)

### Other Recommendations

1. To verify that your system recognizes the Haply devices, we recommend installing the Haply Device Manager.  
   You can find installation instructions here:  
   [Haply Device Manager](https://docs.haply.co/inverseSDK/)

2. Additional demo codes can be found here:  
   [Haply Demos](https://gitlab.com/Haply/public/python_samples)  
   > **Note:** Please note that these examples use direct serial communication, not WebSocket as the ROS2 interface does.



## Getting started

### Cloning the repository

Before cloning the repository, first create a new ROS2 workspace:

```bash
mkdir -p ~/haply_ws/src
cd ~/haply_ws/src
```

Then clone the repository into the `src` folder:

```bash
git clone <repository url>
```

After cloning, return to the workspace root and build it using `colcon`:

```bash
cd ~/haply_ws
colcon build
```

Finally, source the setup script:

```bash
source install/setup.bash
```
> **Note:** This command must be run in every new terminal where you want to use the ROS2 workspace. You can also add it to your shell configuration file (e.g. .bashrc) to have it sourced automatically.

### Set Up Haply Device

This interface is designed for the Haply Inverse3 and VerseGrip Stylus devices.

For setting up and calibrating the devices, please refer to the official Haply documentation:

[Haply Quick Start Guide](https://docs.haply.co/docs/quick-start)
 
It is recommended to recalibrate the device each time it is reconnected to ensure accurate tracking and operation.

### Connecting USB Devices to Linux (WSL 2)

To use the Haply devices under WSL 2 (e.g., with Ubuntu 22.04), it is necessary to make the USB connection available to your Linux distribution.

For first-time setup and detailed guidance, follow the official documentation:   [Microsoft Guide: Connect USB Devices to WSL](https://learn.microsoft.com/en-gb/windows/wsl/connect-usb)

#### After Initial Setup

After completing the installation steps described in the guide above, follow these commands each time you want to use the device:

1. Open PowerShell as Administrator, and list all connected USB devices:
   ```powershell
   usbipd list
   ```
2. Bind the USB device to allow forwarding to WSL. Replace <busid> with the Bus ID shown in the previous step (e.g., 4-4):
   ```powershell
   usbipd bind --busid <busid>
   ```
3. Attach the USB device to WSL
   ```powershell
   usbipd attach --wsl --busid <busid>
   ```
   > **Important:** Your Linux distribution (e.g., Ubuntu in WSL) **must be running** before performing the `attach` step. Simply open a WSL terminal before executing the `usbipd attach` command.

   > **Note:** If you receive the following warning:  
   > `usbipd: warning: The device appears to be used by Windows; stop the software using the device, or bind the device using the '--force' option.`  
   > It may be caused by a background process such as `haply-inverse-service.exe`.  
   > Open **Task Manager** and stop this process before attempting to attach the device.

4. When finished using the device, you can detach it:
   ```powershell
   usbipd detach --busid <busid>
   ```
> **Note:** The device will be inaccessible from Windows while attached to WSL.



## Available Nodes

### Summary of available nodes
- [`haply_driver_node`](#haply_driver_node) – Main driver that handles both Inverse3 and VerseGrip devices simultaneously.
- [`inverse3_driver_node`](#inverse3_driver_node) – Driver for the Inverse3 only (useful for testing).
- [`handle_driver_node`](#handle_driver_node) – Driver for the VerseGrip Stylus only (useful for testing).
- [`rviz_visualization_node`](#rviz_visualization_node) – Publishes transform data to visualize the device in RViz.
- Demos
---

### `haply_driver_node`

This is the main driver node responsible for managing **both** the Inverse3 and the VerseGrip Stylus devices simultaneously. It performs the following key tasks:

To start the node, use the following command:

```bash
ros2 run haply_interface haply_driver_node --ros-args -p frequency:=5.0
```

### `inverse3_driver_node`

This node is a dedicated ROS2 driver for the **Inverse3** device, allowing developers to monitor its state and control the applied forces independently of other hardware.

To start the node, use the following command:

```bash
ros2 run haply_interface inverse3_driver_node --ros-args -p frequency:=5.0
```

### `handle_driver_node`

This node is a dedicated ROS2 driver for the **VerseGrip Stylus** (also referred to as the Handle), allowing standalone monitoring of its orientation and button states without requiring the Inverse3 device.

To start the node, use the following command:

```bash
ros2 run haply_interface handle_driver_node --ros-args -p frequency:=5.0
```

### Demo Nodes

Several demo nodes have been created for testing the interface. These, along with detailed descriptions, can be found in the demos folder.
