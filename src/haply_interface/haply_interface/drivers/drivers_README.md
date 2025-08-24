# Driver nodes

This README contains detailed documentation for the available driver nodes in this including their functionality, published and subscribed topics, parameters, and usage instructions.


## `haply_driver_node`

This is the main driver node responsible for managing **both** the Inverse3 and the VerseGrip Stylus devices simultaneously. It performs the following key tasks:

- Establishes a WebSocket connection with the `haply-inverse-service` running on the host machine.
- Listens to real-time data streams from both devices, such as:
  - Cursor position and velocity (Inverse3)
  - Orientation and button states (VerseGrip Stylus)
- Publishes this data to various ROS2 topics:
  - `inverse3_state` (`haply_msgs/Inverse3State`)  
    → Contains the current position and velocity of the Inverse3 cursor.
  - `handle_quaternion_state` (`geometry_msgs/Quaternion`)  
    → Publishes the raw orientation of the VerseGrip Stylus.
  - `handle_buttons_state` (`std_msgs/UInt8`)  
    → Encodes the binary state of the buttons on the handle.
  - `handle_state` (`haply_msgs/HandleState`)  
    → Combines the orientation and buttons into a single message.
  - `haply_state` (`haply_msgs/HaplyState`)  
    → Unified topic containing the full state: position, velocity, orientation, and buttons.
- Accepts force control commands on the following topic:
  - `haply_force_command` (`haply_msgs/HaplyControl`)  
    → Accepts desired force values (`x`, `y`, `z`) to be applied to the Inverse3 device.
- Runs an internal timer that continuously updates all output topics at a configurable rate.
- Logs uptime and device status on startup to the terminal.

This node is useful when you want to interface with both devices simultaneously.

To start the node, use the following command:

```bash
ros2 run haply_interface haply_driver_node --ros-args -p frequency:=5.0
```

## `inverse3_driver_node`

This node is a dedicated ROS2 driver for the **Inverse3** device, allowing developers to monitor its state and control the applied forces independently of other hardware.

It performs the following key tasks:

- Establishes a WebSocket connection with the `haply-inverse-service` running on the host machine.
- Continuously receives real-time data from the Inverse3 device, including:
  - Cursor position (`x`, `y`, `z`)
  - Cursor velocity
- Publishes this data to a ROS2 topic:
  - `inverse3_state` (`haply_msgs/Inverse3State`)  
    → Contains the current position and velocity of the Inverse3 cursor.
- Subscribes to force control commands from:
  - `haply_force_command` (`haply_msgs/HaplyControl`)  
    → Receives desired force vectors (`x`, `y`, `z`) to be applied to the Inverse3 device.
- Runs a configurable timer to publish the state at a fixed frequency.
- Logs uptime and device status on startup to the terminal.

This node is useful when you only want to interface with the Inverse3 device.

To start the node, use the following command:

```bash
ros2 run haply_interface inverse3_driver_node --ros-args -p frequency:=5.0
```

## `handle_driver_node`

This node is a dedicated ROS2 driver for the **VerseGrip Stylus** (also referred to as the Handle), allowing standalone monitoring of its orientation and button states without requiring the Inverse3 device.

It performs the following key tasks:

- Establishes a WebSocket connection with the `haply-inverse-service` running on the host machine.
- Continuously receives real-time data from the VerseGrip Stylus, including:
  - 3D orientation (quaternion)
  - Button states (on/off)
- Publishes this data to the following ROS2 topics:
  - `handle_quaternion_state` (`geometry_msgs/Quaternion`)  
    → Publishes the raw orientation of the stylus as a quaternion.
  - `handle_buttons_state` (`std_msgs/UInt8`)  
    → Publishes the binary button states as an integer-encoded bitmask.
  - `handle_state` (`haply_msgs/HandleState`)  
    → Combines orientation and button state into a single message for convenience.
- Operates a timer that controls how often the state is published.
- Logs uptime and device status on startup to the terminal.

This node is useful when you only want to interface with the VerseGrip Stylus (Handle) device.

To start the node, use the following command:

```bash
ros2 run haply_interface handle_driver_node --ros-args -p frequency:=5.0
```