# Demo Nodes

## `state_subscriber_haply`

This minimal ROS2 node subscribes to the `haply_state` topic and prints the received state data (position, velocity, and orientation) of the Haply device to the console.

Useful for quick debugging or visualization of the current device state without using RViz.

To start the node, use the following command:

```bash
ros2 run haply_interface state_subscriber_haply
```

## `state_subscriber_inverse3`

This simple ROS2 node subscribes to the `inverse3_state` topic and logs the Inverse3 cursor position and velocity data.

Useful for debugging device position tracking.

To start the node, use the following command:

```bash
ros2 run haply_interface state_subscriber_inverse3
```

## `state_subscriber_handle`

This simple ROS2 node subscribes to the `handle_state` topic and logs the VerseGrip Stylus handle orientation (quaternion) and buttons data.

Useful for debugging device orientation tracking.

To start the node, use the following command:

```bash
ros2 run haply_interface state_subscriber_handle
```

## `target_position_sinus`

This ROS2 node publishes **sinusoidal target positions** to the `haply_target` topic using the `HaplyControl` message.  

Useful for quickly testing **position-based control** and PID tracking behavior of the Inverse3.

- X and Y are fixed (`x=0.03`, `y=-0.13`),  
- Z oscillates around `0.20 m` with amplitude `0.10 m`,  
- Publishes with 100 Hz.  

To start the node, use:

```bash
ros2 run haply_interface target_position_publisher
```

## `target_position_input`

This interactive ROS2 node lets you **manually send target positions** to the `haply_target` topic using the `HaplyControl` message.

- Prompts in the terminal for `x y z`, e.g. `0.05 -0.12 0.22`
- Publishes `target_position`

Useful for quick, manual testing of position-based control, PID tracking and for exploring the boundaries of the device’s reachable workspace.

**Run:**
```bash
ros2 run haply_interface target_position_input
```

**Example input:**
```bash
0.03 -0.13 0.20
```