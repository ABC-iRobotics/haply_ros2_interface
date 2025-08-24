# Test Nodes

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