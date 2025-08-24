# Launch Files for Haply ROS2 Interface

This directory contains various launch files to easily start different driver and state subscriber nodes for the Haply Inverse3 and Handle devices.

## 1. Haply Launch File

This launch file starts the complete `haply_driver_node` which handles **both** the Inverse3 and Handle devices, and a unified `state_subscriber_haply` node to print their states.

**Nodes launched:**
- `haply_driver_node` with `frequency:=5.0`
- `state_subscriber_haply`

**Run with:**
```bash
ros2 launch haply_demos demo_haply_state_read.launch.py
```

## 2. Inverse3 Launch File

This launch file only starts the node for the Inverse3 device and a dedicated subscriber for its state.

**Nodes launched:**
- `inverse3_driver_node ` with `frequency:=5.0`
- `state_subscriber_inverse3`

**Run with:**
```bash
ros2 launch haply_demos demo_inverse3_state_read.launch.py
```

## 3. Handle Launch File

This launch file starts the node for the Handle (VerseGrip) device and a subscriber for its orientation and button states.

**Nodes launched:**
- `handle_driver_node ` with `frequency:=5.0`
- `state_subscriber_handle`

**Run with:**
```bash
ros2 launch haply_demos demo_handle_state_read.launch.py
```

## 4. Visualization Launch File

This launch file starts the RViz-based visualizer for the handle orientation and inverse3 position, along with the main driver node.

**Nodes launched:**
- `haply_driver_node` with `frequency:=5.0`
- `rviz_visualization_node`
- `rviz2`

**Run with:**
```bash
ros2 launch haply_demos visualization.launch.py
```
