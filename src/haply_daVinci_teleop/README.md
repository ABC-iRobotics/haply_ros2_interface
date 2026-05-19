# Haply-daVinci Teleoperation

This ROS 2 package connects a **Haply Inverse3** with a **VerseGrip stylus** to the
**da Vinci Research Kit PSM1 arm**. It provides teleoperation, gripper control,
loop and wire visualization, force feedback for a hotwire task, contact-state
publishing, and helper nodes for study execution and data recording.

The package can be used with the real da Vinci system and with simulated
visualization/contact workflows. The current implementation was tested with:

- Windows 10 with WSL2 Ubuntu 22.04 LTS
- Ubuntu 24.04.3 LTS
- ROS2 Humble 
- Python 3.10.6

## Contents

- [Package Structure](#package-structure)
- [Nodes and Interfaces](#nodes-and-interfaces)
- [Build and Source](#build-and-source)
- [Running the System](#running-the-system)
- [Hotwire Experiment Workflow](#hotwire-experiment-workflow)
- [Controls](#controls)
- [Contact Counting](#contact-counting)
- [Data Recording](#data-recording)
- [Troubleshooting](#troubleshooting)
- [Pre-study Learnings](#pre-study-learnings)

## Package Structure

```text
haply_daVinci_teleop/
+-- bridge/
|   +-- haply_daVinci_bridge_node_jointcontrol.py
|   +-- haply_daVinci_bridge_node_baseframe.py
+-- haptic_visualization/
|   +-- haptic_hotwire.py
|   +-- haptic_loop.py
+-- contact_detection/
|   +-- ContactDetection.ino
|   +-- simulated_contact_detection.py
+-- data_evaluation/
|   +-- study_controller.py
|   +-- data_evaluation.py
|   +-- data_visualization.py
|   +-- questionnaire_evaluation.py
+-- launch/
    +-- teleop_launch.py
```

### Bridge

The bridge maps Haply device movement and stylus buttons to da Vinci PSM1
commands through the dVRK CRTK interface.

`haply_daVinci_bridge_node_jointcontrol.py` is the main teleoperation bridge.
It subscribes to Haply state and PSM1 joint state, calibrates the initial Haply
pose against the current robot joint configuration, then sends joint targets to
the PSM1.

Main behavior:

- Reads `haply_msgs/HaplyState` from `haply_state`
- Reads PSM1 joints from `/PSM1/measured_js`
- Maps Haply translation and orientation offsets to PSM1 joint targets
- Uses Button A to toggle the gripper
- Uses Button B to recalibrate the translational reference position
- Handles continuous yaw unwrapping to avoid jumps around +/- pi

Run manually:

```bash
ros2 run haply_daVinci_teleop haply_daVinci_bridge_node_jointcontrol
```

`haply_daVinci_bridge_node_baseframe.py` is an experimental Cartesian-control
bridge. It is not installed as a console script in the current `setup.py`, so it
is mainly kept as reference/development code.

### Haptic Visualization

The haptic visualization nodes build the digital representation of the hotwire
task. They publish RViz markers, track the loop attached to the gripper, compute
the distance between loop and wire, and publish Haply force commands.

`haptic_hotwire.py`:

- Visualizes the wire as RViz cylinder markers
- Computes force feedback from the loop-to-wire distance
- Publishes force commands on `haply_target`
- Publishes contact status on `contact_status`
- Supports three force feedback modes through the `mode` ROS parameter
- Can measure wire points when `measure_wirepoints_mode` is enabled in the code

Run manually:

```bash
ros2 run haply_daVinci_teleop haptic_hotwire --ros-args -p mode:=1
```

`haptic_loop.py`:

- Visualizes the loop attached to the PSM1 gripper
- Publishes the calculated loop center on `loop_center`
- Uses `/PSM1/local/measured_cp` and `/PSM1/jaw/measured_js`
- Publishes a static `world -> PSM1_base` transform for RViz visualization

Run manually:

```bash
ros2 run haply_daVinci_teleop haptic_loop
```

### Study Controller

`study_controller.py` tracks the progress of the hotwire task by checking
whether the loop center enters a sequence of tolerance regions. It publishes the
current trial state and RViz markers for the target regions.

Run manually:

```bash
ros2 run haply_daVinci_teleop study_controller
```

Trial states:

- `IDLE`
- `RUNNING1`
- `RUNNING2`
- `RUNNING3`
- `RUNNING4`
- `RUNNING5`
- `FINISHED`

### Contact Detection

For the physical setup, `ContactDetection.ino` can be flashed to an ESP32 to
count electrical contact events between the loop and the wire.

The package also contains `simulated_contact_detection.py`, which is intended
for marker-based contact estimation during simulation/development. It is not
currently installed as a console script.

## Nodes and Interfaces

### Installed ROS 2 executables

The following console scripts are installed by this package:

```bash
ros2 run haply_daVinci_teleop haply_daVinci_bridge_node_jointcontrol
ros2 run haply_daVinci_teleop haptic_hotwire
ros2 run haply_daVinci_teleop haptic_loop
ros2 run haply_daVinci_teleop study_controller
```

### Main launch file

The package provides one launch file:

```bash
ros2 launch haply_daVinci_teleop teleop_launch.py mode:=1
```

This starts:

- `haptic_hotwire`
- `haply_daVinci_bridge_node_jointcontrol`
- `haptic_loop`
- `study_controller`

The `mode` launch argument is passed to `haptic_hotwire`.

### Important topics

Subscribed topics:

- `haply_state` (`haply_msgs/HaplyState`)
- `/PSM1/measured_js` (`sensor_msgs/JointState`)
- `/PSM1/local/measured_cp` (`geometry_msgs/PoseStamped`)
- `/PSM1/jaw/measured_js` (`sensor_msgs/JointState`)
- `loop_center` (`geometry_msgs/PoseStamped`)
- `/trial/state` (`std_msgs/String`)

Published topics:

- `haply_target` (`haply_msgs/HaplyControl`)
- `hotwire_marker` (`visualization_msgs/Marker`)
- `loop_marker` (`visualization_msgs/Marker`)
- `loop_center` (`geometry_msgs/PoseStamped`)
- `contact_status` (`std_msgs/Bool`)
- `/trial/state` (`std_msgs/String`)
- `/trial/tolerance_marker` (`visualization_msgs/Marker`)

## Build and Source

From the workspace root:

```bash
cd ~/haply_ros2_interface
colcon build --packages-select haply_daVinci_teleop
source install/setup.bash
```

If the package was already built, sourcing the workspace is enough:

```bash
cd ~/haply_ros2_interface
source install/setup.bash
```

## Running the System

### 1. Start the da Vinci system

Start the dVRK system from the dVRK configuration workspace. The exact path can
differ between machines and is not part of this repository.

Example:

```bash
cd ~/ros2_ws/src/dvrk/dvrk_config_oe
ros2 run dvrk_robot dvrk_system -j OE-daVinci/system-MTMR-PSM1-Teleop.json
```

After the GUI opens, press **Start** and home the da Vinci system.

### 2. Start the cameras

For visual feedback, use the da Vinci camera system and goggles if available.

Camera 1:

```bash
gst-launch-1.0 decklinkvideosrc mode=pal device-number=0 ! videorate ! "video/x-raw,framerate=30/1" ! glimagesink
```

Camera 2:

```bash
gst-launch-1.0 decklinkvideosrc mode=pal device-number=1 ! videorate ! "video/x-raw,framerate=30/1" ! glimagesink
```

### 3. Calibrate the Haply device

Use the Haply software to calibrate the Inverse3 and VerseGrip. The path can
differ between machines.

Example:

```bash
cd ~/Downloads/squashfs-root
./AppRun
```

Follow the Haply calibration instructions. Make sure the Inverse3 and VerseGrip
are physically aligned before starting teleoperation.

### 4. Start the Haply driver

Start the Haply driver from the workspace:

```bash
cd ~/haply_ros2_interface
source install/setup.bash
ros2 run haply_interface haply_driver_node
```

The driver should publish `haply_state` and subscribe to `haply_target`.

### 5. Launch the teleoperation package

Recommended startup:

```bash
cd ~/haply_ros2_interface
source install/setup.bash
ros2 launch haply_daVinci_teleop teleop_launch.py mode:=1
```

Available force feedback modes:

- `mode:=0`: no force feedback
- `mode:=1`: linear force feedback
- `mode:=2`: stepwise force feedback

Manual startup alternative:

```bash
ros2 run haply_daVinci_teleop haptic_hotwire --ros-args -p mode:=1
ros2 run haply_daVinci_teleop haptic_loop
ros2 run haply_daVinci_teleop study_controller
ros2 run haply_daVinci_teleop haply_daVinci_bridge_node_jointcontrol
```

## Hotwire Experiment Workflow

### Measure the wire points

For accurate force feedback, the digital wire should match the physical wire.
The hotwire node can measure wire edge points using Button C on the VerseGrip.

Current limitation: wire measurement is controlled by the hardcoded variable
`self.measure_wirepoints_mode` in `haptic_hotwire.py`.

To measure new wire points:

1. Open `haply_daVinci_teleop/haptic_visualization/haptic_hotwire.py`.
2. Set:

   ```python
   self.measure_wirepoints_mode = True
   ```

3. Start the hotwire node:

   ```bash
   ros2 run haply_daVinci_teleop haptic_hotwire --ros-args -p mode:=0
   ```

4. Move the gripper to each wire edge point and briefly press Button C.

Press Button C only briefly. A long press can trigger recalibration behavior on
the Haply device.

When using predefined points, keep:

```python
self.measure_wirepoints_mode = False
```

and update `self.given_wirepoints` if the physical wire geometry changes.

### Run a trial

1. Start the da Vinci system and cameras.
2. Calibrate the Haply device.
3. Start the Haply driver.
4. Launch this package with the desired force feedback mode.
5. Open RViz if visualization is needed.
6. Move the loop to the start point to enter `RUNNING1`.
7. Follow the hotwire path until the state reaches `FINISHED`.
8. Move to the reset point to return to `IDLE`.

Be careful when enabling force feedback. The Haply can apply forces toward or
away from the virtual wire based on the loop position. Make sure the physical
loop is attached and the digital wire is aligned before running a participant
trial.

## Controls

VerseGrip buttons:

- Button A: toggle PSM1 gripper open/closed
- Button B: recalibrate the Haply translational reference for teleoperation
- Button C: add wire measurement point when wire measurement mode is enabled

## Contact Counting

### ESP32 contact detection

For physical contact counting, flash `contact_detection/ContactDetection.ino`
to an ESP32 using the Arduino IDE or PlatformIO.

Hardware setup:

- Connect the hotwire to GPIO 25 through a 100 ohm series resistor.
- Connect the loop to ground.
- Open the serial monitor at 115200 baud.

The ESP32 counts a contact when the circuit is closed and the input pin reads
LOW. The script uses a 0.5 second lockout to avoid counting one long touch
multiple times.

Example command to start the Arduino IDE:

```bash
cd ~/Downloads
./arduino-ide_2.3.6_Linux_64bit.AppImage --no-sandbox
```

### ROS contact status

The hotwire node also publishes `contact_status` as `std_msgs/Bool`. This is
computed from the digital loop and wire geometry.

## Data Recording

For experiments, record the relevant ROS topics with rosbag.

Example:

```bash
ros2 bag record -o /home/lorant/Desktop/Haply_study_rosbags/Participant5_mode0_round2 \
  /haply_target \
  /hotwire_marker \
  /haply_state \
  /loop_center \
  /PSM1/local/measured_cp \
  /contact_status \
  /trial/state
```

This records Haply state, computed forces, wire visualization, loop position,
PSM1 Cartesian pose, contact status, and trial state.

## Troubleshooting

### Da Vinci stops moving

If Cartesian or joint control still works, one of the following is likely:

- The robot reached a workspace or joint limit. Rotate or move the Haply back
  toward the calibrated neutral pose. The Haply can rotate beyond 360 degrees,
  but the da Vinci cannot.
- The VerseGrip powered off. Shut down the bridge, Haply driver, and da Vinci
  control. Restart the VerseGrip, recalibrate it, check the battery level, home
  the da Vinci, then start the Haply driver and bridge again.

If control no longer works:

- Check whether the Haply driver reported a connection or device error.
- Check whether the da Vinci disabled itself because a target was outside the
  valid workspace.
- Restart in this order: Haply driver, da Vinci system, teleoperation bridge.
- Make sure the da Vinci and Haply are both homed and physically aligned.

### Da Vinci movements are mirrored

This is usually caused by incorrect homing or physical alignment.

Recommended recovery:

1. Stop the bridge.
2. Stop the Haply driver.
3. Re-home and recalibrate the Haply.
4. Place the VerseGrip so the buttons face the user.
5. Home the da Vinci again.
6. Make sure the gripper is centered and within its rotational range.
7. Restart the Haply driver and bridge.

### Force feedback feels wrong

Check the following:

- The measured or predefined wire points match the real wire.
- The loop is attached correctly in the gripper.
- The gripper state is detected correctly through `/PSM1/jaw/measured_js`.
- The trial state is one of `RUNNING1` to `RUNNING5`; otherwise forces are
  intentionally disabled.
- The selected mode is correct.

## Pre-study Learnings

### 1. Explain the teleoperation device clearly

Training efficiency strongly depends on how the teleoperation system is
introduced. It is recommended to explain that the setup is experimental and not
fully equipped with safety limits. This helps participants move more calmly and
avoid fast motions near workspace boundaries.

### 2. Give one clear task objective

How the task is framed affects participant strategy. If participants are told
to be both fast and collision-free, they may prioritize one objective
differently. A single clear goal leads to more comparable behavior.

### 3. Allow enough training time

Practice with and without force feedback is important. Participants often rely
heavily on visual feedback, and haptic cues can feel unintuitive at first. With
too little familiarization, results may mostly show learning effects instead of
the effect of force feedback.

### 4. Reduce deviations between simulation and reality

The virtual wire is modeled as straight segments between edge points, but the
real wire can bend and may not align perfectly. The loop center can also shift
if the gripper does not pick up the loop consistently. A straight wire, careful
wire-point measurement, and consistent loop grasping improve the force-feedback
quality.
