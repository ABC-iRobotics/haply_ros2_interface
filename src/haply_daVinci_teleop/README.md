# Haply-DaVinci Teleoperation

This package provides a ROS2 teleoperation bridge for controlling the **daVinci PSM1 arm** with the **Haply Inverse3** and the **VerseGrip Stylus (Handle)**. Further it provides the use of Haptic Forcefeedback for conducting a hot wire task. Useable both in simulation and on the real system.
This Readme will also lead you through the process of setting up a hotwire experiment using force-feedback.

This implementation was tested on:
- Windows 11 (WSL2 Ubuntu 22.04 LTS (64-bit))
- Ubuntu 24.04.3 LTS
- Python 3.10.6

## Contents
- [Package structure](#package-structure)
    - [Bridge](#bridge)
    - [Haptic visualization](#haptic_visualization)
    - [Contact detection](#contact-detection)
- [Setup the system](#setup-the-system)
    - [Start daVinci](#start-davinci)
    - [Start cameras](#start-cameras)
    - [Haply caibration](#haply-calibration)
    - [Start Haply and Bridge](#start-haply-and-bridge)
    - [Make measurements](#make-measurements)
    - [Conducting the hotwire task](#conducting-the-hotwire-task)
    - [Count the number of touches](#count-the-number-of-touches)
- [Troubleshooting](#troubleshooting)
- [Pre-study Learnings](#pre-study-learnings)

## Package structure

### Bridge:
The Bridge is used to transform the Movements of the Haply to the Movements of the daVinci PSM1 arm. It also implemented a clutch feature and steering of the gripper. The use the dvrk crtk messages. For further information visit this website: link

 #### haply_daVinci_bridge_node_jointcontrol.py
 ---
This ROS2 node maps Haply haptic device inputs to da Vinci PSM1 robot joint commands using the CRTK interface.

#### Features
- Reads HaplyState for position, orientation and button presses
- Reads da Vinci joint states via /PSM1/measured_js
- Computes Cartesian and rotational deltas relative to a calibrated reference position
- Sends joint commands using CRTK servo_jp and servo_cp
- Button A: toggles gripper open/close
- Button B: clutching (reset reference position without full recalibration)

#### Usage
    ros2 run <package_name> haply_to_davinci_bridge.py
    
#### Workflow
1. Node starts, da Vinci arm is enabled and homed.
2. First received HaplyState sets calibration (position + orientation).
3. Haply motion is translated into da Vinci joint commands.
4. Button B resets reference position without affecting orientation (clutching).
5. Button A toggles gripper open/close. 

#### Notes
- Calibration occurs once at startup when the first HaplyState arrives.
- Rotation is unwrapped to avoid sudden jumps on yaw.
- Motion/rotation scaling factors are easily adjustable in the script.

#### haply_daVinci_bridge_node_baseframe.py (experimental)
---

This ROS2 node uses Cartesian control and maps Haply position/orientation offsets into daVinci end-effector motion in PSM1 base frame.

#### Features
- PSM control from Haply Cartesian motion
- Calibration via Haply Button B
- First B press → position + orientation reference
- Subsequent B press → position-only recalibration
- Haply Button A toggles gripper open/close
- Adjustable scaling factors

#### Usage
```bash
ros2 run haply_daVinci_teleop haply_to_davinci_pose.py
```

#### Workflow
1) Wait for first Haply and daVinci pose
2) Button B: calibration (full then position-only)
3) Motion input: pose delta: sent to PSM1
4) Button A: toggles jaw state
---

### Haptic_visualization
The haptic visualization has two main tasks: It calculates a digital twin of a real wire by using measurements of the edge points of the wire path. Further it calculates the loopcenter and orientation out of the gripper pose. 
Further a force feedback is calculated and send to the Haply.

#### haptic_hotwire.py
---

#### Overview
This ROS2 node visualizes a straight wire (as a cylinder) and provides haptic feedback for the Hot Wire experiment using a Haply device. It also allows measuring and publishing the position of a loop moving along the wire.

#### Features
- Visualizes wire segments in RViz using `visualization_msgs/Marker`.
- Computes haptic forces for Haply based on distance from loop to wire.
- Publishes haptic commands using `haply_msgs/HaplyControl`.
- Supports measuring wire points interactively via Haply Button C.
- Configurable force feedback modes: off, linear, or stepwise.
- Supports predefined or measured wirepoints.

#### Publishers
- `hotwire_marker` (`Marker`): Visualizes wire segments in RViz.
- `haply_target` (`HaplyControl`): Publishes haptic force commands to Haply.

#### Usage
1. Launch ROS2 and ensure the Haply device is connected.
2. Start the HotWire node:
    ```bash
    ros2 run <package_name> hot_wire_node
    ```

#### haptic_loop.py
---

#### Overview
This ROS2 node visualizes a loop attached to the PSM1 gripper using a single `Marker` in RViz and publishes the loop's center pose. It is designed for haptic interaction experiments, allowing visualization and tracking of the loop in the world frame.

#### Features
- Visualizes the loop in RViz as a LINE_STRIP marker.
- Publishes the loop's center pose as `PoseStamped` on `loop_center`.
- Automatically transforms gripper pose to the world frame for consistent visualization.
- Supports loop radius, thickness, offsets, and segmentation configuration.
- Applies additional rotation to align the loop plane with the gripper jaws.
- Updates visualization in real-time at high frequency (1 kHz timer).

#### Publishers
- `visualization_marker` (`Marker`): Visualizes the loop in RViz.
- `loop_center` (`PoseStamped`): Publishes the center position and orientation of the loop.

#### Usage
1. Launch ROS2 and ensure the PSM1 gripper node is running.
2. Start the HapticLoop node:
   ```bash
   ros2 run <package_name> haptic_loop_node
    ```
---
### Contact Detection
To detect loop-wire contacts it is recommended to use a ESP32 development board. The following script can be flashed on the ESP board using the Arduino IDE.

#### ContactDetection.ino
---

#### Overview
This script runs on an ESP32 and detects contacts with a hotwire using a digital input pin. It counts the number of contacts and prints the information over the serial port.

#### Hardware
- ESP32 microcontroller
- Hotwire connected to GPIO 25 (configured with internal pull-up resistor) and a 100 ohm resistor in series
- Loop connected to Ground

#### Features
- Detects contact when the hotwire circuit is completed (pin reads LOW).
- Implements a 0.5-second lockout to prevent multiple counts from a single touch.
- Counts and prints the number of contacts over the serial interface.

#### Usage
1. Connect the hotwire to GPIO 25 on the ESP32.
2. Upload the script to the ESP32 using Arduino IDE or PlatformIO.
3. Open the Serial Monitor at 115200 baud to view contact detection messages.
4. Touch the hotwire to see contact counts increment in the serial output.

#### Notes
- The script uses a simple debounce/lockout mechanism to prevent multiple triggers from a single touch.
- Adjust the lockout duration by changing the `500` ms value in the `if` statement if needed.
- The script uses a 10 ms loop delay to reduce CPU usage.
---

## Setup the system
### Start daVinci: 
To prepare the workspace for conducting a hotwire experiment on the real daVinci system you will need to turn on the daVinci. Using the terminal in the ubuntu os you can use following commands to do that. Be aware that this can change due to it's not part of this repo:

    cd ~/ros2_ws/src/dvrk/dvrk_config_oe
    ros2 run dvrk_robot dvrk_system -j OE-daVinci/system-MTMR-PSM1-Teleop.json

After opening the GUI press start and home the daVinci system
    
### Start cameras:
As visual Feedback it is recommended to use the camera system of the daVinci Research Kit System and the Googles for better depth perception. You can start the cameras using the following commands.

Camera1:

    gst-launch-1.0 decklinkvideosrc mode=pal device-number=0 ! videorate ! "video/x-raw,framerate=30/1" ! glimagesink

Camera2:

    gst-launch-1.0 decklinkvideosrc mode=pal device-number=1 ! videorate ! "video/x-raw,framerate=30/1" ! glimagesink

### Haply calibration:
After setting up the daVinci you will need to calibrate the Haply device. Therfore it is recommended to use the Haply software. You can open it by using the following commands. Be aware that the path can differ.
    
    cd ~/Downloads/squashfs-root
    ./AppRun
    
Please follow the instructions for calibrating the Haply and the Inverse grip.
    
### Start Haply and Bridge:
For using the Haply for the controle of the daVinci PSM1 arm please start its driver and the haply control bridge using the following commands.

Run Haply driver:

    cd ~/haply_ros2_interface
    ros2 run haply_interface haply_driver_node
 
Run Haply-daVinci bridge

    cd ~/haply_ros2_interface
    ros2 run haply_daVinci_teleop haply_bridge_jointcontrol


### Make measurements:
For conducting the hotwire task with forcefeedback it is crucial that the simulated wire fits with the position of the real wire. Therefore you need to measure the edge points of the wire by starting the haptic hotwire script in mode 0 and set the variable self.measure_wirepoints_mode = True.

Run Haptic Hotwire:

    cd ~/haply_ros2_interface/src/haply_daVinci_teleop/haply_daVinci_teleop/visualization
    python3 haptic_hotwire.py

After starting the script you will be able to measure the wire edges by pressing Button C of the Inverse3 pen. Be aware to only press it for a short time because pressing long leads to a recalibration of the Inverse3 pen. For measuring open the gripper with Button A, lead it to the wire edge and press Button C. 

### Conducting the hotwire task:
For conducting the task you can currently choose between three different modes:

- Without force feedback **mode0**
- With force feedback **mode1**
- With force feedback **mode2**

Choose the mode by manipulate the python script (needs to be changed!).
After selection of the mode, start the haptic wire node by using the following command:

    cd ~/haply_ros2_interface/src/haply_daVinci_teleop/haply_daVinci_teleop/visualization
        python3 haptic_hotwire.py

You will also need to start the haptic loop node for Force Feedback calculation:

    cd ~/haply_ros2_interface
		ros2 run haply_daVinci_teleop haptic_loop

Be aware, as soon as you start force feedback the manipulator will be pushed towards the wire if it's getting close. That is due to the fact, that it can't detect if a loop is attached!

### Count the number of touches:
For counting the number of touches you can use an **ESP32 Development Board** using the script **Contact Detection** or just open the Arduino IDE with:

    cd ~/Downloads
		./arduino-ide_2.3.6_Linux_64bit.AppImage --no-sandbox	

To run the ESP32 connect pin G25 with a resistance (100 ohm recommended) in series and connect it with the hotwire (using a normal wire). The loop must be connected with Ground. After uploading the Software you will be able to see the number of touches as Serial Output.

- Optinaly start a rosbag to collect data:
For collecting data it is recommended to use a rosbag. You can start one with the following command:
    ```bash
    ros2 bag record -o /home/lorant/Desktop/Haply_study_rosbags/Participant5_mode0_round2 /haply_target /hotwire_marker /haply_state /loop_center /PSM1/local/measured_cp
    ```

This rosbag records the positions of loop, wire and robot as well es the Haply data which also contains the calculated force.

## Troubleshooting:
### 1) DaVinci stops moving: 
- Check if you can still control the position:
    - if yes: There are two known possibilities:
        - You likely reached the workspace limit of the DaVinci end effector.
        No worries, simply rotate the Inverse3 back. The Inverse3 has no rotation limit and can exceed 360°, while the DaVinci cannot. If the Inverse3 is rotated too far in one direction, it sends positions outside the DaVinci workspace, causing the system to stop responding.

        - The InverseGrip likely powered off.
        In this case, shut down the DaVinci, the Haply driver, and the haply_daVinci_bridge. Restart the Inverse3, recalibrate it, and check the battery level. If everything looks good, reattach it to the Haply, home the DaVinci, then start the DaVinci, the Haply driver, and finally the haply_daVinci_bridge again.

    - if no: Check the following:
        - Check if the haply driver threw an error:
        Likely a wire connection issue. Power down the DaVinci and the haply_daVinci_bridge, then restart them in this order: haply_driver, DaVinci, haply_daVinci_bridge. Ensure the DaVinci was homed correctly and the Haply is manually aligned.

        - Check if the daVinci turned off:
        Most likely the Haply transmitted a pose/rotation outside of the DaVinci workspace, often caused by fast or aggressive movements, or a short connection dropout. You can resolve this by shutting down the haply_daVinci_bridge, homing and restarting the DaVinci, and then relaunching the haply_daVinci_bridge. Verify again that both DaVinci and Haply are homed and aligned properly.

### 2) DaVinci movements are mirrored:
- Most likely the DaVinci and Haply were not homed correctly.
Turn off the DaVinci, haply_daVinci_bridge, and the Haply driver. Home the Haply and the Inverse3 again, the Haply must be physically aligned, and the Inverse3 should be oriented so that the buttons face toward the user. The DaVinci needs to be homed as well. Also ensure that the gripper is centered and positioned within its rotational range.
   

## Pre-study Learnings:
1) Explaining the teleop device: 
Training efficiency strongly depends on how the teleoperation system is introduced. It is recommended to clearly communicate that the setup is experimental and not fully equipped with safety limits. This tends to result in calmer, more controlled user behaviour and helps participants build confidence without rushing. By reducing stress and setting realistic expectations, users understand the system faster, adapt more naturally, and are less frustrated when unexpected behaviour occurs, especially near workspace boundaries where fast movements can trigger errors.

2) Task explanation: 
How the task is framed shapes user strategy. If you present multiple objectives"as fast as possible and with as few collisions as possible" users will instinctively prioritise one. To keep behaviour consistent and comparable, it’s better to specify a single clear goal rather than giving multiple competing targets.

3) Training time:
Sufficient practice time, both with and without force feedback, is essential to actually observe the benefit of haptics. Participants often rely heavily on visual information, and unexpected force cues can feel counter-intuitive at first. If a force pushes them away from what they believe is the correct position, they tend to fight against it, which causes confusion and poor performance. Without proper familiarisation time, the results mostly reflect learning effects rather than the impact of force feedback itself.

4) Deviations between simulation and reality:
The simulated wire is modelled as a straight segment between edge points, but real wires bend, curve and rarely align perfectly. The loop’s centre can also shift if the gripper doesn’t pick it up precisely. These geometric differences cause force inaccuracies and position errors. Using a wire that is as straight as possible, and ensuring the gripper engages the loop cleanly and consistently, significantly reduces these deviations.
