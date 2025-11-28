# Haply-DaVinci Teleoperation

This package provides a ROS2 teleoperation bridge for controlling the **daVinci PSM1 arm** with the **Haply Inverse3** and the **VerseGrip Stylus (Handle)**. Further it provides the use of Haptic Forcefeedback for conducting a hot wire task. Useable both in simulation and on the real system.
This Readme will also lead you through the process of conducting a hotwire experiment using the **Haply** with forcefeedback.

This implementation was tested on:
- Windows 11 (WSL2 Ubuntu 22.04 LTS (64-bit))
- Ubuntu 24.04.3 LTS
- Python 3.10.6

## Node explanation
Bridge:
The Bridge is used to transform the Movements of the Haply to the Movements of the daVinci PSM1 arm. It also implemented a clutch feature and steering of the gripper. The use the dvrk crtk messages. For further information visit this website: link
 ### haply_daVinci_bridge_node_jointcontrol.py
This ROS2 node maps Haply haptic device inputs to da Vinci PSM1 robot joint commands using the CRTK interface.

#### Features
- Reads HaplyState for position, orientation and button presses
- Reads da Vinci joint states via /PSM1/measured_js
- Computes Cartesian and rotational deltas relative to a calibrated reference position
- Sends joint commands using CRTK servo_jp and servo_cp
- Button A → toggles gripper open/close
- Button B → clutching (reset reference position without full recalibration)

#### Run
        ros2 run <package_name> haply_to_davinci_bridge.py

#### Requirements
    rclpy  
    haply_msgs  
    sensor_msgs  
    crtk  
    PyKDL  
    numpy  

#### Workflow
1. Node starts, da Vinci arm is enabled and homed.
2. First received HaplyState sets calibration (position + orientation).
3. Haply motion is translated into da Vinci joint commands.
4. Button B resets reference position without affecting orientation (clutching).
5. Button A toggles gripper open/closed.

#### Topics
Subscribed:
    /PSM1/measured_js     (sensor_msgs/JointState)  – da Vinci joint feedback
    haply_state           (haply_msgs/HaplyState)   – haptic input + button states

#### Mapping Overview
- X, Y, Z → PSM1 translation joints (scaled)
- Roll, Pitch, Yaw → rotational joints (yaw supports continuous unwrap)
- Button A → gripper toggle
- Button B → clutch reference reset

#### Notes
- Calibration occurs once at startup when the first HaplyState arrives.
- Rotation is unwrapped to avoid sudden jumps on yaw.
- Motion/rotation scaling factors are easily adjustable in the script.
"""


### haply_daVinci_bridge_node_baseframe.py (experimental)

This ROS2 node provides a real-time teleoperation bridge between the Haply haptic device and the daVinci surgical robot (PSM1) via CRTK.
It uses Cartesian control instead of joint-space and maps Haply position/orientation offsets into daVinci end-effector motion.

#### Features
- PSM control from Haply Cartesian motion
- Calibration via Haply Button B
- First B press → position + orientation reference
- Subsequent B press → position-only recalibration
- Haply Button A toggles gripper open/close
- Adjustable scaling factors

#### Subscribed Topics
- /haply_state: (HaplyState)     Input device pose + buttons
- /PSM1/measured_cp: (PoseStamped)    Live daVinci pose feedback

#### CRTK Commands
- servo_cp(): Cartesian motion command to arm
- servo_jp(): Gripper open/close

#### Control Logic
- Wait for first Haply + daVinci pose
- Button B = calibration (full then position-only)
- Motion input → pose delta → sent to PSM1
- Button A toggles jaw state

#### Run
```bash
ros2 run haply_daVinci_teleop haply_to_davinci_pose.py
```

Haptic Visualization:
###haptic_hotwire.py


#### Overview
This ROS2 node visualizes a straight wire (as a cylinder) and provides haptic feedback for the Hot Wire experiment using a Haply device. It also allows measuring and publishing the position of a loop moving along the wire.

#### Features
- Visualizes wire segments in RViz using `visualization_msgs/Marker`.
- Computes haptic forces for Haply based on distance from loop to wire.
- Publishes haptic commands using `haply_msgs/HaplyControl`.
- Supports measuring wire points interactively via Haply Button C.
- Configurable force feedback modes: off, linear, or stepwise.
- Supports predefined or measured wirepoints.

Topics

#### Subscribers
- `/haply_state` (`HaplyState`): Current state of the Haply device.
- `loop_center` (`PoseStamped`): Position of the loop tip to compute force.
- `/PSM1/local/measured_cp` (`PoseStamped`): Measured Cartesian pose of the robot end-effector.

#### Publishers
- `hotwire_marker` (`Marker`): Visualizes wire segments in RViz.
- `haply_target` (`HaplyControl`): Publishes haptic force commands to Haply.

#### Parameters / Configuration
- `loop_radius`: radius of the loop for effective force calculation (m)
- `distance_tip_to_PSM1_base`: distance from Haply tip to robot base (m)
- `force_feedback_mode`: 0 = off, 1 = linear, 2 = linear step
- `stiffness` and `damping`: Haptic parameters
- `measure_wirepoints_mode`: `True` to record wirepoints using Haply button, `False` to use predefined points
- `given_wirepoints`: Default wire coordinates if not measuring interactively

#### Usage
1. Launch ROS2 and ensure the Haply device is connected.
2. Start the HotWire node:
    ```bash
    ros2 run <package_name> hot_wire_node
    ```



### haptic_loop.py


#### Overview
This ROS2 node visualizes a loop attached to the PSM1 gripper using a single `Marker` in RViz and publishes the loop's center pose. It is designed for haptic interaction experiments, allowing visualization and tracking of the loop in the world frame.

#### Features
- Visualizes the loop in RViz as a LINE_STRIP marker.
- Publishes the loop's center pose as `PoseStamped` on `loop_center`.
- Automatically transforms gripper pose to the world frame for consistent visualization.
- Supports loop radius, thickness, offsets, and segmentation configuration.
- Applies additional rotation to align the loop plane with the gripper jaws.
- Updates visualization in real-time at high frequency (1 kHz timer).

#### Subscribers
- `/PSM1/local/measured_cp` (`PoseStamped`): Current Cartesian pose of the PSM1 gripper.

#### Publishers
- `visualization_marker` (`Marker`): Visualizes the loop in RViz.
- `loop_center` (`PoseStamped`): Publishes the center position and orientation of the loop.

#### Parameters / Configuration
- `loop_radius`: Radius of the loop in meters (default 0.011 m).
- `loop_segments`: Number of segments to approximate the circular loop (default 10).
- `loop_thickness`: Diameter of the loop cylinders for visualization (default 0.002 m).
- `loop_x_offset`, `loop_y_offset`, `loop_z_offset`: Offsets along gripper axes to adjust loop position.
- Optional rotation applied around gripper X-axis to align loop plane (default 90°).

#### Usage
1. Launch ROS2 and ensure the PSM1 gripper node is running.
2. Start the HapticLoop node:
   ```bash
   ros2 run <package_name> haptic_loop_node
    ```


### ContactDetection.ino: 

#### Overview
This script runs on an ESP32 and detects contacts with a hotwire using a digital input pin. It counts the number of contacts and prints the information over the serial port.

#### Hardware
- ESP32 microcontroller
- Hotwire connected to GPIO 25 (configured with internal pull-up resistor)

#### Features
- Detects contact when the hotwire circuit is completed (pin reads LOW).
- Implements a 0.5-second lockout to prevent multiple counts from a single touch.
- Counts and prints the number of contacts over the serial interface.
- Simple and lightweight, suitable for real-time haptic or game applications.

#### Pin Configuration
- `hotwirePin` (GPIO 25): Input pin for the hotwire, set as `INPUT_PULLUP`.

### Serial Output
- Prints `"Contact!"` each time a new contact is detected.
- Prints the total number of contacts, e.g., `Number of contacts: 3`.

#### Usage
1. Connect the hotwire to GPIO 25 on the ESP32.
2. Upload the script to the ESP32 using Arduino IDE or PlatformIO.
3. Open the Serial Monitor at 115200 baud to view contact detection messages.
4. Touch the hotwire to see contact counts increment in the serial output.

#### Notes
- The script uses a simple debounce/lockout mechanism to prevent multiple triggers from a single touch.
- Adjust the lockout duration by changing the `500` ms value in the `if` statement if needed.
- The script uses a 10 ms loop delay to reduce CPU usage without affecting responsiveness.


## Using the system
### start daVinci: 
To prepare the workspace for conducting a hotwire experiment on the real daVinci system you will need to turn on the daVinci. Using the terminal in the ubuntu os you can use following commands to do that. Be aware that this can change due to it's not part of this repo:
    ```bash
    cd ~/ros2_ws/src/dvrk/dvrk_config_oe
    ros2 run dvrk_robot dvrk_system -j OE-daVinci/system-MTMR-PSM1-Teleop.json
    ```

    After opening the GUI press start and home the daVinci system
    
### start cameras:
As visual Feedback it is recommended to use the camera system of the daVinci Research Kit System and the Googles for better depth perception. You can start the cameras using the following commands:
    ```bash 
    gst-launch-1.0 decklinkvideosrc mode=pal device-number=0 ! videorate ! "video/x-raw,framerate=30/1" ! glimagesink
    ```
    ```bash 
    gst-launch-1.0 decklinkvideosrc mode=pal device-number=1 ! videorate ! "video/x-raw,framerate=30/1" ! glimagesink
    ```
### Haply calibration:
After setting up the daVinci you will need to calibrate the Haply device. Therfore it is recommended to use the Haply software. You can open it by using the following commands. Be aware that the path can differ.
    ```bash
    cd ~/Downloads/squashfs-root
    ./AppRun
    ```
    Please follow the instructions for calibrating the Haply and the Inverse grip.
    
### start haply and bridge:
For using the Haply for the controle of the daVinci PSM1 arm please start its driver and the haply control bridge using the following commands:
    ```bash
    cd ~/haply_ros2_interface
    ros2 run haply_interface haply_driver_node
    ```
    ```bash
    cd ~/haply_ros2_interface
    ros2 run haply_daVinci_teleop haply_bridge_jointcontrol
    ```

### Make measurements:
For conducting the hotwire task with forcefeedback it is crucial that the simulated wire fits with the position of the real wire. Therefore you need to measure the edge points of the wire by starting the haptic hotwire script in mode 0 and set the variable self.measure_wirepoints_mode = True 

    cd ~/haply_ros2_interface/src/haply_daVinci_teleop/haply_daVinci_teleop/visualization
    python3 haptic_hotwire.py
    mode 0, self.measure_wirepoints_mode = True

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
`

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

## Troubleshoot:
### 1) Suddenly the daVinci doesn't move anymore: 
- Check if you can still control the position:
    - if yes: There are two known possibilities:
        - you reached the edge of workspace of the daVinci endeffector:
        No worries, you can fix the problem by just rotating the Inverse3 back. There is no limit set, so you can rotate the Inverse3 more than 360 degrees while the daVinci is limited there. If you rotate more than 360 degrees in one direction the Inverse3 is sending a position which is outside the daVinci space so the daVinci won't react to that.
        - The InverseGrip switched off:
        Please turn off the daVinci, haply driver and haply_daVinci_bridge. Turn on the Inverse3 again and recalibrate it. Check the battery status. If it's fine just attach it again to the Haply, home the daVinci, start the daVinci, start the Haply driver and start the haply_daVinci bridge.  Make sure you correctly homed the daVinci and also aligned the Haply manually!
    - if no: Check the following:
        - Check if the haply driver threw an error:
        Most likely there was a wire connection error. Just turn off the daVinci and the haply_daVinci bridge and turn them on again in the following order: haply_driver, daVinci, haply_daVinci_bridge. Make sure you correctly homed the daVinci and also aligned the Haply manually!
        - Check if the daVinci turned off:
        Most likely the Haply sent a position/rotation which was too far away from the actual daVinci position. This can happen when the user did too aggressive/fast movements or there was a short connection interruption. No worries you can fix this by turn off the haply_daVinci bridge, home and restart the daVinci and after also restart the Haply_daVinci_bridge. Make sure you correctly homed the daVinci and also aligned the Haply manually!  
### 2) The Steering of the DaVinci is mirrored:
- Most likely the daVinci and Haply wasn't home properly. 
Just turn off the daVinci, haply_daVinci_bridge and haply driver. Please home the Haply and Inverse3. The Haply should be aligned and the Inverse3 should oriented such so that the buttons look into the direction to the user. The daVinci must be homed as well. Make sure the Gripper is aligned and also in the middle of it's rotatinal space.    

		
## Learnings from pre-study:
1) Depending on how you explain the teleop device: the training time can be more efficient: Better say that it's an experimental setup which hasn't implemented all safety limits leads to a more calm and smoother behaviour of the participants. This will help to get a better understanding for the system and a faster process in getting used to the new teleoperation device. Also less frustration because of unexpected errors which can be caused by fast movements on the edge of the endeffector space.

2) Depending on how you explain the task: 
You can say the goal of the task is to reach the end:
    - as fast as possible and with as less touches as possible
    - as fast as possible
    . with as less touche as possible
It's recommended to only give one goal because otherwise the participants will choose one where they put their focus on

3) Training time:
Giving enough training time with and without force feedback is crucial for exploring the advantage of force feedback. Participants lead to the behaviour to trust more in their visual perception. If a force pushes you out of your subjective correct position you will try to work against this force. That leads to confusion and a worse result with force feedback. 
If you won't give enough training time and the participant is not getting enough time to be able to perform the task properly you will only see a trainings effect in the data.

4) Impact of deviations between simulation and real setup:
In the actual setup the wire must be measured via edgepoints. The simulation calculates the wire as a straight between these edge points. Nevertheless the real wire isn't straight between two edgepoints. It also doesn`t have a perfect edge but more a curved one. Also the center of the loop can vary between simulation and real setup if it's not aligned perfectly. These variations can lead to precision deviations in force feedback. To prevent this, it is recommended to use a wire which is as straight as possible and always be aware that the gripper is picking up the loop in the correct way.
