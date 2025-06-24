# Hybrid Control Framework for Whole-Body Robot Teleoperation

## Overview
Physical testbed: TIAGo OMNI ++

Control input: HTC Vive Pro 2 System

Communication: UDP

Windows 11/Unity 2019: publish the data that received from HTC Vive headset and controllers

Ubuntu 20.04/ROS Noetic: subscribe the data from Unity via UDP server to control the TIAGo

## Installation
Clone this repository into your catkin workspace, then use `catkin build` to build:
```bash
cd ~/catkin_ws/src
git clone https://github.com/yourusername/vive_teleop.git
cd ..
catkin build
```

## Preparation
1. Connect the TIAGo via Ethernet to the Ubuntu PC
2. Plugin the Vive system (headset and two controllers) to the Windows PC

## Launch the Hybrid Interface
The baseline control via hybrid interface allows the HTC Vive VR system to control the robot’s head, arms, torso, and base. Refer to the teleoperation interface mapping for more details.
### Windows Side
1. Launch the Unity Hub (tested with version 3.8.0) and open the Vive Control project
2. Play the project by clicking the play button at the top center of the software
### Ubuntu Side
In the first terminal, build the connection between ROS and Unity
```bash
ssh pal@IP
python3 ros_ws/src/socket_connection.py
```
In the second terminal, start the teleoperation
```bash
Master
IP
cd ~/catkin_ws/
source devel/setup.bash
roslaunch vive_teleop vive_teleop.launch sim:=false rviz:=true record:=false
```
In the third terminal, initiative the GUI
```bash
cd ui_ws/
python3 ui_baseline.py
```

## Launch File Arguments
The launch file accepts several arguments that allow you to customize its behavior:

- `sim` (default: `true`): If set to `true`, the node operates in simulation mode. If set to `false`, the node operates with a real robot.

- `record` (default: `false`): If set to `true`, the node records head, arms, torso, base configuration during operation. This can be useful for debugging or analysis.

- `rvi` (default: `true`): If set to `true`, the node launches RViz for visualizing the robot and its sensor data.

## Graphical User Interface
<p align="left">
  <img src="https://github.com/intuitivecomputing/hybrid-teleoperation/blob/main/figures/GUI.jpg" alt="image" alt="image" width="50%"/>
</p>

The GUI integrates a video stream, arm control states, and both back and top views of the robot’s mini model, displaying the robot’s arm pose, torso height, and surrounding obstacles. Red arrows indicate control inputs, with red lines for the torso’s vertical limits. The base is colored green or gray to signify “fast” or “slow” mode depending on whether the center of the base is within or outside the purple area.

## Teleoperation Interface Mapping

### HTC Vive
<p align="left">
  <img src="https://github.com/intuitivecomputing/hybrid-teleoperation/blob/main/figures/ViveMapping.jpg" alt="image" alt="image" width="50%"/>
</p>

1. **Head Control**: After launching the interface, the motion of the HTC Vive headset will instantly map to control the robot's head (pan and tilt).
2. **Arm Control**: Press the grip button on the handheld controller to activate arm control, which will map the controller’s movement to move the robot's hand. You can pause the control by pressing the same grip button again. Use the menu button to return the arm to its starting configuration.
3. **Torso Control**: Press the up and down sections of the trackpad on the right controller to move the torso up and down.
4. **Base Control**: [Translation] Press the trackpad on the left controller to move the base forward, backward, or sideways. [Rotation] Press the left and right sections of the trackpad on the right controller to rotate the base.
5. **Gripper Control**: Press the trigger on the controller to open or close the robot's fingers.

## Troubleshooting
1. **GUI Lagging**: Check the Wi-Fi network is connected to different hosts between the two operating systems.
2. **Gripper Issues**: Avoid prolonged operation (e.g., holding an object) due to hardware limitations that may cause motor overheating. Ensure the two fingers remain parallel to the palm to minimize friction.
