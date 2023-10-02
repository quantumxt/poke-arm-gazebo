# poke-arm-gazebo

> WIP Port to ROS2 Humble

[![Docker build](https://img.shields.io/github/actions/workflow/status/quantumxt/poke-arm-gazebo/docker-image.yml?style=for-the-badge)](https://github.com/quantumxt/poke-arm-gazebo/actions)

An experimental gazebo robotic arm model with 4 movable joints and a depth camera.

![poke_arm_main](img/pa_main.png)

The simulation would be containerized with docker (Ubuntu 22.04, ROS Humble & Gazebo Garden).

## Prerequistes

> Ensure that a Nvidia GPU driver is installed before proceeding.

Ensure that docker & Nvidia Container toolkit has been installed beforehand.

### Docker

More information on docker installation could be found here: [https://docs.docker.com/engine/install/ubuntu/](https://docs.docker.com/engine/install/ubuntu/).

### Nvidia Container toolkit

Configure the repository.

```bash
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg \
  && curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
    sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
    sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list \
  && \
    sudo apt-get update
```

After that, install the NVIDIA Container Toolkit packages:
```bash
sudo apt-get install -y nvidia-container-toolkit
```

## Installation

Clone the repository onto your desired location.
```bash
git clone https://github.com/quantumxt/poke-arm-gazebo.git -b ros2
```

Enter the directory & build the docker image via the `build.sh` script.
```bash
cd poke-arm-gazebo
./build.sh
```

After the docker image has been build, run `run_docker.sh` to start the container:
```bash
./run_docker.sh
```

> The *poke arm* model depends on the following packages, which would be preinstalled in the docker image:
> - ROS2 Controllers 
> - MoveIt!

## Overview
### Joints
The poke arm is designed to push an object, therefore the *revolute* joints were configured to move in the x-axis direction.
- 1 *fixed* joint
- 3 *revolute* joints
- 1 *continuous* joint

![poke_arm_joints](img/pa_joints.png)

### Camera
A RGBD (Depth) Camera is mounted at the end of the robot arm. :camera:
```
Camera Resolution: 320 X 240
Image Range: 0.01 - 18.0m
Field of View (FOV, Horizontal): 60°
```

## File structure
The folder are organised into 3 parts:
- poke_description: Contains the URDF description of the robot.
- poke_gazebo: Contains *launch* files to start gazebo & spawn the model in it.
- poke_control: Contains the controllers used to control the poke arm.

## Testing
### Via *rostopic*
To test the poke arm, roslaunch *poke_control.launch*. This will launch both the arm model and the controller in Gazebo.
```bash
$ roslaunch poke_control poke_control.launch
```
![poke_arm_pose](img/pa_pose.png)

To retract the arm:
```bash
$ rostopic pub /arm_controller/command trajectory_msgs/JointTrajectory '{joint_names: ["p0_joint","p1_joint","p2_joint","p3_joint"], points: [{positions: [-1.2,2.5,0.6,0.0],time_from_start:[1.0,0.0]}]}' -1
```

To extend the arm:
```bash
$ rostopic pub /arm_controller/command trajectory_msgs/JointTrajectory '{joint_names: ["p0_joint","p1_joint","p2_joint","p3_joint"], points: [{positions: [1.2,0.2,0.6,0.0],time_from_start:[1.0,0.0]}]}' -1
```

### Via *Rviz*
To test the arm via Rviz, roslaunch *poke_moveit_control.launch*. This utilises the moveit_group to plan & execute the motion of the arm determined by the interactive marker of Rviz.
```
$ roslaunch poke_control poke_moveit_control.launch
```
![poke_arm_rviz](img/pa_rviz.png)

The rqt_graph:

![poke_arm_rviz](img/pa_rviz_moveit_rqt.png)


## Mounting *Poke arm* on Turtlebot
> It is better to download the relevant Turtlebot packages to your workspace than to install it via *apt-get*, as you have to modify the URDF/Xacro file.

> For this section, it is assumed that you are using the default turtlebot base configuration. (Kobuki, hexagon, asus_xtion_pro)

To mount the *Poke arm* on Turtlebot, you have to modify some files in the *turtlebot_description* directory/ROS Package. Open up the following xacro file. 
> turtlebot/turtlebot_description/robots/kobuki_hexagons_asus_xtion_pro.urdf.xacro

To add the arm, we'll add the *<poke_arm>* into the file. Therefore, the file should look like this:
```xml
<?xml version="1.0"?>
<!--
    - Base      : kobuki
    - Stacks    : hexagons
    - 3d Sensor : kinect
-->
<robot name="turtlebot" xmlns:xacro="http://ros.org/wiki/xacro">

  <xacro:include filename="$(find turtlebot_description)/urdf/turtlebot_common_library.urdf.xacro" />
  <xacro:include filename="$(find kobuki_description)/urdf/kobuki.urdf.xacro" />
  <xacro:include filename="$(find turtlebot_description)/urdf/stacks/hexagons.urdf.xacro"/>
  <xacro:include filename="$(find turtlebot_description)/urdf/sensors/asus_xtion_pro.urdf.xacro"/>

  <kobuki/>
  <stack_hexagons                 parent="base_link"/>
  <sensor_asus_xtion_pro          parent="base_link"/>

  <!-- Added the arm here -->
  <xacro:include filename="$(find poke_description)/urdf/poke.xacro" />
  <poke_arm parent="plate_top_link" color="white" joints_vlimit="1.571">
    <!-- Place the "floating" arm at the location it should be if mounted on a turtlebot -->
    <origin xyz="-0.05 0 0"/>
  </poke_arm>

</robot>
```

And viola! The arm is mounted!


## References

- https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html
