
# Overview
Provides an interface for simulating grasp pose detection (GPD) using a UR5 robot
with a Robotiq 85 gripper.

GPD composes a 6-DOF grasp poses for 2-finger robot grippers. The input is a 3D point cloud and produces pose estimates of possible grasps [1].

GPD consists of two main steps: </br>
1) Sampling a large number of grasp candidates </br>
2) Classifying the candidates as viable grasps

# Getting started
1) Download and install the [GPD library](https://github.com/atenpas/gpd). I am running Ubuntu 18.04 and ROS Melodic and found that in order to successfully use the GPD library the `-03` compiler optimization needs to be removed from the GPD `CMakeLists.txt`.


2) Download the rosinstall file
* note: There are two MoveIt! config packages that are downloaded if you use both rosinstall files. The first is from the [Kavraki Lab](http://www.kavrakilab.org/) and the second is from [PickNik](https://picknik.ai/). Both can be used for simulation but the config from PickNik is currently used for testing on PickNik's UR5.

`mkdir -p gpd_ws/src`

`wstool merge https://raw.githubusercontent.com/bostoncleek/gpd_simulator/master/gpd.rosinstall`

`wstool merge https://raw.githubusercontent.com/PickNikRobotics/picknik_ur5_moveit_config/master/picknik_ur5_moveit_config.rosinstall`

`wstool update`



3) Check dependencies </br>
`rosdep update` </br>
`rosdep install --rosdistro $ROS_DISTRO --ignore-src --from-paths src`

4) Build </br>
`catkin build`

# How to run

## PickNik MoveIt Config
1) `roslaunch ur_gazebo ur5_demo.launch fake_execution:=false`
* note: Set fake_execution:=true to run in simulation.
* note: You may need to set usb permissions for the Robotiq Gripper

`sudo chmod 666 /dev/ttyUSB0`

2) Request a grasp candidate using the current point cloud published by the depth camera

`rosservice call /request_grasp`



## Kavraki Lab MoveIt Config
1) Launch the simulation </br>
`roslaunch ur_gazebo ur5_robotiq85.launch` </br>

`roslaunch ur5_robotiq85_moveit_config ur5_robotiq85_moveit_planning_execution.launch sim:=true` </br>

`roslaunch ur5_robotiq85_moveit_config moveit_rviz.launch config:=true`

2) Launch GPD </br>
`roslaunch gpd_simulation gpd_sim.launch`

3) To request a grasp candidate (this selected the current point cloud) </br>

`rosservice call /request_grasp`

4) Press `q` on the point cloud viewer window to continue the demo


# Results

On this run 26 grasp candidates (shown in blue) where generated.

<p align="center">
  <img src="media/gpdcandidates.jpg" width="450" height="450"/>
</p>


The green marker is the highest ranked grasp candidate using point cloud data.

<p align="center">
  <img src="media/gpdur5rviz.gif" width="450" height="450"/>
</p>

The Gazebo physics engine is not robust enough to accurately simulate grasping. The robot is unable to pick up the object when the friction between gripper and box is maximum.

<p align="center">
  <img src="media/gpdur5gazebo.gif" width="450" height="450"/>
</p>



# Issues
1) In order to successfully use the GPD library the `-03` compiler optimization needs to be removed from the GPD `CMakeLists.txt`.

2) The gripper is not usually able to successfully lift the cracker box in Gazebo as a result of the difficulty in contact modeling. A possible future solution will use a world plugin to create the joint at run time.

# References
[1] Andreas ten Pas, Marcus Gualtieri, Kate Saenko, and Robert Platt. [Grasp Pose Detection in Point Clouds.](https://arxiv.org/abs/1706.09911) The International Journal of Robotics Research, Vol 36, Issue 13-14, pp. 1455-1473. October 2017.
