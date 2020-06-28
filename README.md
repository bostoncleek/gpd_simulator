
# Overview
Provides an interface for simulating grasp pose determination using a UR5 robot
with a Robotiq 85 gripper.

# Getting started
1) Download and install the [GPD library](https://github.com/atenpas/gpd). I am running Ubuntu 18.04 and ROS Melodic and found that in order to successfully use the GPD library the `-03` compiler optimization needs to be removed from the GPD `CMakeLists.txt`.


2) Download the rosinstall file

3) Check dependencies </br>
`rosdep update` </br>
`rosdep install --rosdistro $ROS_DISTRO --ignore-src --from-paths src`

4) Build </br>
`catkin_make`

# How to run

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

<p align="center">
  <img src="media/gpdur5gazebo.gif" width="450" height="450"/>
</p>



# Issues
1) In order to successfully use the GPD library the `-03` compiler optimization needs to be removed from the GPD `CMakeLists.txt`.

2) The gripper is not usually able to successfully lift the cracker box in Gazebo as a result of the difficulty in contact modeling. A possible future solution will use a world plugin to create the joint at run time.
