

# How to run
* note: make sure the `ur5_ws` is in the `ROS_PACKAGE_PATH` of the `gpd_ws`


    roslaunch ur_gazebo ur5_robotiq85.launch

    roslaunch ur5_robotiq85_moveit_config ur5_robotiq85_moveit_planning_execution.launch sim:=true
    
    roslaunch ur5_robotiq85_moveit_config moveit_rviz.launch config:=true
