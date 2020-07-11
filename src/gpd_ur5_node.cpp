/**
* @file gpd_ur5_node.cpp
* @author Boston Cleek
* @date 28 Jun 2020
* @brief Uses grasp pose determination to select a grasp candidate
*        and plans to grasp pose using MoveIt.
*
* @PUBLISHES:
*   grasp_candidate (visualization_msgs/MarkerArray): best grasp candidate visualization
*   grasp_approach (visualization_msgs/Marker): grasp candidate approach vector visualization
*   cloud_stitched (sensor_msgs::PointCloud2): filtered point cloud for GPD
* @SUBSCRIBES:
*   camera/depth/points (sensor_msgs/PointCloud2): raw point cloud
*   detect_grasps/clustered_grasps (gpd_ros/GraspConfigList): list of all grasp candidates from GPD
* @SEERVICES:
*   request_grasp (std_srvs/Empty): requests grasp candidates and selects current point cloud as the input to GPD
*/

// ROS
#include <ros/ros.h>
#include <ros/console.h>
#include <eigen_conversions/eigen_msg.h>
#include <std_srvs/Empty.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <rviz_visual_tools/rviz_visual_tools.h>

// MoveIt
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

// C++
#include <vector>
#include <iostream>
#include <string>

// PCL
#include <pcl/common/common.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/visualization/cloud_viewer.h>

// Eigen
#include <Eigen/Dense>
#include <Eigen/Geometry>

// GPD
#include <gpd_ros/GraspConfig.h>
#include <gpd_ros/GraspConfigList.h>

// Robotiq Gripper msgs
#include <robotiq_85_msgs/GripperCmd.h>
#include <robotiq_85_msgs/GripperStat.h>

/**
* @brief Required data for grasp candidate
*/
struct Gripper
{
  // Translation from frame to gripper
  Eigen::Vector3d position;

  // Orientation of gripper, R = [approach binormal axis]
  Eigen::Quaterniond quat;
  Eigen::Vector3d approach;
  Eigen::Vector3d binormal;
  Eigen::Vector3d axis;

  // Required opening width
  float width;

  // GPD score
  float score;

  // Frame of reference
  std::string frame_id;
};


static geometry_msgs::PoseStamped grasp_pose_robot;             // grasp pose
static Gripper grasp_candidate;                                 // best gpd grasp rep. using eigen

static bool grasp_selected;                                     // grasp received
static bool gripper_ready;                                      // robotiq gripper ready


/**
* @brief Call back for the grasp candidate list
* @param msg - A list containing the top grasp candidates
* @details Selects the top ranked grasp candidate
*/
void graspCallBack(const gpd_ros::GraspConfigList::ConstPtr &msg)
{
  // Use the best grasp
  // Position of grasp
  Eigen::Vector3d position;
  tf::pointMsgToEigen(msg->grasps.at(0).position, position);

  // Orientation of grasp
  Eigen::Vector3d approach;
  Eigen::Vector3d binormal;
  Eigen::Vector3d axis;
  tf::vectorMsgToEigen(msg->grasps.at(0).approach, approach);
  tf::vectorMsgToEigen(msg->grasps.at(0).binormal, binormal);
  tf::vectorMsgToEigen(msg->grasps.at(0).axis, axis);


  Eigen::Matrix3d orientation;
  orientation.col(0) = approach;
  orientation.col(1) = binormal;
  orientation.col(2) = axis;

  Eigen::Affine3d grasp_orientation;
  grasp_orientation.translation() = position;
  grasp_orientation.linear() = orientation;


  tf::poseEigenToMsg(grasp_orientation, grasp_pose_robot.pose);
  grasp_pose_robot.header.frame_id = msg->header.frame_id;

  tf::quaternionMsgToEigen(grasp_pose_robot.pose.orientation, grasp_candidate.quat);

  grasp_candidate.position = position;
  grasp_candidate.approach = approach;
  grasp_candidate.binormal = binormal;
  grasp_candidate.axis = axis;

  grasp_candidate.width = msg->grasps.at(0).width.data;
  grasp_candidate.score = msg->grasps.at(0).score.data;
  grasp_candidate.frame_id = msg->header.frame_id;


  std::cout << "Pose of grasp: " << grasp_pose_robot << std::endl;
  ROS_INFO_NAMED("GPD", "Required gripper width: %f", grasp_candidate.width);

  grasp_selected = true;
}


/**
* @brief Call back for robotiq gripper status
* @param msg - current status of gripper
* @details set gripper status to true if hardware is ready
*/
void robotiqCallBack(const robotiq_85_msgs::GripperStat::ConstPtr &msg)
{
  if (msg->is_ready)
  {
    gripper_ready = true;
  }
}


/**
* @brief Compose the gripper trajectory
* @param posture - Trajectory message
* @param width - Desired opening width (0 is open and 0.8 is closed)
*/
void setGripperState(trajectory_msgs::JointTrajectory& posture, double width)
{
  posture.joint_names.resize(1);
  posture.joint_names[0] = "robotiq_85_left_knuckle_joint";

  posture.points.resize(1);
  posture.points[0].effort.resize(1);
  posture.points[0].effort[0] = 50.0;

  posture.points[0].positions.resize(1);
  posture.points[0].positions[0] = width;
  posture.points[0].time_from_start = ros::Duration(0.5);
}


/**
* @brief Constructs a collision obeject (cracker box) and adds it to the planning scene
* @param planning_scene_interface - The planning scene interface
*/
void addCollisionBox(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface)
{

  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.resize(3);

  //////////////////////////////////////////////////////////////////////////
  // Add the box
  // collision_objects[0].id = "box";
  // collision_objects[0].header.frame_id = "base_link";
  //
  // // Define the primitive and its dimensions
  // collision_objects[0].primitives.resize(1);
  // collision_objects[0].primitives[0].type = collision_objects[1].primitives[0].BOX;
  // collision_objects[0].primitives[0].dimensions.resize(3);
  // collision_objects[0].primitives[0].dimensions[0] = 0.1;
  // collision_objects[0].primitives[0].dimensions[1] = 0.066;
  // collision_objects[0].primitives[0].dimensions[2] = 0.04;
  //
  // // Define the pose of the box
  // collision_objects[0].primitive_poses.resize(1);
  // collision_objects[0].primitive_poses[0].position.x = 0.4;
  // collision_objects[0].primitive_poses[0].position.y = 0.0;
  // collision_objects[0].primitive_poses[0].position.z = 0.05;
  //
  // collision_objects[0].primitive_poses[0].orientation.x = 0.5;
  // collision_objects[0].primitive_poses[0].orientation.y = -0.5;
  // collision_objects[0].primitive_poses[0].orientation.z = 0.5;
  // collision_objects[0].primitive_poses[0].orientation.w = 0.5;
  //
  // collision_objects[0].operation = collision_objects[0].ADD;

  //////////////////////////////////////////////////////////////////////////
  // Add the table
  collision_objects[1].id = "table";
  collision_objects[1].header.frame_id = "base_link";

  // Define the primitive and its dimensions
  collision_objects[1].primitives.resize(1);
  collision_objects[1].primitives[0].type = collision_objects[1].primitives[0].BOX;
  collision_objects[1].primitives[0].dimensions.resize(3);
  collision_objects[1].primitives[0].dimensions[0] = 1.225;
  collision_objects[1].primitives[0].dimensions[1] = 0.77;
  collision_objects[1].primitives[0].dimensions[2] = 0.04;

  // Define the pose of the box
  collision_objects[1].primitive_poses.resize(1);
  collision_objects[1].primitive_poses[0].position.x = 0.0;
  collision_objects[1].primitive_poses[0].position.y = 0.0;
  collision_objects[1].primitive_poses[0].position.z = 0.0;

  collision_objects[1].primitive_poses[0].orientation.x = 0.0;
  collision_objects[1].primitive_poses[0].orientation.y = 0.0;
  collision_objects[1].primitive_poses[0].orientation.z = 0.0;
  collision_objects[1].primitive_poses[0].orientation.w = 1.0;

  collision_objects[1].operation = collision_objects[1].ADD;


  //////////////////////////////////////////////////////////////////////////
  // Add the camera
  collision_objects[2].id = "camera";
  collision_objects[2].header.frame_id = "base_link";

  // Define the primitive and its dimensions
  collision_objects[2].primitives.resize(1);
  collision_objects[2].primitives[0].type = collision_objects[1].primitives[0].BOX;
  collision_objects[2].primitives[0].dimensions.resize(3);
  collision_objects[2].primitives[0].dimensions[0] = 0.05;
  collision_objects[2].primitives[0].dimensions[1] = 0.05;
  collision_objects[2].primitives[0].dimensions[2] = 0.7;

  // Define the pose of the box
  collision_objects[2].primitive_poses.resize(1);
  collision_objects[2].primitive_poses[0].position.x = -0.085;
  collision_objects[2].primitive_poses[0].position.y = -0.125;
  collision_objects[2].primitive_poses[0].position.z = 0.35;

  collision_objects[2].primitive_poses[0].orientation.x = 0.0;
  collision_objects[2].primitive_poses[0].orientation.y = 0.0;
  collision_objects[2].primitive_poses[0].orientation.z = 0.0;
  collision_objects[2].primitive_poses[0].orientation.w = 1.0;

  collision_objects[2].operation = collision_objects[2].ADD;

  // Apply objects to planning scene
  planning_scene_interface.applyCollisionObjects(collision_objects);
}


/**
* @brief Compose the pre-grasp pose the end effector
* @param distance - Tranlation distance away from the grasp pose determined by GPD
* @return pre-grasp pose
*/
geometry_msgs::Pose preGrasp(double distance)
{
  geometry_msgs::Pose pre_grasp = grasp_pose_robot.pose;

  // L2-norm of approach vector
  const auto approach_norm = grasp_candidate.approach.norm();

  // Flip direction of approach vector and noralmize it
  // Multiply by the distance to get translation in the specified direction
  pre_grasp.position.x += (grasp_candidate.approach(0) / approach_norm) * (-1.0 * distance);
  pre_grasp.position.y += (grasp_candidate.approach(1) / approach_norm) * (-1.0 * distance);
  pre_grasp.position.z += (grasp_candidate.approach(2) / approach_norm) * (-1.0 * distance);

  return pre_grasp;
}


/**
* @brief Compose the grasping pose of the end effector
* @param pre_grasp - The pose of the pre-grasp
* @param distance - Distance away from the grasp goal
* @return grasp pose of the end effector link
*/
geometry_msgs::Pose graspPose(const geometry_msgs::Pose &pre_grasp, double distance)
{
  geometry_msgs::Pose grasp_goal = pre_grasp;

  // L2-norm of approach vector
  const auto approach_norm = grasp_candidate.approach.norm();

  // Multiply by the distance to get translation in the specified direction after normalization
  grasp_goal.position.x += (grasp_candidate.approach(0) / approach_norm) * distance;
  grasp_goal.position.y += (grasp_candidate.approach(1) / approach_norm) * distance;
  grasp_goal.position.z += (grasp_candidate.approach(2) / approach_norm) * distance;

  return grasp_goal;
}


/**
* @brief Compose the post-grasp pose
* @param grasp - The grasp pose
* @param direction - A unit vector decribing the direction to move after grasping
* @param distance - Distance to move in the direction vector
* @return post-grasp pose
*/
geometry_msgs::Pose postGrasp(const geometry_msgs::Pose &grasp, const geometry_msgs::Vector3 &direction, double distance)
{
  geometry_msgs::Pose post_grasp = grasp;

  // Multiply by the distance to get translation in the specified direction after normalization
  post_grasp.position.x += direction.x * distance;
  post_grasp.position.y += direction.y * distance;
  post_grasp.position.z += direction.z * distance;

  return post_grasp;
}




int main(int argc, char** argv)
{
  ros::init(argc, argv, "gpd_ur5");
  ros::NodeHandle nh("~");
  ros::NodeHandle node_handle;


  ros::Subscriber grasp_sub = node_handle.subscribe("detect_grasps/clustered_grasps", 1, graspCallBack);
  ros::Subscriber robotiq_stat_sub = node_handle.subscribe("/gripper/stat", 1, robotiqCallBack);

  ros::Publisher gripper_cmd_pub = node_handle.advertise<trajectory_msgs::JointTrajectory>("/gripper/command", 1);    // gazebo
  ros::Publisher robotiq_cmd_pub = node_handle.advertise<robotiq_85_msgs::GripperCmd>("/gripper/cmd", 1);             // hardware


  ROS_INFO("Successfully launch gpd_ur5 node");


  // Wait for gripper status
  ROS_INFO("Waiting for Robotiq gripper...");
  while (node_handle.ok() && !gripper_ready) { ros::spinOnce(); }
  ROS_INFO("Robotiq gripper is ready");


  // No msgs or services have been called
  grasp_selected = false;
  gripper_ready = false;

  // Required
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // Move group interface
  static const std::string PLANNING_GROUP = "manipulator";
  static const std::string POSE_REF_FRAME = "base_link";
  static const std::string END_EFFECTOR_LINK = "ee_link";

  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
  move_group.setPlanningTime(60.0);
  // move_group.setNumPlanningAttempts(2);

  move_group.setEndEffectorLink(END_EFFECTOR_LINK);
  move_group.setPoseReferenceFrame(POSE_REF_FRAME);


  // Setup visualization
  // rviz_visual_tools::RvizVisualToolsPtr visual_tools(new rviz_visual_tools::RvizVisualTools("base_link","/rviz_visual_markers"));
  // visual_tools->trigger();


  // Reference frame for this robot.
  ROS_INFO_NAMED("GPD", "Planning frame: %s", move_group.getPlanningFrame().c_str());

  // End-effector link for this group.
  ROS_INFO_NAMED("GPD", "End effector link: %s", move_group.getEndEffectorLink().c_str());

  // List of all the groups in the robot:
  ROS_INFO_NAMED("GPD", "Available Planning Groups:");
  std::copy(move_group.getJointModelGroupNames().begin(), move_group.getJointModelGroupNames().end(),
            std::ostream_iterator<std::string>(std::cout, ", "));


  ROS_INFO_NAMED("GPD", "Pose reference frame: %s", move_group.getPoseReferenceFrame().c_str());
  ROS_INFO_NAMED("GPD", "Planning reference frame: %s", move_group.getPlanningFrame().c_str());


  // Setup planning scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  addCollisionBox(planning_scene_interface);


  /////////////////////////////////////////////////////
  // visual_tools->prompt("Press 'next' in the RvizVisualToolsGui window to start configuration");

  // starting joint angles std::vector<double> joint_group_positions;
  std::vector<double> joint_group_positions(6);
  joint_group_positions.at(0) = 0.9547982215881348;
  joint_group_positions.at(1) = -1.5013459364520472;
  joint_group_positions.at(2) = 0.5664740800857544;
  joint_group_positions.at(3) = -1.3329065481769007;
  joint_group_positions.at(4) = -1.8420470396624964;
  joint_group_positions.at(5) = 0.44860324263572693;

  move_group.setJointValueTarget(joint_group_positions);
  move_group.move();


  /////////////////////////////////////////////////////
  // wait for gripper to be ready

  ROS_INFO("Waiting for grasp candidate... ");
  while (node_handle.ok() && !grasp_selected) { ros::spinOnce(); }
  ROS_INFO("Grasp candidate received");


  /////////////////////////////////////////////////////
  // Hard code goal for testing (Previous result from GPD)
  // grasp_pose_robot.header.frame_id = "base_link";
  // grasp_pose_robot.pose.position.x = 0.396656;
  // grasp_pose_robot.pose.position.y = 0.00208467;
  // grasp_pose_robot.pose.position.z = 0.110008;
  //
  // grasp_pose_robot.pose.orientation.x = -0.0626088;
  // grasp_pose_robot.pose.orientation.y = 0.665858;
  // grasp_pose_robot.pose.orientation.z = -0.0728375;
  // grasp_pose_robot.pose.orientation.w = 0.73987;
  //
  // tf::quaternionMsgToEigen(grasp_pose_robot.pose.orientation, grasp_candidate.quat);
  // grasp_candidate.approach = grasp_candidate.quat.toRotationMatrix().col(0);
  /////////////////////////////////////////////////////



  /////////////////////////////////////////////////////
  // Pick
  // visual_tools->prompt("Press 'next' in the RvizVisualToolsGui window to plan to pick");

  robotiq_85_msgs::GripperCmd gripper_cmd;              // gripper hardware msg

  // Open gripper
  gripper_cmd.position = 0.8;
  robotiq_cmd_pub.publish(gripper_cmd);


  // Pre-grasp displacement, pre grasp distance 25cm from tool_changer_tool0
  geometry_msgs::Pose pre_grasp = preGrasp(0.25);
  move_group.setPoseTarget(pre_grasp);
  move_group.move();

  // move 10cm from pre-grasp goal
  geometry_msgs::Pose grasp_goal = graspPose(pre_grasp, 0.1);
  move_group.setPoseTarget(grasp_goal);
  move_group.move();


  // Close gripper
  gripper_cmd.position = 0.0;
  robotiq_cmd_pub.publish(gripper_cmd);

  // wait for gripper to close
  ros::Duration(1.0).sleep();

  // // Attach box to gripper
  // move_group.attachObject("box");

  geometry_msgs::Vector3 lift_direction;
  lift_direction.x = 0.0;
  lift_direction.y = 0.0;
  lift_direction.z = 1.0;


  // Post-grasp displacement
  geometry_msgs::Pose post_grasp = postGrasp(grasp_goal, lift_direction, 0.15);

  move_group.setPoseTarget(post_grasp);
  move_group.move();
  /////////////////////////////////////////////////////


  /////////////////////////////////////////////////////
  // Place
  geometry_msgs::Pose drop_goal = grasp_goal;
  drop_goal.position.y = drop_goal.position.y + 0.20;

  move_group.setPoseTarget(drop_goal);
  move_group.move();


  // Open gripper
  gripper_cmd.position = 0.8;
  robotiq_cmd_pub.publish(gripper_cmd);

  // wait for gripper to open
  ros::Duration(1.0).sleep();

  // // Detach box to gripper
  // move_group.detachObject("cracker_box");


  // place retreat
  geometry_msgs::Pose post_drop = postGrasp(drop_goal, lift_direction, 0.15);

  move_group.setPoseTarget(post_drop);
  move_group.move();

  /////////////////////////////////////////////////////


  ros::shutdown();
  return 0;
}


// end file
