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


// MoveitCpp
#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/moveit_cpp/planning_component.h>

// MoveIt
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>


// C++
#include <vector>
#include <iostream>
#include <string>
#include <memory>

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
// #include <gpd_ros/CloudSources.h>
// #include <gpd_ros/CloudSamples.h>
#include <gpd_ros/GraspConfig.h>
#include <gpd_ros/GraspConfigList.h>


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



static sensor_msgs::PointCloud2 cloud_msg;                      // point cloud to search for graps
static geometry_msgs::TransformStamped t_stamped_base_opt;      // Transform from robot base_link to camera_optical_link
static geometry_msgs::PoseStamped grasp_pose_robot;             // grasp pose goal
static Gripper grasp_candidate;                                 // best grasp candidate


static bool cloud_srv_active;                                   // request grasps status
static bool cloud_msg_received;                                 // point cloud message received status
static bool grasp_selected;                                     // publish selected grasp


// TODO read params in through server
static const auto outer_diameter = 0.12;
static const auto hand_depth = 0.06;
static const auto finger_width = 0.01;
static const auto hand_height = 0.02;


/**
* @brief Segments objects from table plane
* @param [out] cloud - Segemented cloud
*/
void removeTable(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
  // SAC segmentor without normals
  pcl::SACSegmentation<pcl::PointXYZRGB> segmentor;
  segmentor.setOptimizeCoefficients(true);
  segmentor.setModelType(pcl::SACMODEL_PLANE);
  segmentor.setMethodType(pcl::SAC_RANSAC);

  // Max iterations and model tolerance
  segmentor.setMaxIterations(1000);
  segmentor.setDistanceThreshold (0.01);

  // Input cloud
  segmentor.setInputCloud(cloud);

  // Inliers representing points in the plane
  pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices);

  // Use a plane as the model for the segmentor
  pcl::ModelCoefficients::Ptr coefficients_plane(new pcl::ModelCoefficients);
  segmentor.segment(*inliers_plane, *coefficients_plane);

  if (inliers_plane->indices.size () == 0)
  {
    ROS_ERROR("Could not estimate a planar model for the given dataset");
  }

  // Extract the inliers from the cloud
  pcl::ExtractIndices<pcl::PointXYZRGB> extract_indices;
  extract_indices.setInputCloud(cloud);
  extract_indices.setIndices(inliers_plane);

  // Remove plane inliers and extract the rest
  extract_indices.setNegative(true);
  extract_indices.filter(*cloud);


  if (cloud->points.empty())
  {
    ROS_ERROR("Can't find objects");
    // ros::shutdown();
  }

  // pcl::visualization::CloudViewer viewer ("Cloud");
  // viewer.showCloud (cloud);
  // while (!viewer.wasStopped ()) {}
}


/**
* @brief Call back for requesting grasp empty service
* @return true when called
* @details this is an empty service
*/
bool requestGraspCloudCallBack(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
{
  ROS_INFO("Request grasp service activated");
  cloud_srv_active = true;
  return true;
}


/**
* @brief Point cloud call back
* @param msg - point cloud message
* @details Segments objects from table plane and sends point cloud to GPD
*/
void cloudCallBack(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
  if (cloud_srv_active)
  {
    ROS_INFO("Point cloud selected");

    // use RANSAC to filter points above table
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    // convert from ROS msg to a point cloud
    pcl::fromROSMsg(*msg.get(), *cloud);
    ROS_INFO("Point cloud size: %lu ", cloud->points.size());

    // segment objects from table
    removeTable(cloud);
    ROS_INFO("Segmented point cloud size: %lu ", cloud->points.size());

    if (cloud->points.empty())
    {
      ROS_ERROR("Cloud empty");
      // ros::shutdown();
    }

    // covert back to ROS msg
    pcl::toROSMsg(*cloud, cloud_msg);

    cloud_srv_active = false;
    cloud_msg_received = true;
  }
}


/**
* @brief Call back for the grasp candidate list
* @param msg - A list containing the top grasp candidates
* @details Selects the top ranked grasp candidate
*/
void graspCallBack(const gpd_ros::GraspConfigList::ConstPtr &msg)
{
  ROS_INFO("Number of graps received %lu", msg->grasps.size());

  // Use the best grasp
  // grasp_msg = msg->grasps.at(0);

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

  // Transform from camera optical link to grasp
  Eigen::Affine3d t_optical_grasp;
  t_optical_grasp.translation() = position;
  t_optical_grasp.linear() = orientation;

  // std::cout << "Translation: " <<  t_optical_grasp.translation() << std::endl;
  // std::cout << "Rotation: " << t_optical_grasp.rotation() << std::endl;

  // Convert Eigen transform into pose msg in frame of optical link
  geometry_msgs::Pose pose_frame_optical;
  tf::poseEigenToMsg(t_optical_grasp, pose_frame_optical);

  geometry_msgs::PoseStamped grasp_pose_optical;
  grasp_pose_optical.pose = pose_frame_optical;

  // Transform grasp pose in frame of optical link into frame of the base_link of the robot
  // geometry_msgs::PoseStamped grasp_pose_robot;
  tf2::doTransform(grasp_pose_optical, grasp_pose_robot, t_stamped_base_opt);

  // Convert to representation used to visualize in frame of robot's base_link
  tf::quaternionMsgToEigen(grasp_pose_robot.pose.orientation, grasp_candidate.quat);
  Eigen::Matrix3d grasp_rot = grasp_candidate.quat.toRotationMatrix();

  tf::pointMsgToEigen(grasp_pose_robot.pose.position, grasp_candidate.position);
  grasp_candidate.approach = grasp_rot.col(0);
  grasp_candidate.binormal = grasp_rot.col(1);
  grasp_candidate.axis = grasp_rot.col(2);

  grasp_candidate.width = msg->grasps.at(0).width.data;
  grasp_candidate.score = msg->grasps.at(0).score.data;
  grasp_candidate.frame_id = grasp_pose_robot.header.frame_id;

  std::cout << "Pose of grasp frame: " << grasp_pose_robot << std::endl;
  ROS_INFO_NAMED("GPD", "Required gripper width: %f", grasp_candidate.width);

  grasp_selected = true;
}


/**
* @brief Constructs a marker representing the base of the gripper
* @param center - center of cube
* @param quat - orientation of cube
* @param lwh - dimensions of cube
* @param id - marker ID
* @param frame_id - frame of reference
* @return visualization marker for base
*/
visualization_msgs::Marker fingerMarker(const Eigen::Vector3d& center,
  const Eigen::Quaterniond quat, const Eigen::Vector3d& lwh, int id, const std::string& frame_id)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = frame_id;
  marker.header.stamp = ros::Time();
  marker.ns = "finger";
  marker.id = id;
  marker.type = visualization_msgs::Marker::CUBE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = center(0);
  marker.pose.position.y = center(1);
  marker.pose.position.z = center(2);
  marker.lifetime = ros::Duration(100);

  // use orientation of hand
  marker.pose.orientation.x = quat.x();
  marker.pose.orientation.y = quat.y();
  marker.pose.orientation.z = quat.z();
  marker.pose.orientation.w = quat.w();

  // these scales are relative to the hand frame (unit: meters)
  marker.scale.x = lwh(0); // forward direction
  marker.scale.y = lwh(1); // hand closing direction
  marker.scale.z = lwh(2); // hand vertical direction

  marker.color.a = 1.0;
  marker.color.r = 0.0;
  marker.color.g = 0.5;
  marker.color.b = 0.0;

  return marker;
}


/**
* @brief Constructs a marker representing the base of the gripper
* @param start - left side of base
* @param end - right side of base
* @param quat - orientation of base
* @param length - length of base
* @param height - height of base
* @param id - marker ID
* @param frame_id - frame of reference
* @return visualization marker for base
*/
visualization_msgs::Marker handBaseMarker(const Eigen::Vector3d& start,
  const Eigen::Vector3d& end, const Eigen::Quaterniond quat, double length, double height, int id,
  const std::string& frame_id)
{
  Eigen::Vector3d center = start + 0.5 * (end - start);

  visualization_msgs::Marker marker;
  marker.header.frame_id = frame_id;
  marker.header.stamp = ros::Time();
  marker.ns = "hand_base";
  marker.id = id;
  marker.type = visualization_msgs::Marker::CUBE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = center(0);
  marker.pose.position.y = center(1);
  marker.pose.position.z = center(2);
  marker.lifetime = ros::Duration(100);

  // use orientation of hand
  marker.pose.orientation.x = quat.x();
  marker.pose.orientation.y = quat.y();
  marker.pose.orientation.z = quat.z();
  marker.pose.orientation.w = quat.w();

  // these scales are relative to the hand rotation (unit: meters)
  marker.scale.x = length; // forward direction
  marker.scale.y = (end - start).norm(); // hand closing direction
  marker.scale.z = height; // hand vertical direction

  marker.color.a = 1.0;
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;

  return marker;
}


/**
* @brief Constructs a MarkerArray message that displays the gripper
* @param [out] marker_array - The gripper from GPD
*/
void visualizeGraspCandidate(visualization_msgs::MarkerArray &marker_array)
{
  // use the first id which is the best hand
  const auto i = 0;

  const auto hw = 0.5*outer_diameter - 0.5*finger_width;

  visualization_msgs::Marker left_finger, right_finger, base, approach;
  Eigen::Vector3d left_bottom, right_bottom, left_top, right_top, left_center, right_center, approach_center, base_center;

  left_bottom = grasp_candidate.position - hw * grasp_candidate.binormal;
  right_bottom = grasp_candidate.position + hw * grasp_candidate.binormal;
  left_top = left_bottom + hand_depth * grasp_candidate.approach;
  right_top = right_bottom + hand_depth * grasp_candidate.approach;
  left_center = left_bottom + 0.5*(left_top - left_bottom);
  right_center = right_bottom + 0.5*(right_top - right_bottom);
  base_center = left_bottom + 0.5*(right_bottom - left_bottom) - 0.01*grasp_candidate.approach;
  approach_center = base_center - 0.04*grasp_candidate.approach;

  Eigen::Vector3d finger_lwh, approach_lwh;
  finger_lwh << hand_depth, finger_width, hand_height;
  approach_lwh << 0.08, finger_width, hand_height;


  base = handBaseMarker(left_bottom, right_bottom, grasp_candidate.quat, 0.02, hand_height, i, grasp_candidate.frame_id);
  left_finger = fingerMarker(left_center, grasp_candidate.quat, finger_lwh, i*3, grasp_candidate.frame_id);
  right_finger = fingerMarker(right_center, grasp_candidate.quat, finger_lwh, i*3+1, grasp_candidate.frame_id);
  approach = fingerMarker(approach_center, grasp_candidate.quat, approach_lwh, i*3+2, grasp_candidate.frame_id);

  marker_array.markers.push_back(left_finger);
  marker_array.markers.push_back(right_finger);
  marker_array.markers.push_back(approach);
  marker_array.markers.push_back(base);
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
  collision_objects.resize(1);

  // Add the cracker box
  collision_objects[0].id = "cracker_box";
  collision_objects[0].header.frame_id = "world";

  // Define the primitive and its dimensions
  collision_objects[0].primitives.resize(1);
  collision_objects[0].primitives[0].type = collision_objects[1].primitives[0].BOX;
  collision_objects[0].primitives[0].dimensions.resize(3);
  collision_objects[0].primitives[0].dimensions[0] = 0.16;
  collision_objects[0].primitives[0].dimensions[1] = 0.06;
  collision_objects[0].primitives[0].dimensions[2] = 0.21;

  // Define the pose of the table
  collision_objects[0].primitive_poses.resize(1);
  collision_objects[0].primitive_poses[0].position.x = 0;
  collision_objects[0].primitive_poses[0].position.y = -0.5;
  collision_objects[0].primitive_poses[0].position.z = 1.0194;
  collision_objects[0].primitive_poses[0].orientation.w = 0.5;
  collision_objects[0].primitive_poses[0].orientation.z = 0.866;

  collision_objects[0].operation = collision_objects[0].ADD;

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
  ros::NodeHandle node_handle;
  ros::NodeHandle nh("~");

  ros::Subscriber cloud_sub = node_handle.subscribe("camera/depth/points", 1, cloudCallBack);
  // ros::Subscriber cloud_sub = node_handle.subscribe("depth_camera/depth/points", 1, cloudCallBack);
  ros::Subscriber grasp_sub = node_handle.subscribe("detect_grasps/clustered_grasps", 1, graspCallBack);

  ros::Publisher grasp_viz_pub = node_handle.advertise<visualization_msgs::MarkerArray>("grasp_candidate", 1);
  ros::Publisher grasp_approach_pub = node_handle.advertise<visualization_msgs::Marker>("grasp_approach", 1);
  ros::Publisher cloud_pub = node_handle.advertise<sensor_msgs::PointCloud2>("cloud_stitched", 1);
  ros::Publisher gripper_cmd_pub = node_handle.advertise<trajectory_msgs::JointTrajectory>("/gripper/command", 1);

  ros::ServiceServer cloud_serv = node_handle.advertiseService("request_grasp", requestGraspCloudCallBack);

  ROS_INFO("Successfully launch gpd_ur5 node");

  cloud_srv_active = false;
  cloud_msg_received = false;
  grasp_selected = false;

  // Required
  ros::AsyncSpinner spinner(4);
  spinner.start();

  // Move group interface
  static const std::string PLANNING_GROUP = "manipulator";
  static const std::string POSE_REF_FRAME = "base_link";
  static const std::string END_EFFECTOR_LINK = "ee_link";


  /* Otherwise robot with zeros joint_states */
  ros::Duration(1.0).sleep();


  auto moveit_cpp_ptr = std::make_shared<moveit::planning_interface::MoveItCpp>(node_handle);

  auto planning_components =
      std::make_shared<moveit::planning_interface::PlanningComponent>(PLANNING_GROUP, moveit_cpp_ptr);

  auto robot_model_ptr = moveit_cpp_ptr->getRobotModel();
  auto robot_start_state = planning_components->getStartState();
  auto joint_model_group_ptr = robot_model_ptr->getJointModelGroup(PLANNING_GROUP);


  // Setup visualization
  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("shoulder_link");
  visual_tools.deleteAllMarkers();
  visual_tools.loadRemoteControl();

  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.75;
  visual_tools.publishText(text_pose, "GPD Demo", rvt::WHITE, rvt::XLARGE);

  visual_tools.trigger();

  // Set the start state of the plan to the current state of the robot
  planning_components->setStartStateToCurrentState();


  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to plan");


  // // Add the cracker box to the planning scene
  // moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  // addCollisionBox(planning_scene_interface);


  // // Need transform between the robot and the camera optical link
  // tf2_ros::Buffer tfBuffer;
  // tf2_ros::TransformListener tfListener(tfBuffer);
  //
  // try
  // {
  //   ROS_INFO("Looking up transform between base_link and camera_rgb_optical_frame...");
  //   t_stamped_base_opt = tfBuffer.lookupTransform("base_link", "camera_rgb_optical_frame", ros::Time(2.0));
  // }
  //
  // catch (tf2::TransformException &ex)
  // {
  //   ROS_WARN("%s", ex.what());
  //   ROS_ERROR("No transform between base_link and camera_rgb_optical_frame found");
  //   ros::shutdown();
  // }
  //
  //
  //
  // ROS_INFO("Waiting for grasp candidate... ");
  // while(node_handle.ok())
  // {
  //   ros::spinOnce();
  //
  //   if (cloud_msg_received)
  //   {
  //     cloud_pub.publish(cloud_msg);
  //     cloud_msg_received = false;
  //   }
  //
  //   if (grasp_selected)
  //   {
  //     visualization_msgs::MarkerArray marker_array;
  //     visualizeGraspCandidate(marker_array);
  //     grasp_viz_pub.publish(marker_array);
  //     grasp_selected = false;
  //     break;
  //   }
  // }


  /////////////////////////////////////////////////////
  // Hard code goal for testing (Previous result from GPD)
  grasp_pose_robot.header.frame_id = "base_link";
  grasp_pose_robot.pose.position.x = 0.622204;
  grasp_pose_robot.pose.position.y = -0.038441;
  grasp_pose_robot.pose.position.z = 0.105954;

  grasp_pose_robot.pose.orientation.x = 0.00222623;
  grasp_pose_robot.pose.orientation.y = -0.00977791;
  grasp_pose_robot.pose.orientation.z = 0.255572;
  grasp_pose_robot.pose.orientation.w = 0.966738;

  tf::quaternionMsgToEigen(grasp_pose_robot.pose.orientation, grasp_candidate.quat);
  grasp_candidate.approach = grasp_candidate.quat.toRotationMatrix().col(0);
  /////////////////////////////////////////////////////

  // call the PlanningComponents to compute the pla
  planning_components->setGoal(grasp_pose_robot, "base_link");
  auto plan_solution = planning_components->plan();

  moveit_cpp_ptr->execute(PLANNING_GROUP, plan_solution.trajectory);


  /////////////////////////////////////////////////////
  // Publish approach vector
  // visualization_msgs::Marker marker;
  // marker.header.frame_id = "base_link";
  // marker.header.stamp = ros::Time();
  // marker.ns = "approach";
  // marker.id = 0;
  // marker.type = visualization_msgs::Marker::ARROW;
  // marker.action = visualization_msgs::Marker::ADD;
  // marker.pose = grasp_pose_robot.pose;
  // marker.lifetime = ros::Duration(20);
  //
  // marker.scale.x = 0.1;
  // marker.scale.y = 0.01;
  // marker.scale.z = 0.01;
  //
  // marker.color.a = 1.0;
  // marker.color.r = 1.0;
  // marker.color.g = 0.0;
  // marker.color.b = 0.0;
  //
  // grasp_approach_pub.publish(marker);
  // /////////////////////////////////////////////////////
  //
  //
  // /////////////////////////////////////////////////////
  // // Pick
  // visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to plan to pick");
  //
  // // Open gripper
  // trajectory_msgs::JointTrajectory gripper_state_msg;
  // setGripperState(gripper_state_msg, 0.0);
  // gripper_cmd_pub.publish(gripper_state_msg);
  //
  // // Pre-grasp displacement
  // geometry_msgs::Pose pre_grasp = preGrasp(0.25);
  // move_group.setPoseTarget(pre_grasp);
  // move_group.move();
  //
  // geometry_msgs::Pose grasp_goal = graspPose(pre_grasp, 0.1);
  // move_group.setPoseTarget(grasp_goal);
  // move_group.move();
  //
  // // Close gripper
  // setGripperState(gripper_state_msg, 0.3);
  // gripper_cmd_pub.publish(gripper_state_msg);
  //
  // // Attach box to gripper
  // move_group.attachObject("cracker_box");
  //
  // // Post-grasp displacement
  // geometry_msgs::Vector3 direction;
  // direction.x = 0.0;
  // direction.y = 0.0;
  // direction.z = 1.0;
  // geometry_msgs::Pose post_grasp = postGrasp(grasp_goal, direction, 0.1);
  //
  // move_group.setPoseTarget(post_grasp);
  // move_group.move();
  // /////////////////////////////////////////////////////
  //
  //
  // /////////////////////////////////////////////////////
  // // Place
  // geometry_msgs::Pose place_pose;
  // place_pose.position.x = 0.0;
  // place_pose.position.y = -0.40;
  // place_pose.position.z = 0.3;
  // place_pose.orientation.w = 1.0;
  //
  // move_group.setPoseTarget(place_pose);
  // move_group.move();
  //
  // // Open gripper
  // setGripperState(gripper_state_msg, 0.0);
  // gripper_cmd_pub.publish(gripper_state_msg);
  //
  // // Detach box to gripper
  // move_group.detachObject("cracker_box");
  /////////////////////////////////////////////////////

  ros::waitForShutdown();
  // ros::shutdown();
  return 0;
}


// end file
