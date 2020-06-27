/// \file
/// \brief publish a point cloud and retrieve grasp from GPD
///
/// \author Boston Cleek
/// \date 6/18/20
///
/// PUBLISHES:
/// SUBSCRIBES:
/// SEERVICES:

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


struct Gripper
{
  // Translation from frame to gripper
  Eigen::Vector3d position;

  // Orientation of gripper, R = [approach binormal axis]
  Eigen::Quaterniond quat;
  Eigen::Vector3d approach;
  Eigen::Vector3d binormal;
  Eigen::Vector3d axis;

  float width;
  float score;

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
  // while (!viewer.wasStopped ())
  // {
  // }
}


bool requestGraspCloudCallBack(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
{
  ROS_INFO("Request grasp service activated");
  cloud_srv_active = true;
  return true;
}


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

  std::cout << "Pose of grasp in base_link frame: " << grasp_pose_robot << std::endl;


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

  grasp_selected = true;
}


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

  marker.color.a = 0.5;
  marker.color.r = 0.0;
  marker.color.g = 0.5;
  marker.color.b = 0.0;

  return marker;
}


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

  marker.color.a = 0.5;
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;

  return marker;
}


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


void openGripper(trajectory_msgs::JointTrajectory& posture)
{
  posture.joint_names.resize(1);
  posture.joint_names[0] = "robotiq_85_left_knuckle_joint";

  posture.points.resize(1);
  posture.points[0].positions.resize(1);
  posture.points[0].positions[0] = 0.04;
  posture.points[0].time_from_start = ros::Duration(0.5);
}


void addCollisionObjects(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface)
{

  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.resize(2);

  // Add the first table where the cube will originally be kept
  collision_objects[0].id = "table";
  collision_objects[0].header.frame_id = "world";

  // Define the primitive and its dimensions
  collision_objects[0].primitives.resize(1);
  collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
  collision_objects[0].primitives[0].dimensions.resize(3);
  collision_objects[0].primitives[0].dimensions[0] = 3.0;
  collision_objects[0].primitives[0].dimensions[1] = 3.0;
  collision_objects[0].primitives[0].dimensions[2] = 0.9144;

  // Define the pose of the table
  collision_objects[0].primitive_poses.resize(1);
  collision_objects[0].primitive_poses[0].position.x = 0.0;
  collision_objects[0].primitive_poses[0].position.y = 0.0;
  collision_objects[0].primitive_poses[0].position.z = 0.4572;
  collision_objects[0].primitive_poses[0].orientation.w = 1.0;

  collision_objects[0].operation = collision_objects[0].ADD;


  // Add the cracker box
  collision_objects[1].id = "cracker_box";
  collision_objects[1].header.frame_id = "world";

  // Define the primitive and its dimensions
  collision_objects[1].primitives.resize(1);
  collision_objects[1].primitives[0].type = collision_objects[1].primitives[0].BOX;
  collision_objects[1].primitives[0].dimensions.resize(3);
  collision_objects[1].primitives[0].dimensions[0] = 0.16;
  collision_objects[1].primitives[0].dimensions[1] = 0.06;
  collision_objects[1].primitives[0].dimensions[2] = 0.21;

  // Define the pose of the table
  collision_objects[1].primitive_poses.resize(1);
  collision_objects[1].primitive_poses[0].position.x = 0;
  collision_objects[1].primitive_poses[0].position.y = -0.5;
  collision_objects[1].primitive_poses[0].position.z = 1.0194;
  collision_objects[1].primitive_poses[0].orientation.w = 0.5;
  collision_objects[1].primitive_poses[0].orientation.z = 0.866;

  collision_objects[1].operation = collision_objects[1].ADD;

  // Apply objects to planning scene
  planning_scene_interface.applyCollisionObjects(collision_objects);
}




int main(int argc, char** argv)
{
  ros::init(argc, argv, "gpd_sim");
  ros::NodeHandle nh("~");
  ros::NodeHandle node_handle;

  ros::Subscriber cloud_sub = node_handle.subscribe("camera/depth/points", 1, cloudCallBack);
  // ros::Subscriber cloud_sub = node_handle.subscribe("depth_camera/depth/points", 1, cloudCallBack);

  ros::Subscriber grasp_sub = node_handle.subscribe("detect_grasps/clustered_grasps", 1, graspCallBack);
  ros::Publisher grasp_viz_pub = node_handle.advertise<visualization_msgs::MarkerArray>("grasp_candidate", 1);
  ros::Publisher cloud_pub = node_handle.advertise<sensor_msgs::PointCloud2>("cloud_stitched", 1);
  ros::ServiceServer cloud_serv = node_handle.advertiseService("request_grasp", requestGraspCloudCallBack);

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);

  ROS_INFO("Successfully launch gpd_sim node");

  cloud_srv_active = false;
  cloud_msg_received = false;
  grasp_selected = false;

  // Required
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // Move group interface
  static const std::string PLANNING_GROUP = "manipulator";
  static const std::string POSE_REF_FRAME = "base_link";
  static const std::string END_EFFECTOR_LINK = "ee_link";

  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
  move_group.setPlanningTime(45.0);

  move_group.setEndEffectorLink(END_EFFECTOR_LINK);
  move_group.setPoseReferenceFrame(POSE_REF_FRAME);

  const moveit::core::JointModelGroup* joint_model_group =
       move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  // Setup visualization
  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("shoulder_link");
  visual_tools.deleteAllMarkers();
  visual_tools.loadRemoteControl();

  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.75;
  visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);

  visual_tools.trigger();

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


  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window");


  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  addCollisionObjects(planning_scene_interface);


  moveit_msgs::Grasp grasp_msg;

  // openGripper(grasp_msg.pre_grasp_posture);




  // spinner.stop();

  // ROS_INFO("Waiting for grasp candidate... ");
  // while(node_handle.ok())
  // {
  //   ros::spinOnce();
  //
  //   try
  //   {
  //     // ROS_INFO("Looking up transform between base_link and camera_rgb_optical_frame...");
  //     t_stamped_base_opt = tfBuffer.lookupTransform("base_link", "camera_rgb_optical_frame", ros::Time(2.0));
  //   }
  //
  //   catch (tf2::TransformException &ex)
  //   {
  //     ROS_WARN("%s", ex.what());
  //     // ROS_ERROR("No transform between base_link and camera_rgb_optical_frame found");
  //     // ros::shutdown();
  //   }
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
  //
  // // spinner.start();
  //
  //
  // visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to plan to the pose goal");
  //
  // // Set pose goal from GPD
  // // move_group.setGoalPositionTolerance(0.05);
  // // move_group.setGoalOrientationTolerance(0.05);
  // move_group.setPlanningTime(30.0);
  // // grasp_pose_robot.pose.position.z += 0.2;
  // move_group.setPoseTarget(grasp_pose_robot.pose);
  //
  //
  // // Compose motion plan
  // moveit::planning_interface::MoveGroupInterface::Plan plan;
  // bool success = (move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  // ROS_INFO_NAMED("GPD", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
  //
  // // Visualizing plans
  // ROS_INFO_NAMED("GPD", "Visualizing plan 1 as trajectory line");
  // visual_tools.publishAxisLabeled(grasp_pose_robot.pose, "graps pose");
  // visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
  // visual_tools.publishTrajectoryLine(plan.trajectory_, joint_model_group);
  // visual_tools.trigger();
  // visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to move to the pose goal");
  //
  // // Move to the goal
  // // move_group.move();
  // move_group.execute(plan);


  ros::shutdown();
  return 0;
}


// end file
