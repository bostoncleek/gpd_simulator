/**
* @file gpd_realsense_node.cpp
* @author Boston Cleek
* @date 29 Jun 2020
* @brief Uses point cloud data from a Realsense depth camera
*        as the input to grasp pose detection to select a grasp candidate
*
* @PUBLISHES:
*   grasp_candidate (visualization_msgs/MarkerArray): best grasp candidate visualization
*   cloud_stitched (sensor_msgs::PointCloud2): filtered point cloud for GPD
* @SUBSCRIBES:
*   camera/depth/color/points (sensor_msgs/PointCloud2): raw point cloud
*   detect_grasps/clustered_grasps (gpd_ros/GraspConfigList): list of all grasp candidates from GPD
* @SEERVICES:
*   request_grasp (std_srvs/Empty): requests grasp candidates and selects current point cloud as the input to GPD
*/

// ROS
#include <ros/ros.h>
#include <ros/console.h>
#include <std_srvs/Empty.h>
#include <eigen_conversions/eigen_msg.h>
#include <std_srvs/Empty.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

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
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/visualization/cloud_viewer.h>

// GPD
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
static Gripper grasp_candidate;

static bool cloud_srv_active;                                   // request grasps status
static bool cloud_msg_received;                                 // point cloud message received status
static bool grasp_selected;                                     // publish selected grasp


// TODO read params in through server
static const auto outer_diameter = 0.12;
static const auto hand_depth = 0.06;
static const auto finger_width = 0.01;
static const auto hand_height = 0.02;


static auto y_max = 0.0;
static auto y_min = 0.0;
static auto y_padding = 0.05;

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

  pcl::visualization::CloudViewer viewer ("Cloud");
  viewer.showCloud (cloud);
  while (!viewer.wasStopped ()) {}
}


/**
* @brief Filter points above a specific height
* @param cloud - filtered cloud
*/
void passThroughFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
  pcl::PassThrough<pcl::PointXYZRGB> pass;
  pass.setInputCloud(cloud);
  pass.setFilterFieldName("y");
  // min and max values in z axis to keep
  pass.setFilterLimits(y_min, y_max - y_padding);
  pass.filter(*cloud);
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
    // ROS_INFO("Point cloud selected");
    ROS_INFO("Frame ID: %s", msg->header.frame_id.c_str());

    // use RANSAC to filter points above table
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    // // convert from ROS msg to a point cloud
    pcl::fromROSMsg(*msg.get(), *cloud);
    // ROS_INFO("Point cloud size: %lu ", cloud->points.size());

    // pcl::PointXYZRGB min_pt, max_pt;
    // pcl::getMinMax3D(*cloud, min_pt, max_pt);
    //
    // // std::cout << "Pre filter" << std::endl;
    // // std::cout << "min x: " << min_pt.x << std::endl;
    // // std::cout << "min y: " << min_pt.y << std::endl;
    // // std::cout << "min z: " << min_pt.z << std::endl<<std::endl;
    // //
    // // std::cout << "max x: " << max_pt.x << std::endl;
    // // std::cout << "max y: " << max_pt.y << std::endl;
    // // std::cout << "max z: " << max_pt.z << std::endl<<std::endl;
    //
    // y_min = min_pt.y;
    // y_max = max_pt.y;


    // remove points bellow a vertical threshold
    // passThroughFilter(cloud);


    // segment objects from table
    removeTable(cloud);
    ROS_INFO("Segmented point cloud size: %lu ", cloud->points.size());

    if (cloud->points.empty())
    {
      ROS_ERROR("Cloud empty");
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

  std::cout << "Pose of grasp: " << grasp_pose_robot << std::endl;
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


int main(int argc, char** argv)
{
  ros::init(argc, argv, "gpd_realsense");
  ros::NodeHandle nh("~");
  ros::NodeHandle node_handle;

  ros::Subscriber cloud_sub = node_handle.subscribe("camera/depth/color/points", 1, cloudCallBack);
  ros::Subscriber grasp_sub = node_handle.subscribe("detect_grasps/clustered_grasps", 1, graspCallBack);

  ros::Publisher cloud_pub = node_handle.advertise<sensor_msgs::PointCloud2>("cloud_stitched", 1);
  ros::Publisher grasp_viz_pub = node_handle.advertise<visualization_msgs::MarkerArray>("grasp_candidate", 1);

  ros::ServiceServer cloud_serv = node_handle.advertiseService("request_grasp", requestGraspCloudCallBack);

  ROS_INFO("Successfully launch gpd_realsense node");

  cloud_srv_active = false;
  cloud_msg_received = false;
  grasp_selected = false;


  // Need transform between the robot and the camera optical link
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);

  // point cloud is in frame: camera_rgb_optical_frame
  // need transform from base_link to camera_depth_optical_frame
  // to limit bounds of point cloud

  geometry_msgs::TransformStamped T_base_to_optical;

  // wait for things to get up and running
  ros::Duration(1.0).sleep();


  try
  {
    ROS_INFO("Looking up transform between base_link and camera_depth_optical_frame...");
    T_base_to_optical = tfBuffer.lookupTransform("base_link", "camera_depth_optical_frame", ros::Time(2.0));
  }

  catch (tf2::TransformException &ex)
  {
    ROS_WARN("%s", ex.what());
    ROS_ERROR("No transform between base_link and camera_depth_optical_frame found");
  }




  // // transform from camera_link to camera_rgb_optical_frame
  // geometry_msgs::TransformStamped T_camera_to_optical;
  //
  // // transform from base_link to camera_link
  // geometry_msgs::TransformStamped T_base_to_camera;
  //
  //
  // // wait for things to get up and running
  // ros::Duration(1.0).sleep();
  //
  //
  // try
  // {
  //   ROS_INFO("Looking up transform between base_link and camera_link...");
  //   T_base_to_camera = tfBuffer.lookupTransform("base_link", "camera_link", ros::Time(2.0));
  // }
  //
  // catch (tf2::TransformException &ex)
  // {
  //   ROS_WARN("%s", ex.what());
  //   ROS_ERROR("No transform between base_link and camera_link found");
  // }
  //
  // try
  // {
  //   ROS_INFO("Looking up transform between camera_link and camera_depth_optical_frame...");
  //   T_camera_to_optical = tfBuffer.lookupTransform("camera_link", "camera_depth_optical_frame", ros::Time(2.0));
  // }
  //
  // catch (tf2::TransformException &ex)
  // {
  //   ROS_WARN("%s", ex.what());
  //   ROS_ERROR("No transform between camera_link and camera_depth_optical_frame found");
  // }
  //
  // // transform from base_link to camera_depth_optical_frame
  // geometry_msgs::TransformStamped T_base_to_optical;
  //
  // // T_base_to_optical = T_base_to_camera * T_camera_to_optical
  // // tf2::doTransform(T_base_to_camera, T_base_to_optical, T_camera_to_optical);
  // tf2::doTransform(T_camera_to_optical, T_base_to_optical, T_base_to_camera);
  //
  //
  //
  // std::cout << "T_base_to_camera" << std::endl;
  // std::cout << T_base_to_camera << std::endl;
  //
  // std::cout << "T_camera_to_optical" << std::endl;
  // std::cout << T_camera_to_optical << std::endl;
  //
  std::cout << "T_base_to_optical" << std::endl;
  std::cout << T_base_to_optical << std::endl;



  // // assume camera_depth_optical_frame is at base_link
  // std::string frame_id = "camera_depth_optical_frame";
  // grasp_candidate.frame_id = frame_id;
  // t_stamped_base_opt.header.frame_id = frame_id;
  // t_stamped_base_opt.child_frame_id = frame_id;
  // t_stamped_base_opt.transform.rotation.w = 1.0;
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
  //   }
  // }

  return 0;
}


// end file
