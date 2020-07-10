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
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

// C++
#include <vector>
#include <iostream>
#include <string>
#include <cmath>
#include <limits>


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
static Gripper grasp_candidate;                                 // best gpd grasp rep. using eigen

static bool cloud_srv_active;                                   // request grasps status
static bool cloud_msg_received;                                 // point cloud message received status
static bool grasp_selected;                                     // publish selected grasp


// Inliers representing points in the plane
// static pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices);
// Use a plane as the model for the segmentor
// static pcl::ModelCoefficients::Ptr coefficients_plane(new pcl::ModelCoefficients);
// static pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);



// TODO read params in through server
static const auto outer_diameter = 0.12;
static const auto hand_depth = 0.06;
static const auto finger_width = 0.01;
static const auto hand_height = 0.02;

// bounds on point cloud in base_lik frame
static auto x_max = 0.0;
static auto x_min = 0.0;

static auto y_max = 0.0;
static auto y_min = 0.0;

static auto z_max = 0.0;
static auto z_min = 0.0;




/**
   * Source code: pcl_ros/transfoms.h (unable to compile both pcl_ros and PCL)
   * \brief Transform a sensor_msgs::PointCloud2 dataset using an Eigen 4x4 matrix.
   * \param transform the transformation to use on the points
   * \param in the input PointCloud2 dataset
   * \param out the resultant transformed PointCloud2 dataset
   */
void transformPointCloud(const Eigen::Matrix4f &transform, const sensor_msgs::PointCloud2 &in, sensor_msgs::PointCloud2 &out)
{
  // Get X-Y-Z indices
  int x_idx = pcl::getFieldIndex (in, "x");
  int y_idx = pcl::getFieldIndex (in, "y");
  int z_idx = pcl::getFieldIndex (in, "z");

  if (x_idx == -1 || y_idx == -1 || z_idx == -1)
  {
    ROS_ERROR ("Input dataset has no X-Y-Z coordinates! Cannot convert to Eigen format.");
    return;
  }

  if (in.fields[x_idx].datatype != sensor_msgs::PointField::FLOAT32 ||
      in.fields[y_idx].datatype != sensor_msgs::PointField::FLOAT32 ||
      in.fields[z_idx].datatype != sensor_msgs::PointField::FLOAT32)
  {
    ROS_ERROR ("X-Y-Z coordinates not floats. Currently only floats are supported.");
    return;
  }

  // Check if distance is available
  int dist_idx = pcl::getFieldIndex (in, "distance");

  // Copy the other data
  if (&in != &out)
  {
    out.header = in.header;
    out.height = in.height;
    out.width  = in.width;
    out.fields = in.fields;
    out.is_bigendian = in.is_bigendian;
    out.point_step   = in.point_step;
    out.row_step     = in.row_step;
    out.is_dense     = in.is_dense;
    out.data.resize (in.data.size ());
    // Copy everything as it's faster than copying individual elements
    memcpy (&out.data[0], &in.data[0], in.data.size ());
  }

  Eigen::Array4i xyz_offset (in.fields[x_idx].offset, in.fields[y_idx].offset, in.fields[z_idx].offset, 0);

  for (size_t i = 0; i < in.width * in.height; ++i)
  {
    Eigen::Vector4f pt (*(float*)&in.data[xyz_offset[0]], *(float*)&in.data[xyz_offset[1]], *(float*)&in.data[xyz_offset[2]], 1);
    Eigen::Vector4f pt_out;

    bool max_range_point = false;
    int distance_ptr_offset = (dist_idx < 0 ? -1 : (i*in.point_step + in.fields[dist_idx].offset)); // If dist_idx is negative, it must not be used as an index
    float* distance_ptr = (dist_idx < 0 ? NULL : (float*)(&in.data[distance_ptr_offset]));
    if (!std::isfinite (pt[0]) || !std::isfinite (pt[1]) || !std::isfinite (pt[2]))
    {
      if (distance_ptr==NULL || !std::isfinite(*distance_ptr))  // Invalid point
      {
        pt_out = pt;
      }
      else  // max range point
      {
        pt[0] = *distance_ptr;  // Replace x with the x value saved in distance
        pt_out = transform * pt;
        max_range_point = true;
        //std::cout << pt[0]<<","<<pt[1]<<","<<pt[2]<<" => "<<pt_out[0]<<","<<pt_out[1]<<","<<pt_out[2]<<"\n";
      }
    }
    else
    {
      pt_out = transform * pt;
    }

    if (max_range_point)
    {
      // Save x value in distance again
      *(float*)(&out.data[distance_ptr_offset]) = pt_out[0];
      pt_out[0] = std::numeric_limits<float>::quiet_NaN();
    }

    memcpy (&out.data[xyz_offset[0]], &pt_out[0], sizeof (float));
    memcpy (&out.data[xyz_offset[1]], &pt_out[1], sizeof (float));
    memcpy (&out.data[xyz_offset[2]], &pt_out[2], sizeof (float));


    xyz_offset += in.point_step;
  }

  // Check if the viewpoint information is present
  int vp_idx = pcl::getFieldIndex (in, "vp_x");
  if (vp_idx != -1)
  {
    // Transform the viewpoint info too
    for (size_t i = 0; i < out.width * out.height; ++i)
    {
      float *pstep = (float*)&out.data[i * out.point_step + out.fields[vp_idx].offset];
      // Assume vp_x, vp_y, vp_z are consecutive
      Eigen::Vector4f vp_in (pstep[0], pstep[1], pstep[2], 1);
      Eigen::Vector4f vp_out = transform * vp_in;

      pstep[0] = vp_out[0];
      pstep[1] = vp_out[1];
      pstep[2] = vp_out[2];
    }
  }
}


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
  }
}


/**
* @brief Filter points above a specific height
* @param cloud - filtered cloud
*/
void passThroughFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr filter_cloud)
{
  pcl::PassThrough<pcl::PointXYZRGB> pass;
  pass.setInputCloud(filter_cloud);

  pass.setFilterFieldName("x");
  pass.setFilterLimits(x_min, x_max);
  pass.filter(*filter_cloud);

  pass.setFilterFieldName("y");
  pass.setFilterLimits(y_min, y_max);
  pass.filter(*filter_cloud);

  pass.setFilterFieldName("z");
  pass.setFilterLimits(z_min, z_max);
  pass.filter(*filter_cloud);
}


/**
* @brief Call back for requesting grasp empty service
* @return true when called
* @details this is an empty service
*/
bool requestGraspCloudCallBack(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
{
  ROS_INFO("Request grasp service activated");
  ROS_INFO("Point cloud selected");
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
  // Convert transform from base_link to optical frame to a eigen matrix
  Eigen::Matrix4f eigen_transform_base_optical = tf2::transformToEigen(t_stamped_base_opt).matrix().cast<float>();

  // transfom point cloud into frame of base_link
  transformPointCloud(eigen_transform_base_optical, *msg.get(), cloud_msg);
  cloud_msg.header.frame_id = t_stamped_base_opt.header.frame_id;


  // convert from ROS msg to a point cloud
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

  pcl::fromROSMsg(cloud_msg, *cloud);
  ROS_INFO("Point cloud size: %lu ", cloud->points.size());


  // pcl::PointXYZRGB min_pt, max_pt;
  // pcl::getMinMax3D(*cloud, min_pt, max_pt);
  //
  // std::cout << "Pre filter" << std::endl;
  // std::cout << "min x: " << min_pt.x << std::endl;
  // std::cout << "min y: " << min_pt.y << std::endl;
  // std::cout << "min z: " << min_pt.z << std::endl<<std::endl;
  //
  // std::cout << "max x: " << max_pt.x << std::endl;
  // std::cout << "max y: " << max_pt.y << std::endl;
  // std::cout << "max z: " << max_pt.z << std::endl<<std::endl;


  // remove points bellow a vertical threshold
  passThroughFilter(cloud);


  // segment objects from table
  removeTable(cloud);
  ROS_INFO("Segmented point cloud size: %lu ", cloud->points.size());


  if (cloud->points.empty())
  {
    ROS_ERROR("Cloud empty");
  }

  // // covert back to ROS msg
  pcl::toROSMsg(*cloud, cloud_msg);

  cloud_msg_received = true;
}



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

  ros::Publisher filter_cloud_pub = node_handle.advertise<sensor_msgs::PointCloud2>("filtered_cloud", 1);
  ros::Publisher gpd_cloud_pub = node_handle.advertise<sensor_msgs::PointCloud2>("cloud_stitched", 1);
  ros::Publisher grasp_viz_pub = node_handle.advertise<visualization_msgs::MarkerArray>("grasp_candidate", 1);

  ros::ServiceServer cloud_serv = node_handle.advertiseService("request_grasp", requestGraspCloudCallBack);

  nh.getParam("x_min", x_min);
  nh.getParam("x_max", x_max);
  nh.getParam("y_min", y_min);
  nh.getParam("y_max", y_max);
  nh.getParam("z_min", z_min);
  nh.getParam("z_max", z_max);

  ROS_INFO("Successfully launch gpd_realsense node");

  cloud_srv_active = false;
  cloud_msg_received = false;
  grasp_selected = false;

  // point cloud is in frame: camera_rgb_optical_frame
  // need transform from base_link to camera_depth_optical_frame
  // to limit bounds of point cloud
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);

  // wait for things to get up and running
  ros::Duration(1.0).sleep();

  try
  {
    ROS_INFO("Looking up transform between base_link and camera_depth_optical_frame...");
    t_stamped_base_opt = tfBuffer.lookupTransform("base_link", "camera_depth_optical_frame", ros::Time(2.0));
  }

  catch (tf2::TransformException &ex)
  {
    ROS_WARN("%s", ex.what());
    ROS_ERROR("No transform between base_link and camera_depth_optical_frame found");
    ros::shutdown();
  }


  // std::cout << "t_stamped_base_opt" << std::endl;
  // std::cout << t_stamped_base_opt << std::endl;


  ROS_INFO("Waiting for grasp candidate... ");
  while(node_handle.ok())
  {
    ros::spinOnce();

    if (cloud_msg_received)
    {
      filter_cloud_pub.publish(cloud_msg);
      cloud_msg_received = false;

      if (cloud_srv_active)
      {
        gpd_cloud_pub.publish(cloud_msg);
        cloud_srv_active = false;
      }
    }


    if (grasp_selected)
    {
      // marker array viz best grasp
      visualization_msgs::MarkerArray marker_array;
      visualizeGraspCandidate(marker_array);
      grasp_viz_pub.publish(marker_array);
      grasp_selected = false;
    }
  }

  return 0;
}


// end file
