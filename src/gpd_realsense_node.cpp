/**
* @file gpd_realsense_node.cpp
* @author Boston Cleek
* @date 29 Jun 2020
* @brief Uses point cloud data from a Realsense depth camera
*        as the input to grasp pose detection to select a grasp candidate
*
* @PUBLISHES:
* @SUBSCRIBES:
* @SEERVICES:
*/

// ROS
#include <ros/ros.h>
#include <ros/console.h>
#include <std_srvs/Empty.h>
#include <pcl_conversions/pcl_conversions.h>

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


static bool cloud_srv_active;                                   // request grasps status
static bool cloud_msg_received;                                 // point cloud message received status


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



    // // segment objects from table
    // removeTable(cloud);
    // ROS_INFO("Segmented point cloud size: %lu ", cloud->points.size());
    //
    // if (cloud->points.empty())
    // {
    //   ROS_ERROR("Cloud empty");
    //   // ros::shutdown();
    // }
    //
    // // covert back to ROS msg
    // pcl::toROSMsg(*cloud, cloud_msg);



    pcl::visualization::CloudViewer viewer ("Cloud");
    viewer.showCloud (cloud);
    while (!viewer.wasStopped ()) {}


    cloud_srv_active = false;
    cloud_msg_received = true;
  }
}



int main(int argc, char** argv)
{
  ros::init(argc, argv, "gpd_realsense");
  ros::NodeHandle nh("~");
  ros::NodeHandle node_handle;

  ros::Subscriber cloud_sub = node_handle.subscribe("camera/depth/color/points", 1, cloudCallBack);
  ros::ServiceServer cloud_serv = node_handle.advertiseService("request_grasp", requestGraspCloudCallBack);

  ros::spin();

  return 0;
}


// end file
