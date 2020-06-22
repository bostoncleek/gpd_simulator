/// \file
/// \brief publish a point cloud and retrieve grasp from GPD
///
/// \author Boston Cleek
/// \date 6/18/20
///
/// PUBLISHES:
/// SUBSCRIBES:
/// SEERVICES:


#include <ros/ros.h>
#include <ros/console.h>
#include <std_srvs/Empty.h>
// #include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <vector>
#include <iostream>

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

#include <gpd_ros/CloudSources.h>
#include <gpd_ros/CloudSamples.h>
#include <gpd_ros/GraspConfig.h>
#include <gpd_ros/GraspConfigList.h>


static sensor_msgs::PointCloud2 cloud_msg;
static bool cloud_srv_active;
static bool cloud_msg_received;





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
  ROS_INFO("number of graps received %lud", msg->grasps.size());
}



int main(int argc, char** argv)
{
  ros::init(argc, argv, "gpd_sim");
  ros::NodeHandle nh("~");
  ros::NodeHandle node_handle;

  ros::Subscriber cloud_sub = node_handle.subscribe("depth_camera/depth/points", 1, cloudCallBack);
  ros::Subscriber grasp_sub = node_handle.subscribe("detect_grasps/clustered_grasps", 1, graspCallBack);

  // ros::Publisher cloud_pub = node_handle.advertise<gpd_ros::CloudSamples>("cloud_stitched", 1);
  ros::Publisher cloud_pub = node_handle.advertise<sensor_msgs::PointCloud2>("cloud_stitched", 1);
  ros::ServiceServer cloud_serv = node_handle.advertiseService("request_grasp", requestGraspCloudCallBack);

  ROS_INFO("Successfully launch gpd_sim node");

  cloud_srv_active = false;
  cloud_msg_received = false;


  while(node_handle.ok())
  {
    ros::spinOnce();

    if (cloud_msg_received)
    {

      cloud_pub.publish(cloud_msg);

      cloud_msg_received = false;
    }

  }

  return 0;
}


// end file
