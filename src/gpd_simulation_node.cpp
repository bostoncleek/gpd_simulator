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

#include <pcl/common/common.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

#include <gpd_ros/CloudSources.h>
#include <gpd_ros/CloudSamples.h>
#include <gpd_ros/GraspConfig.h>
#include <gpd_ros/GraspConfigList.h>


static sensor_msgs::PointCloud2::ConstPtr point_cloud;
static bool cloud_srv;
static bool cloud_msg;



bool requestGraspCloudCallBack(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
{
  cloud_srv = true;
  return true;
}



void cloudCallBack(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
  if (cloud_srv)
  {
    ROS_INFO("point cloud selected");

    point_cloud = msg;

    // // use RANSAC to filter points above table
    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);
    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    //
    // // convert from ROS msg to a point cloud
    // pcl::fromROSMsg(*msg.get(), *cloud_in);
    //
    // // find inliers to plane
    // pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    // pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    // pcl::SACSegmentation<pcl::PointXYZ> seg;
    // seg.setOptimizeCoefficients(true);
    // seg.setModelType(pcl::SACMODEL_PLANE);
    // seg.setMethodType(pcl::SAC_RANSAC);
    // seg.setDistanceThreshold (0.01);
    //
    // seg.setInputCloud(cloud_in);
    // seg.segment(*inliers, *coefficients);
    //
    // if (inliers->indices.size () == 0)
    // {
    //   ROS_ERROR("Could not estimate a planar model for the given dataset");
    // }
    //
    // // extract indices of points not within plane
    // std::vector<int> outliers;
    // pcl::ExtractIndices<pcl::PointXYZ> extract;
    // extract.setInputCloud(cloud_in);
    // extract.setIndices(inliers);
    // extract.setNegative(true);
    // // extract.filter(outliers);
    //
    // // fill point cloud with outliers
    // // extract.setIndices(outliers);
    // // extract.filter(cloud_filtered);
    //
    // // // covert back to ROS msg
    // // pcl::moveFromROSMsg(point_cloud, *cloud_filtered);


    cloud_srv = false;
    cloud_msg = true;
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

  ROS_INFO("Successfully launch gpd_sim");

  cloud_srv = false;
  cloud_msg = false;


  while(node_handle.ok())
  {
    ros::spinOnce();

    if (cloud_msg)
    {
      // gpd_ros::CloudSources cloud_sources;
      // cloud_sources.cloud = *point_cloud.get();
      //
      // gpd_ros::CloudSamples cloud_samples;
      // cloud_samples.cloud_sources = cloud_sources;

      // could_pub.publish(cloud_samples);
      cloud_pub.publish(*point_cloud.get());

      cloud_msg = false;
    }

  }

  return 0;
}


// end file
