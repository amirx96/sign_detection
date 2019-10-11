#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/ModelCoefficients.h>
#include "std_msgs/Float32.h"

ros::Publisher pub;
ros::Publisher vis_pub;
ros::Publisher lidar_pub;
std_msgs::Float32 lidar_msg;

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
{
  // Container for original & filtered data
  pcl::PCLPointCloud2 *cloud = new pcl::PCLPointCloud2; // pointer to empty pointcloud2 struct cloud
  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
  pcl::PCLPointCloud2 cloud_filtered; // defines cloud_filtered as type pointcloud2
  //pcl::PCLPointCloud2 cloud_projected;

  // i hate pointers

  // create filtering objects
  pcl::PassThrough<pcl::PCLPointCloud2> pass;
  pcl::StatisticalOutlierRemoval<pcl::PCLPointCloud2> sor;
  pcl::RadiusOutlierRemoval<pcl::PCLPointCloud2> outrem;

  // Convert to PCL data type
  pcl_conversions::toPCL(*cloud_msg, *cloud);

  // Perform the actual filtering

  // Filter intensity values
  pass.setInputCloud(cloudPtr);
  pass.setFilterFieldName("intensity");
  pass.setFilterLimits(80, 10000.0);
  pass.filter(*cloud);
  // Filter Z values
  if (cloud->width > 0)
  { //make sure that the point cloud has points
    pass.setFilterFieldName("z");
    pass.setFilterLimits(-5.0, 10.0);
    pass.filter(*cloud);
  }
  // filter X values
  if (cloud->width > 0)
  { //make sure that the point cloud has points
    pass.setFilterFieldName("x");
    pass.setFilterLimits(0, 100.0);
    pass.filter(*cloud);
  }
  //filter Y
  if (cloud->width > 0)
  { //make sure that the point cloud has points
    pass.setFilterFieldName("y");
    pass.setFilterLimits(-10.0, 0);
    pass.filter(*cloud);
  }

  if (cloud->width > 0)
  { //make sure that the point cloud has points
    // neighbor filterng
    outrem.setInputCloud(cloudPtr);
    outrem.setRadiusSearch(0.5);
    outrem.setMinNeighborsInRadius(3);
    outrem.filter(*cloud);
  }
  if (cloud->width > 0)
  { //make sure that the point cloud has points

    // statistical outlier removal
    sor.setInputCloud(cloudPtr);
    sor.setMeanK(70);
    sor.setStddevMulThresh(1.0);
    sor.filter(*cloud);
  }

  pcl::SACSegmentation<pcl::PointXYZI> seg;
  pcl::ProjectInliers<pcl::PointXYZI> proj;
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_xyzPtr(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr segPtr(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  pcl::fromPCLPointCloud2(*cloud, *cloud_xyzPtr);

  //------------------------------ PLANE FIITING ---------------------------------------//
  //------------------------------               ---------------------------------------//
  if (cloud->width > 15)
  {

    seg.setOptimizeCoefficients(true);
    // Mandatory
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.11);

    seg.setInputCloud(cloud_xyzPtr);
    seg.segment(*inliers, *coefficients);

    proj.setModelType(pcl::SACMODEL_PLANE);
    proj.setIndices(inliers);
    proj.setInputCloud(cloud_xyzPtr);
    proj.setModelCoefficients(coefficients);
    proj.filter(*segPtr);
    //------------------------------               ---------------------------------------//
    //------------------------------------------------------------------------------------//
  }

  if (segPtr->points.size() > 10)
  {

    // Compute average intensity
    float avg = 0.0;
    for (size_t i = 0; i < segPtr->points.size(); ++i)
    {
      avg = avg + segPtr->points[i].intensity;
    }
    avg = avg / segPtr->points.size();

    // Final Check for Sign
    if (avg > 80 && coefficients->values[0] > 0.90)
    { //IF intensity average and Normal vector meet specifications, convert segPtr to cloud_filtered and output

      //Convert Data and publish
      pcl::toPCLPointCloud2(*segPtr, cloud_filtered);
      ROS_INFO("SIGN DETECTED, X: %f", segPtr->points[0].x); // Printout X location 
      lidar_msg.data = segPtr->points[0].x;

      // Visualization marker stuff
      visualization_msgs::Marker marker;
      marker.header.frame_id = "map";
      marker.header.stamp = ros::Time();
      marker.ns = "my_namespace";
      marker.id = 0;
      marker.type = visualization_msgs::Marker::SPHERE;
      marker.action = visualization_msgs::Marker::ADD;
      marker.pose.position.x = segPtr->points[0].x;
      marker.pose.position.y = segPtr->points[0].y;
      marker.pose.position.z = segPtr->points[0].z;
      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = 0.5;
      marker.scale.y = 2.5;
      marker.scale.z = 2.5;
      marker.color.a = 1.0; // Don't forget to set the alpha!
      marker.color.r = 1.0;
      marker.color.g = 0.0;
      marker.color.b = 0.0;
      //only if using a MESH_RESOURCE marker type:
      vis_pub.publish(marker);
      lidar_pub.publish(lidar_msg);
    }
  }

  // Convert to ROS data type & Publish
  sensor_msgs::PointCloud2 output;
  pcl_conversions::fromPCL(cloud_filtered, output);
  pub.publish(output);
}

int main(int argc, char **argv)
{
  // Initialize ROS
  ros::init(argc, argv, "sign_detection");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe("velodyne_points", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2>("sd_pointcloud_filtered", 1);

  // Create a ROS publisher for the marker output
  vis_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 0);

  // Create a ROS publisher for the sign distance
  lidar_pub = nh.advertise<std_msgs::Float32>("sd_distance", 1000);

  // Spin
  ros::spin();
}
