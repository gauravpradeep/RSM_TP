#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>
ros::Publisher pointcloud_pub;
tf::TransformListener* tf_listener_ptr;

void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& input_cloud)
{

  if (!tf_listener_ptr->waitForTransform("map", "zed_depth_camera", input_cloud->header.stamp, ros::Duration(2.0)))
  {
    ROS_WARN("Transform not available");
    return;
  }
  // Convert the input point cloud to PCL format
  pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
  pcl::fromROSMsg(*input_cloud, pcl_cloud);

  // Lookup the transformation matrix between source and target frames
  tf::StampedTransform transform;
  try
  {
	
    tf_listener_ptr->lookupTransform("map", "zed_depth_camera", ros::Time(0), transform);
  }
  catch (tf::TransformException& ex)
  {
    ROS_ERROR("%s", ex.what());
    return;
  }

  // Transform the point cloud
  pcl::PointCloud<pcl::PointXYZ> transformed_cloud;
  pcl_ros::transformPointCloud(pcl_cloud, transformed_cloud, transform);

  // Convert the transformed point cloud back to ROS format
  sensor_msgs::PointCloud2 transformed_cloud_msg;
  pcl::toROSMsg(transformed_cloud, transformed_cloud_msg);
  transformed_cloud_msg.header = input_cloud->header;
  transformed_cloud_msg.header.frame_id = "map";

  // Publish the transformed point cloud
  pointcloud_pub.publish(transformed_cloud_msg);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pointcloud_transformer");
  ros::NodeHandle nh;

  tf::TransformListener tf_listener;
  tf_listener_ptr = &tf_listener;

  // Advertise the output point cloud topic
  pointcloud_pub = nh.advertise<sensor_msgs::PointCloud2>("transformed_pointcloud", 1);

  // Subscribe to the input point cloud topic
  ros::Subscriber pointcloud_sub = nh.subscribe("zed/depth/points", 1, pointCloudCallback);

  ros::spin();

  return 0;
}
