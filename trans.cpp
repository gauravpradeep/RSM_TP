#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "transformation_node");
  ros::NodeHandle nh;

  tf::TransformBroadcaster broadcaster;
  tf::TransformListener listener;

  // Wait for the transformation to become available
  listener.waitForTransform("map", "zed_depth_camera", ros::Time(0), ros::Duration(5));

  ros::Rate rate(10.0);
  while (nh.ok())
  {
    try
    {
      tf::StampedTransform transform;
      listener.lookupTransform("map", "zed_depth_camera", ros::Time(0), transform);

      // Print the transformation matrix
      tf::Matrix3x3 rotation = transform.getBasis();
      tf::Vector3 translation = transform.getOrigin();

      ROS_INFO_STREAM("Transformation Matrix (zed_depth_camera to map):");
      ROS_INFO_STREAM(rotation[0][0] << " " << rotation[0][1] << " " << rotation[0][2] << " " << translation.x());
      ROS_INFO_STREAM(rotation[1][0] << " " << rotation[1][1] << " " << rotation[1][2] << " " << translation.y());
      ROS_INFO_STREAM(rotation[2][0] << " " << rotation[2][1] << " " << rotation[2][2] << " " << translation.z());

      // Publish the transformation
      broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "zed_depth_camera"));
    }
    catch (tf::TransformException& ex)
    {
      ROS_ERROR_STREAM("Error during transformation: " << ex.what());
    }

    rate.sleep();
  }

  return 0;
}
