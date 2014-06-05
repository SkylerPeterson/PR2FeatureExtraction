#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
// PCL specific includes
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl_ros/point_cloud.h>

ros::Publisher pub;

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input) {
  
  // convert point cloud
  pcl::PointCloud< pcl::PointXYZ> output;
  pcl::fromROSMsg(*input, output);
  
  // Write point cloud to disk
  std::stringstream ss;
  pcl::PCDWriter writer;
  ss << "/home/skyler/catkin_groovy_ws/cloud_cluster.pcd";
  writer.write<pcl::PointXYZ>(ss.str(), output, false); //*
  
  // Publish the data
  pub.publish(output);

}

int main(int argc, char** argv) {
  // Initialize ROS
  ros::init(argc, argv, "ros_to_pcl_cloud");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe("/camera/depth/points", 1, cloud_cb);
  
  // Create a ROS publisher for the output point cloud
  pub = nh.advertise< pcl::PointCloud< pcl::PointXYZ > >("/pcl_pointcloud", 1);

  // Spin
  ros::spin();
}
