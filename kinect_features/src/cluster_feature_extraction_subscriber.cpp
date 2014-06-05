#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

class MoIF {
public:
  static int moment_of_inertia_features(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& inputCluster);
};

int main(int argc, char** argv) {
  // Initialize ROS
  ros::init(argc, argv, "features_from_pcl_cluster");
  ros::NodeHandle nh;
  // Create a ROS subscriber for the input point cloud cluster
  ros::Subscriber sub = nh.subscribe<pcl::PointCloud<pcl::PointXYZ> >("/pcl_clustercloud", 1, MoIF::moment_of_inertia_features);
  // Spin
  ros::spin();
}
