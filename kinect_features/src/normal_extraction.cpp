#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/pfh.h>
#include <pcl/io/pcd_io.h>
#include <pcl_ros/point_cloud.h>

void pfh_cb(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& inputCluster) {
  
  //****************************************
  // Read in data
  //****************************************
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  // Read from file
  //pcl::PCDReader reader;
  //reader.read ("cloud_cluster_2.pcd", *cloud);
  //std::cout << "PointCloud input: " << cloud->points.size() << " data points." << std::endl; //*
  
  // Read from publisher
  pcl::copyPointCloud(*inputCluster, *cloud);
  
  //****************************************
  // Extract normals
  //****************************************
  
  // Create the normal estimation class, and pass the input dataset to it
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  ne.setInputCloud(cloud);
  
  // Create an empty kdtree representation, and pass it to the normal estimation object.
  // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ> ());
  ne.setSearchMethod(tree);
  
  // Output datasets
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
  
  // Use all neighbors in a sphere of radius 3cm
  ne.setRadiusSearch(0.03);
  
  // Compute the features
  ne.compute(*cloud_normals);
  
  for (int i = 0; i < cloud_normals->points.size(); i++) {
    if (!pcl::isFinite<pcl::Normal>(cloud_normals->points[i])) {
      PCL_WARN("normals[%d] is not finite\n", i);
    }
  }
  
  std::cout << "PointCloud normals: " << cloud_normals->points.size() << " data points." << std::endl; //*
  
  //****************************************
  // Extract PFH
  //****************************************
  
  // Create the PFH estimation class, and pass the input dataset+normals to it
  pcl::PFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::PFHSignature125> pfh;
  pfh.setInputCloud(cloud);
  pfh.setInputNormals(cloud_normals);
  // alternatively, if cloud is of tpe PointNormal, do pfh.setInputNormals (cloud);

  // Create an empty kdtree representation, and pass it to the PFH estimation object.
  // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
  pcl::search::KdTree<pcl::PointXYZ>::Ptr PFHtree(new pcl::search::KdTree<pcl::PointXYZ>());
  //pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr tree(new pcl::KdTreeFLANN<pcl::PointXYZ> ()); -- older call for PCL 1.5-
  pfh.setSearchMethod(PFHtree);

  // Output datasets
  pcl::PointCloud<pcl::PFHSignature125>::Ptr pfhs(new pcl::PointCloud<pcl::PFHSignature125>());
  
  // Use all neighbors in a sphere of radius 5cm
  // IMPORTANT: the radius used here has to be larger than the radius used to estimate the surface normals!!!
  pfh.setRadiusSearch(0.05);

  // Compute the features
  pfh.compute(*pfhs);
  
  std::cout << "PointCloud PFH: " << pfhs->points.size() << " data points." << std::endl; //*
  
  std::cout << "Histogram value at index..." << std::endl; 
  
  float finalHist[125] = {0};
  // print histogram
  for (size_t i = 0; i < cloud->points.size (); ++i) {
	//std::cout << i << " : ";
    for(int j = 0; j < 125; j++) {
      //std::cout << pfhs->points[i].histogram[j] << " ";
      finalHist[j] += pfhs->points[i].histogram[j];
    }
    //std::cout << sum << std::endl;
  }
  
  // Print the result
  std::cout << "Final Histogram: [";
  for(int j = 0; j < 124; j++) {
    std::cout << finalHist[j] << ", ";
  }
  std::cout << finalHist[124] << "]" << std::endl;
  
}

int main(int argc, char** argv) {
  // Initialize ROS
  ros::init(argc, argv, "pfh_from_pcl_cloud");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe<pcl::PointCloud<pcl::PointXYZ> >("/pcl_clustercloud", 1, pfh_cb);
  
  // Spin
  ros::spin();
}
