#include <ros/ros.h>
#include <float.h>
#include <math.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/visualization/cloud_viewer.h>

ros::Publisher pub;

int cluster_cb(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& inputCluster) {
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);

  // Read in the cloud data from file.
  //pcl::PCDReader reader;
  //reader.read ("/home/skyler/catkin_groovy_ws/min_cut_segmentation_tutorial.pcd", *cloud);

  // Read from publisher
  pcl::copyPointCloud(*inputCluster, *cloud);
  std::cout << "PointCloud before filtering has: " << cloud->points.size () << " data points." << std::endl; //*

  // Create the filtering object: downsample the dataset using a leaf size of 1cm
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  vg.setInputCloud (cloud);
  vg.setLeafSize (0.01f, 0.01f, 0.01f);
  vg.filter (*cloud_filtered);
  std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size ()  << " data points." << std::endl; //*
  
  // Get center of point cloud.
  float cloud_center[3] = {0.0, 0.0, 0.0};
  for (int i = 0; i < cloud_filtered->points.size(); i++) {
      cloud_center[0] = cloud_filtered->points[i].x;
      cloud_center[1] = cloud_filtered->points[i].y;
      cloud_center[2] = cloud_filtered->points[i].z;
  }
  // Calc centroid of pointcloud
  cloud_center[0] /= cloud_filtered->points.size();
  cloud_center[1] /= cloud_filtered->points.size();
  cloud_center[2] /= cloud_filtered->points.size();
  std::cout << "Center of Point Cloud: x=" << cloud_center[0] << " y=" << cloud_center[1] << " z=" << cloud_center[2] << std::endl;

  // Create the segmentation object for the planar model and set all the parameters
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (100);
  seg.setDistanceThreshold (0.02);

  int i=0, nr_points = (int) cloud_filtered->points.size ();
  while (cloud_filtered->points.size () > 0.3 * nr_points)
  {
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (cloud_filtered);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
      std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
      break;
    }

    // Extract the planar inliers from the input cloud
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud (cloud_filtered);
    extract.setIndices (inliers);
    extract.setNegative (false);

    // Get the points associated with the planar surface
    extract.filter (*cloud_plane);
    
    // Remove the planar inliers, extract the rest
    extract.setNegative (true);
    extract.filter (*cloud_f);
    *cloud_filtered = *cloud_f;
  }

  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (cloud_filtered);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance (0.02); // 2cm
  ec.setMinClusterSize (100);
  ec.setMaxClusterSize (25000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud_filtered);
  ec.extract (cluster_indices);

  float minDistance = FLT_MAX; // We want to find segment with smallest distance.
  pcl::PointCloud<pcl::PointXYZ>::Ptr min_cloud_cluster_ptr(new pcl::PointCloud<pcl::PointXYZ>);
  
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    float center[3] = {0.0, 0.0, 0.0};
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++) {
      cloud_cluster->points.push_back (cloud_filtered->points[*pit]); //*
      // Get center point. And then distance.
      center[0] += cloud_filtered->points[*pit].x;
      center[1] += cloud_filtered->points[*pit].y;
      center[2] += cloud_filtered->points[*pit].z;
    }
    
    cloud_cluster->width = cloud_cluster->points.size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;
    
    if(cloud_cluster->points.size() < 1000) { // Filter out large objects
      // Calc centroid
      center[0] /= cloud_cluster->points.size();
      center[1] /= cloud_cluster->points.size();
      center[2] /= cloud_cluster->points.size();
      // Get Distance from original cloud center.
      float distance = sqrt(pow(center[0] - cloud_center[0], 2) + pow(center[1] - cloud_center[1], 2) + pow(center[2] - cloud_center[2], 2));
      // Check min distance.
      if(distance < minDistance) {
        minDistance = distance;
        min_cloud_cluster_ptr = cloud_cluster;
      }
    }
  }
  
  if(minDistance < FLT_MAX) {
    std::cout << "Minimum distance cluster is " << minDistance << std::endl;
    std::cout << "PointCloud representing the Cluster: " << min_cloud_cluster_ptr->points.size() << " data points." << std::endl;

    // Calculate bounding box
    float min_x, min_y, min_z = FLT_MAX;
    float max_x, max_y, max_z = FLT_MIN;
    for (pcl::PointCloud<pcl::PointXYZ>::iterator iter = min_cloud_cluster_ptr->points.begin(); iter != min_cloud_cluster_ptr->points.end(); iter++) {
      float x = iter->x;
      if (min_x > x) {
        min_x = x;
      }
      if (max_x < x) {
        max_x = x;
      }
      float y = iter->y;
      if (min_y > y) {
        min_y = y;
      }
      if (max_y < y) {
        max_y = y;
      }
      float z = iter->z;
      if (min_z > z) {
        min_z = z;
      }
      if (max_z < z) {
        max_z = z;
      }
    }
    std::cout << "PointCloud x size: " << (max_x - min_x) << "." << std::endl;
    std::cout << "PointCloud y size: " << (max_y - min_y) << "." << std::endl;
    std::cout << "PointCloud z size: " << (max_z - min_z) << "." << std::endl;
    std::stringstream ss;
    pcl::PCDWriter writer;
    ss << "/home/skyler/catkin_groovy_ws/cloud_cluster_" << minDistance << ".pcd";
    writer.write<pcl::PointXYZ>(ss.str(), *min_cloud_cluster_ptr, false); //*
    // Publish the data
    pub.publish(*min_cloud_cluster_ptr);
  } else {
    std::cout << "No appropriate cluster found " << std::endl;
  }

  std::cout << "\n" << std::endl;
  pcl::visualization::CloudViewer viewer ("Segmented Object");
  viewer.showCloud(min_cloud_cluster_ptr);
  while (!viewer.wasStopped ()) {}
}

int main(int argc, char** argv) {
  // Initialize ROS
  ros::init(argc, argv, "clusters_from_pcl_cloud");
  ros::NodeHandle nh;
  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe<pcl::PointCloud<pcl::PointXYZ> >("/pcl_pointcloud", 1, cluster_cb);
  // Create a ROS publisher for the output point cloud
  pub = nh.advertise< pcl::PointCloud< pcl::PointXYZ > >("/pcl_clustercloud", 1);
  
  // Spin
  ros::spin();
  
}

