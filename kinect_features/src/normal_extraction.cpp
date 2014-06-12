#include <iostream>
#include <fstream>
#include <cmath>
#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/pfh.h>
#include <pcl/io/pcd_io.h>
#include <pcl_ros/point_cloud.h>

#define PI 3.14159265
#define NUM_BINS 5

/* computeRelativeAngles function. This function will take in a cloud and
 * it's normals and return a vector of histograms for each point over the
 * angles between the normal of a point in the cloud and the normals of
 * it's neighbors.
 * 
 * params: inCloud - Input cloud of points.
 *         inNorms - Input normals of points in cloud.
 *         radius - The neighborhood of points to calculate normals for.
 *         size - The number of bins for each histogram.
 * return: vector of histograms for relative angles between each point and
 *         it's neighbors based on the radius.
 */
std::vector<std::vector<int> >*
computeRelativeAngles(pcl::PointCloud<pcl::PointXYZ>::Ptr inCloud, pcl::PointCloud<pcl::Normal>::Ptr inNorms, double_t radius, int size);

/* getAngle function. This function calculates the angle between two
 * vectors in 3 dimmensional space.
 * 
 * params: A - A vector.
 *         B - An other vector.
 * returns: The angle between A and B.
 */
double getAngle(pcl::Normal A, pcl::Normal B);

void pfh_cb(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& inputCluster) {
  
  //****************************************
  // Read in data
  //****************************************
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  // Read from file
  //pcl::PCDReader reader;
  //reader.read ("cloud_cluster_sphere.pcd", *cloud);
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
  ne.setRadiusSearch(0.02);
  
  // Compute the features
  ne.compute(*cloud_normals);
  
  for (int i = 0; i < cloud_normals->points.size(); i++) {
    if (!pcl::isFinite<pcl::Normal>(cloud_normals->points[i])) {
      PCL_WARN("normals[%d] is not finite\n", i);
    }
  }
  
  std::cout << "PointCloud normals: " << cloud_normals->points.size() << " data points." << std::endl;
  
  //****************************************
  // Extract Relative norm angles
  //****************************************
  std::vector<std::vector<int> > *hist = computeRelativeAngles(cloud, cloud_normals, 0.03, NUM_BINS);
  std::vector<double> collapsedHist(NUM_BINS, 0.0);
  // Calculate final histogram
  double totalVals = 0.0;
  for (size_t i = 0; i < hist->size(); i++) {
    for (size_t j = 0; j < NUM_BINS; j++) {
      collapsedHist[j] += (*hist)[i][j];
      totalVals += (*hist)[i][j];
    }
  }
  // Print final histogram
  std::cout << "final histogram [";
  for (size_t j = 0; j < NUM_BINS - 1; j++) {
    collapsedHist[j] = collapsedHist[j] / totalVals;
    std::cout << collapsedHist[j] << ", ";
  }
  std::cout << collapsedHist[NUM_BINS] << "]" << std::endl;
  delete hist;
  
  //****************************************
  // Extract PFH
  //****************************************
  /*
  // Create the PFH estimation class, and pass the input dataset+normals to it
  pcl::PFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::PFHSignature125> pfh;
  pfh.setInputCloud(cloud);
  pfh.setInputNormals(cloud_normals);
  // alternatively, if cloud is of type PointNormal, do pfh.setInputNormals (cloud);

  // Create an empty kdtree representation, and pass it to the PFH estimation object.
  // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
  pcl::search::KdTree<pcl::PointXYZ>::Ptr PFHtree(new pcl::search::KdTree<pcl::PointXYZ>());
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
  // calculate the final histogram
  for (size_t i = 0; i < cloud->points.size (); ++i) {
    for(int j = 0; j < 125; j++) {
      finalHist[j] += pfhs->points[i].histogram[j];
    }
  }
  
  // Print the result and save to file
  std::string pfhOutFile = "/home/skyler/catkin_groovy_ws/src/kinect_features/pfh.txt";
  std::ofstream pfhOutStream;
  pfhOutStream.open(pfhOutFile.c_str());
  if (pfhOutStream.is_open()) {
	pfhOutStream << "pfh [";
	std::cout << "Final Histogram: [";
    for(int j = 0; j < 124; j++) {
      pfhOutStream << finalHist[j] << ",";
      std::cout << finalHist[j] << ", ";
    }
    pfhOutStream << finalHist[124] << "]" << std::endl;
    std::cout << finalHist[124] << "]" << std::endl;
    pfhOutStream.close();
  } else {
    std::cout << "Unable to open file to write pfh" << std::endl;
    std::exit(EXIT_FAILURE);
  }
  */
}

std::vector<std::vector<int> > *computeRelativeAngles(pcl::PointCloud<pcl::PointXYZ>::Ptr inCloud, pcl::PointCloud<pcl::Normal>::Ptr inNorms, double_t radius, int size) {
  // Fill in KDTree with cloud points
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  kdtree.setInputCloud(inCloud);
  
  // Create histogram boundaries
  std::vector<double> histBounds(size, 0);
  double histSplit = PI / size;
  for (int i = 1; i < size; i++) {
    histBounds[i-1] = i * histSplit;
  }
  histBounds[size - 1] = 4.0;
  
  // Create empty vector of histograms
  std::vector<std::vector<int> > *finalHist = new std::vector<std::vector<int> >(inCloud->size(), std::vector<int>(size, 0));
  
  int numNeighbors = 0;
  double angle;
  std::vector<int> pointIdxRadiusSearch;
  std::vector<float> pointRadiusSquaredDistance;
  // search KDTree and build histograms for each point.
  for (size_t p = 0; p < inCloud->size(); p++) {
    kdtree.radiusSearch(inCloud->points[p], radius, pointIdxRadiusSearch, pointRadiusSquaredDistance);
    numNeighbors += pointIdxRadiusSearch.size() - 1;
    // get angle for each normal in radius
    for (size_t i = 1; i < pointIdxRadiusSearch.size(); ++i) {
      angle = getAngle(inNorms->points[pointIdxRadiusSearch[0]], inNorms->points[pointIdxRadiusSearch[i]]);
      // Put angle into histogram
      int b = 0;
      while (b < size - 1 and angle > histBounds[b]) {
        b++;
      }
      (*finalHist)[p][b] += 1;
    }
  }
  std::cout << "Total Neighbors: " << numNeighbors << std::endl;
  return finalHist;
}

double getAngle(pcl::Normal A, pcl::Normal B) {
  // cos^-1 (A.B / |A||B|)
  double AdotB = A.normal_x * B.normal_x + A.normal_y * B.normal_y + A.normal_z * B.normal_z;
  double squaredMagA = A.normal_x * A.normal_x + A.normal_y * A.normal_y + A.normal_z * A.normal_z;
  double squaredMagB = B.normal_x * B.normal_x + B.normal_y * B.normal_y + B.normal_z * B.normal_z;
  double angle = AdotB / (sqrt(squaredMagA) * sqrt(squaredMagB));
  if (angle > 1.0) {
    angle = 1.0;
  } else if (angle < -1.0) {
    angle = -1.0;
  }
  if (isnan(acos(angle))) {
    std::cout << "(" << A.normal_x << ", " << A.normal_y << ", " << A.normal_z << "); "
              << "(" << B.normal_x << ", " << B.normal_y << ", " << B.normal_z << ") " << std::endl
              << "    AdotB: " << AdotB << " squaredMagA: " << squaredMagA << " squaredMagB: " << squaredMagB 
              << " angle: " << angle << std::endl;
  }
  return acos(angle);
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
