#include <ImageProcessing.h>

#include <iostream>
#include <vector>

/* ------------------------- PCL includes ------------------------- */
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>



#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing.h>



/* Definitions for ImageProcessing class */


//cloud.width = 640; // Image-like organized structure, with 640 rows and 480 columns,
//cloud.height = 480; // thus 640*480=307200 points total in the dataset
void ImageProcessing::buildAPointCloud() {
    pcl::PointCloud<pcl::PointXYZ> cloud;

  // Fill in the cloud data
  cloud.width    = 5;
  cloud.height   = 1;
  cloud.is_dense = false;
 cloud.points.resize (cloud.width * cloud.height);

  for (size_t i = 0; i < 4; ++i)
  {
    cloud.points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud.points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud.points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
  }

  pcl::io::savePCDFileASCII ("test_pcd.pcd", cloud);
  std::cerr << "Saved " << cloud.points.size () << " data points to test_pcd.pcd." << std::endl;

  for (size_t i = 0; i < cloud.points.size (); ++i)
    std::cerr << "    " << cloud.points[i].x << " " << cloud.points[i].y << " " << cloud.points[i].z << std::endl;
}

void ImageProcessing::visualizePointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    
//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
//    pcl::io::loadPCDFile("test_pcd.pcd", *cloud);

    pcl::visualization::PCLVisualizer viewer("PCL Visualizer");

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_color_handler(cloud, 255, 255, 255);
    viewer.addPointCloud(cloud, cloud_color_handler, "cloud");

    viewer.setBackgroundColor(0.1, 0.1, 0.1);
    viewer.addCoordinateSystem();

    while (!viewer.wasStopped()) {
        viewer.spinOnce();
    }
}

void ImageProcessing::visualizePointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) {
    
//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
//    pcl::io::loadPCDFile("test_pcd.pcd", *cloud);

    pcl::visualization::PCLVisualizer viewer("PCL Visualizer");

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> cloud_color_handler(cloud, 255, 255, 255);
    viewer.addPointCloud(cloud, cloud_color_handler, "cloud");

    viewer.setBackgroundColor(0.1, 0.1, 0.1);
    viewer.addCoordinateSystem();

    while (!viewer.wasStopped()) {
        viewer.spinOnce();
    }
}



void ImageProcessing::segRegionGrowing() {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  if ( pcl::io::loadPCDFile <pcl::PointXYZ> ("region_growing_tutorial.pcd", *cloud) == -1)
  {
    std::cout << "Cloud reading failed." << std::endl;
    //return (-1);
  }
    
    //visualize input cloud
    visualizePointCloud(cloud);

  pcl::search::Search<pcl::PointXYZ>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZ> > (new pcl::search::KdTree<pcl::PointXYZ>);
  pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
  normal_estimator.setSearchMethod (tree);
  normal_estimator.setInputCloud (cloud);
  normal_estimator.setKSearch (50);
  normal_estimator.compute (*normals);

  pcl::IndicesPtr indices (new std::vector <int>);
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0.0, 1.0);
  pass.filter (*indices);

  pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
  reg.setMinClusterSize (50);
  reg.setMaxClusterSize (1000000);
  reg.setSearchMethod (tree);
  reg.setNumberOfNeighbours (30);
  reg.setInputCloud (cloud);
  //reg.setIndices (indices);
  reg.setInputNormals (normals);
  reg.setSmoothnessThreshold (3.0 / 180.0 * M_PI);
  reg.setCurvatureThreshold (1.0);

  std::vector <pcl::PointIndices> clusters;
  reg.extract (clusters);

  std::cout << "Number of clusters is equal to " << clusters.size () << std::endl;
  std::cout << "First cluster has " << clusters[0].indices.size () << " points." << endl;
  std::cout << "These are the indices of the points of the initial" <<
    std::endl << "cloud that belong to the first cluster:" << std::endl;
  int counter = 0;
//  while (counter < clusters[0].indices.size ())
//  {
//    std::cout << clusters[0].indices[counter] << ", ";
//    counter++;
//    if (counter % 10 == 0)
//      std::cout << std::endl;
//  }
  std::cout << std::endl;

  pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud ();
//  pcl::visualization::CloudViewer viewer ("Cluster viewer");
//  viewer.showCloud(colored_cloud);
//  while (!viewer.wasStopped ())
//  {
//  }
  
  //visualize segmented cloud
  visualizePointCloud(colored_cloud);

}

void waitHere() {
        char temp;
        cin>>temp;
}