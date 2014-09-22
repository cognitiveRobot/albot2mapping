#include <ImageProcessing.h>


/* ------------------------- PCL includes ------------------------- */
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

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

void ImageProcessing::visualizePointCloud() {
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile("out.pcd", *cloud);

    pcl::visualization::PCLVisualizer viewer("PCL Visualizer");

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_color_handler(cloud, 255, 255, 255);
    viewer.addPointCloud(cloud, cloud_color_handler, "cloud");

    viewer.setBackgroundColor(0.1, 0.1, 0.1);
    viewer.addCoordinateSystem();

    while (!viewer.wasStopped()) {
        viewer.spinOnce();
    }
}

void waitHere() {
        char temp;
        cin>>temp;
}