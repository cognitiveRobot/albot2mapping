
#include "Camera.h"

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

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include "Color.h"



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
  if ( pcl::io::loadPCDFile <pcl::PointXYZ> ("test_pcd.pcd", *cloud) == -1)
  {
    std::cout << "Cloud reading failed." << std::endl;
    //return (-1);
  }
    
    //visualize input cloud
    //visualizePointCloud(cloud);

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
  pcl::visualization::CloudViewer viewer ("Cluster viewer");
  viewer.showCloud(colored_cloud);
  while (!viewer.wasStopped ())
  {
  }
  
  //visualize segmented cloud
  //visualizePointCloud(colored_cloud);

}

void ImageProcessing::segEuclideanClusters() {
    // Read in the cloud data
    pcl::PCDReader reader;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
    reader.read ("test_pcd.pcd", *cloud);
    std::cout << "PointCloud before filtering has: " << cloud->points.size () << " data points." << std::endl; //*

    // Create the filtering object: downsample the dataset using a leaf size of 1cm
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    vg.setInputCloud (cloud);
    vg.setLeafSize (0.01f, 0.01f, 0.01f);
    vg.filter (*cloud_filtered);
    std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size ()  << " data points." << std::endl; //*

    // Create the segmentation object for the planar model and set all the parameters
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::PCDWriter writer;
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
        std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;

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

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr all_clusters (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointXYZRGB aPoint;
    int j = 0;
    Color myColor;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
        myColor = Color::random(); //one color for each cluster.
        //adding all points of one cluster
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++) {
            cloud_cluster->points.push_back(cloud_filtered->points[*pit]); //*
            aPoint.x = cloud_cluster->points.back().x;
            aPoint.y = cloud_cluster->points.back().y;
            aPoint.z = cloud_cluster->points.back().z;
            aPoint.r = myColor.red;
            aPoint.g = myColor.green;
            aPoint.b = myColor.blue;
            all_clusters->points.push_back(aPoint);

        }
        cloud_cluster->width = cloud_cluster->points.size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
        std::stringstream ss;
        ss << "cloud_cluster_" << j << ".pcd";

        j++;
    }

    //writer.write<pcl::PointXYZRGB> ("clustered_cloud.pcd", *cloud_cluster, false); //*

    pcl::visualization::CloudViewer viewer ("Cluster viewer");
    viewer.showCloud(all_clusters);
    while (!viewer.wasStopped ())
    {
    }
}

int ImageProcessing::getMaxMin(double value[], int valueSize, int maxmin)
{
    int pos = 0;
    int i = 0;
    double max1 =  - 1; //?-999999;
    double min1 = 999999;

    if (maxmin == 1)
    {
        //find max
        for (i = 0; i < valueSize; i++)
        {
            //find the index with the max ncc;
            if (value[i] > max1)
            {
                pos = i;
                max1 = value[i];
            }
        }
    }

    if (maxmin == 0)
    {
        //find min
        for (i = 0; i < valueSize; i++)
        {
            //find the index with the max ncc;
            if (value[i] < min1)
            {
                pos = i;
                min1 = value[i];
            }
        }
    }

    return pos;
}


////////////Stereo matching///////////////////
template <class T> class Image
{
private:
    IplImage *imgp;
public:
    Image(IplImage *img = 0)
    {
        imgp = img;
    }
    ~Image()
    {
        imgp = 0;
    }
    void operator = (IplImage *img)
    {
        imgp = img;
    }
    inline T *operator[](const int rowIndx)
    {
        return ((T*)(imgp->imageData + rowIndx * imgp->widthStep));
    }

};

typedef struct
{
    unsigned char b, g, r;
} RgbPixel;

typedef struct
{
    float b, g, r;
} RgbPixelFloat;

typedef Image < RgbPixel > RgbImage;
typedef Image < RgbPixelFloat > RgbImageFloat;
typedef Image < unsigned char > BwImage;
typedef Image < float > BwImageFloat;


void ImageProcessing::getDisparity() {

    int windowSize = 9;
    int DSR = 20;
//char *leftImgPath, *rightImgPath;
//cout<<"Enter full path of Left image ";
//cin>>leftImgPath;
//cout<<"Enter full path of Left image ";
//cin>>rightImgPath;
    IplImage *LeftinputImage= cvLoadImage("left-0.pgm", 0);
    IplImage *RightinputImage = cvLoadImage("right-0.pgm", 0);

//    int width = LeftinputImage->width;
//    int height = LeftinputImage->height;

    /****************8U to 32F**********************/
    IplImage *LeftinputImage32 = cvCreateImage(cvSize(LeftinputImage->width, LeftinputImage->height), 32, 1);
    //IPL_DEPTH_32F
    IplImage *RightinputImage32 = cvCreateImage(cvSize(LeftinputImage->width, LeftinputImage->height), 32, 1);
    cvConvertScale(LeftinputImage, LeftinputImage32, 1 / 255.);
    cvConvertScale(RightinputImage, RightinputImage32, 1 / 255.);

    int offset = floor((double)windowSize / 2);
    int height = LeftinputImage32->height;
    int width = LeftinputImage32->width;
    double *localNCC = new double[DSR];

    int x = 0, y = 0, d = 0, m = 0;
    int N = windowSize;

    IplImage *leftWinImg = cvCreateImage(cvSize(N, N), 32, 1);
    //mySubImage(LeftinputImage32,cvRect(0,0,N,N));
    IplImage *rightWinImg = cvCreateImage(cvSize(N, N), 32, 1);
    ; //mySubImage(RightinputImage32,cvRect(0,0,N,N));
    IplImage *disparity = cvCreateImage(cvSize(width, height), 8, 1);
    //or IPL_DEPTH_8U
    BwImage imgA(disparity);

    for (y = 0; y < height; y++)
    {
        for (x = 0; x < width; x++)
        {
            imgA[y][x] = 0;
        }
    }

    CvScalar s1;
    CvScalar s2;
    for (y = 0; y < height - N; y++)
    {
        //height-N
        for (x = 0; x < width - N; x++)
        {
            //width-N
            //getWindow(i,j,leftim,wl,N);
            cvSetImageROI(LeftinputImage32, cvRect(x, y, N, N));
            s1 = cvAvg(LeftinputImage32, NULL);
            cvSubS(LeftinputImage32, s1, leftWinImg, NULL); //zero-means
            cvNormalize(leftWinImg, leftWinImg, 1, 0, CV_L2, NULL);
            d = 0;

            //initialise localNCC
            for (m = 0; m < DSR; m++)
            {
                localNCC[m] = 0;
            }

            do
            {
                if (x - d >= 0)
                {

                    cvSetImageROI(RightinputImage32, cvRect(x - d, y, N, N));
                    s2 = cvAvg(RightinputImage32, NULL);
                    cvSubS(RightinputImage32, s2, rightWinImg, NULL); //zero-means
                    cvNormalize(rightWinImg, rightWinImg, 1, 0, CV_L2, NULL);
                }
                else
                {
                    break;
                }
                localNCC[d] = cvDotProduct(leftWinImg, rightWinImg);
                cvResetImageROI(RightinputImage32);
                d++;
            }
            while (d <= DSR);

            //to find the best d and store
            imgA[y + offset][x + offset] = getMaxMin(localNCC, DSR, 1) *16;
            cvResetImageROI(LeftinputImage32);
        } //x
        if (y % 10 == 0)
            cout << "row=" << y << " of " << height << endl;
    } //y

    cvReleaseImage(&leftWinImg);
    cvReleaseImage(&rightWinImg);

    cvShowImage( "Disparity", disparity);
    cv::waitKey(0);
    //return disparity;
}


void waitHere() {
        char temp;
        cin>>temp;
}
