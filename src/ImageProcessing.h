/* 
 * File:   ImageProcessing.h
 * Author: guest
 *
 * Created on June 11, 2013, 9:58 PM
 */

#ifndef IMAGEPROCESSING_H
#define	IMAGEPROCESSING_H


/* ------------------------- Basic Includes ------------------------- */
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <assert.h>
#include <iostream>
#include <sstream>
#include <cmath>
#include <math.h>

/* ------------------------- Open CV includes ------------------------- */
#include <opencv/cv.h>
#include <opencv2/opencv.hpp>

/* ------------------------- PCL includes ------------------------- */
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>


/* ------------------------- Headers ------------------------ */




/* ------------------------- Namespaces ------------------------- */
using namespace std;
//using namespace cv;






/* View class contains :
 * Robot position and orientation
 * Vector of obstacles
*/

class ImageProcessing
{
    
private:
    
    
public:
    void buildAPointCloud();
        
    void visualizePointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    void visualizePointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);

    
    void segRegionGrowing();
   
    void segEuclideanClusters();
    
    int getMaxMin(double value[], int valueSize, int maxmin);
    void getDisparity();
    
};

void waitHere();

#endif	/* IMAGEPROCESSING_H */

