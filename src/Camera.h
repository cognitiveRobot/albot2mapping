/* 
 * File:   Camera.h
 * Author: Guillaume Diallo-Mulliez
 *
 * Created on May 28, 2013, 11:54 PM
 */

#ifndef CAMERA_H
#define	CAMERA_H


/* ------------------------- Basic Includes ------------------------- */
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <assert.h>
#include <iostream>
#include <sstream>
#include <cmath>
#include <math.h>


/* ------------------------- Robot includes ------------------------- */
#include "Aria.h"


/* ------------------------- Camera includes ------------------------- */
#include <dc1394/conversions.h>
#include <dc1394/control.h>
#include <dc1394/utils.h>

/* -------------------------PGR includes ------------------------- */
#include "../include/pgrlibdcstereo/pgr_registers.h"
#include "../include/pgrlibdcstereo/pgr_stereocam.h"
#include "../include/pgrlibdcstereo/pnmutils.h"
#include <libraw1394/raw1394.h>


/* ------------------------- Open CV includes ------------------------- */
#include <opencv/cv.h>
#include <opencv2/opencv.hpp>
//#include <mrpt/bayes/CParticleFilterCapable.h>

/* ------------------------- PCL includes ------------------------- */
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

/* ------------------------- Headers ------------------------ */
#include "Robot.h"

/* ------------------------- Defines ------------------------- */

#define _HANDLE_TRICLOPS_ERROR( function, error ) \
{ \
   if( error != TriclopsErrorOk ) \
   { \
      printf( \
	 "ERROR: %s reported %s.\n", \
	 function, \
	 triclopsErrorToString( error ) ); \
      exit( 1 ); \
   } \
} \

using namespace std;
//using namespace cv;   //bcz of error: reference to ‘flann’ is ambiguous

class Camera {
private:

    /* Camera variables for initialization */
    dc1394camera_t* camera;
    dc1394error_t err;
    dc1394_t * d;
    dc1394camera_list_t * list;

    unsigned int Id;
    PGRStereoCamera_t stereoCamera;

    /* Triclops variables */
    TriclopsError error;
    TriclopsContext triclops;
    TriclopsInput input;
    TriclopsInput colorInput;
    TriclopsImage16 depthImage;
    TriclopsImage rectifiedImage;
    TriclopsColorImage rectifiedColor;
    pcl::PointCloud<pcl::PointXYZ> pointCloud;


    // Buffers to hold the de-interleaved images
    unsigned char* pucDeInterlacedBuffer;
    unsigned char* pucRGBBuffer;
    unsigned char* pucGreenBuffer;
    unsigned char* pucRightRGB;
    unsigned char* pucLeftRGB;
    unsigned char* pucCenterRGB;


    /* Variables for output display */
    char filenameDepth[50], filenameRectified[50], filenameColor[50];


    /* Mapping variables */
    int v;




public:

    /* Constructor */
    Camera();

    /* Destructor */
    ~Camera();


    /* Mapping related functions */
    void setV(int newV);
    int getV();
    void incV();


    /* Camera related functions */

    void initialize(); //Initialize the camera

    void getImage(); // Acquire image from Camera

    void cleanup_and_exit(dc1394camera_t* camera); //Clean the camera and exit




    void convertColorTriclopsInput(TriclopsInput* colorInput, unsigned char* pucRGB);


    void deleteBuff();

    void savePointCloud(TriclopsContext triclops, TriclopsImage16 depthImage);

    /* Outputs */
    // Image buffers
    unsigned char* getDeInterlacedBuffer();
    unsigned char* getRGBBuffer();
    unsigned char* getGreenBuffer();
    unsigned char* getRightRGB();
    unsigned char* getLeftRGB();
    unsigned char* getCenterRGB();


    // Cameras
    PGRStereoCamera_t getStereoCam();

    // Triclops outputs
    TriclopsError getError();
    TriclopsContext getTriclops();
    TriclopsInput getInput();
    TriclopsInput getColorInput();
    TriclopsImage16 getDepthImage();
    TriclopsImage getRectifiedImage();
    TriclopsColorImage getRectifiedColor();


};

#endif	/* CAMERA_H */

