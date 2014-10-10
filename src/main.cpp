/* 
 * File:   main.cpp
 * Author: Guillaume Diallo-Mulliez
 *
 * Created on June 7, 2013, 12:52 AM
 */

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

/* ------------------------- Headers ------------------------ */
#include "Camera.h"
#include "Robot.h"
#include "Map.h"
#include "View.h"
#include "ImageProcessing.h"
#include "SameObjectFinderColor.h"

/* ------------------------- Defines ------------------------- */
#define DISPARITY_HEIGHT 240.
#define DISPARITY_WIDTH 320.
#define RECTIFIED_HEIGHT 768.
#define RECTIFIED_WIDTH 1024.
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

/* ------------------------- Namespaces ------------------------- */

using namespace std;
//using namespace cv;

/* ------------------------- Program ------------------------- */

void print(std::map<int, int> map);

int main(int argc, char** argv) {

//    //pcl test
//	ImageProcessing imgPro;
////        imgPro.buildAPointCloud();
////        imgPro.visualizePointCloud();
//        imgPro.segRegionGrowing();
//        imgPro.segEuclideanClusters();
//        waitHere();

	/*------------------------------------------ Variables declaration ------------------------------------------ */

	Robot Albot;
	ArSimpleConnector connector(&argc, argv);

	Camera Bumblebee;
	View curView;
	Map curMap(1500, 1500);

	Albot.saveTravelInfo(0, 0, "../outputs/surfaces/coordTrans-0");

	SameObjectFinderColor sameObjectFinder;

	/*------------------------------------------ Construction & Initialization ------------------------------------------ */

	Albot.connect(argc, argv, &connector);

	Bumblebee.initialize();
	Bumblebee.setV(0);

	/*------------------------------------------ Start Xploring ------------------------------------------ */

	/* -------- Initialization : View 0 ------- */
	Bumblebee.getImage();               // Acquire image from Camera

	// View
	curView.setView(Bumblebee.getTriclops(), Bumblebee.getDepthImage(),
			Albot.getPos());        // Set view from camera photograph

	curView.display();                  // Display View in output file
	// Map
	curMap.update(curView); // Update the map according to the new view
	curMap.display();                   // Display Map in output file

	/* -------- Loop ------- */
	char tkStep;
	cout << endl << endl << "Take first step? (y/n) ";   // Ask user if continue
	cin >> tkStep;
	while (tkStep != 'n' && tkStep != 'N') {
		/* Increment counters */
		Bumblebee.incV();
		curView.setId(curView.getId() + 1);

		cout << endl << "=================================================="
				<< endl << endl;
		cout << "View no. " << Bumblebee.getV() << ":" << endl;

		/* Move Albot using user input */
		if (Bumblebee.getV() != 0) {
			Albot.move();
		}

		/* This part is unresolved : we must acquire 2 times the image or the camera gives the previous View instead of the new */
		Bumblebee.getImage();
		curView.setView(Bumblebee.getTriclops(), Bumblebee.getDepthImage(),
				Albot.getPos());

		Bumblebee.getImage();           // Acquire image from camera
		curView.setView(Bumblebee.getTriclops(), Bumblebee.getDepthImage(),
				Albot.getPos());            // Set view from camera photograph
		curView.display();              // Display View in output file

		std::map<int, int> sameSurfaceIds = sameObjectFinder.findSameSurfaces(
				curMap.getView().getSurfaces(), curView.getSurfaces());

		curMap.update(curView);     // Update the map according to the new view
		curMap.display();               // Display Map in output file

		cout << endl << endl << "Take another step? (y/n) "; // Ask user if continue
		cin >> tkStep;

	}
	cout << endl;

	return 0;
}
