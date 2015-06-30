/*
* File:   minhMapping.cpp
* Author: Minh, modified from onlineMapping
*
*
* Created on June 30 2015
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
#include <string>


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
#include <boost/filesystem/operations.hpp>
//#include <mrpt/bayes/CParticleFilterCapable.h>


/* -------------------------Segmentation includes ------------------------- */
#include "../include/segmentation/image.h"
#include "../include/segmentation/misc.h"
#include "../include/segmentation/pnmfile.h"
#include "../include/segmentation/segment-image.h"

/* ------------------------- Headers ------------------------ */
#include "Camera.h"
#include "Robot.h"
#include "Map.h"
#include "View.h"
#include "ImageProcessing.h"
#include "SameSurfaceFinderColor.h"
#include "SameSurfaceFinderOdo.h"
#include "Printer.h"
#include "PathFinder.h"

/* ------------------------- Namespaces ------------------------- */

using namespace std;

/* ------------------------- Program ------------------------- */

void print(std::map<int, int> map);

int main(int argc, char** argv) {
	/*------------------------------------------ Variables declaration ------------------------------------------ */
	Robot Albot;

	Camera Bumblebee;
	View curView;
	curView.setRobotSurfaces(Albot.getRectRobot());
	Map curMap(1500, 1500);

	ArSimpleConnector connector(&argc, argv);
	Albot.connect(argc, argv, &connector);
	Bumblebee.initialize();
	Bumblebee.setV(0);
	Albot.saveTravelInfo(0, 0, "../inputs/surfaces/coordTrans-0");

	/*------------------------------------------ Start Xploring ------------------------------------------ */

	//curView.printView();
	char viewName[50], mapName[50], pointFile[50];
	int localSpaceCounter = 0;

	/* -------- Loop ------- */
	char tkStep = 'y';
	while (true) 
  {
		
		Bumblebee.setV(0);		
		Bumblebee.getImage();		
    ArUtil::sleep(500);
	}
	cout << "Lost cases: " << curMap.getLostStepsNumber().size() << endl;
	for (unsigned int i = 0; i < curMap.getLostStepsNumber().size(); i++) {
		cout << curMap.getLostStepsNumber()[i] << " ";
	}
	cout << endl;

	return 0;
}


