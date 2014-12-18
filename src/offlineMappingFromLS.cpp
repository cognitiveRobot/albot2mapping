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


/* -------------------------Segmentation includes ------------------------- */
//#include "../include/segmentation/image.h"
//#include "../include/segmentation/misc.h"
//#include "../include/segmentation/pnmfile.h"
//#include "../include/segmentation/segment-image.h"

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
    ArSimpleConnector connector(&argc, argv);

    Camera Bumblebee;
    View curView;
    Map curMap(1500, 1500);
    Map localSpace(1500, 1500);
    PathFinder pathFinder;
    AngleAndDistance nextGoal;

    Printer printer;

    Albot.saveTravelInfo(0, 0, "../outputs/surfaces/coordTrans-0");

    SameSurfaceFinderOdo sameSurfaceFinderOdo;

    /*------------------------------------------ Construction & Initialization ------------------------------------------ */

 //   Albot.connect(argc, argv, &connector);

   // Bumblebee.initialize();
    Bumblebee.setV(0);

    /*------------------------------------------ Start Xploring ------------------------------------------ */

    //curView.printView();
    char viewName[50], mapName[50];

    bool initializeLocalSpace = true;
    int localSpaceCounter = 0;

    
    
    /* -------- Loop ------- */
    char tkStep = 'y';

    while (tkStep != 'n' && tkStep != 'N') {
        /* Increment counters */
        //read view.
        curView.setId(curView.getId() + 1);
        sprintf(viewName, "%s%d", "../outputs/localSpaces/localSpace-", curView.getId());
        readALocalSpace(curView,viewName );
        curView.setRobotSurfaces(Albot.getRectRobot());

        curView.markLandmarks();
        
        sprintf(viewName, "%s%d%s", "../outputs/Views/View-", curView.getId(), ".png");
        plotViewGNU(viewName, curView);
        
        cout << endl << "==================================================" << endl << endl;
        cout << BOLDRED << "                View no. " << curView.getId() << ":" << RESET << endl;
        plotViewGNU("../outputs/Maps/currentView.png", curView);


        if (COMPUTE_GLOBAL_MAP == true) {
            
            if (curView.getId() == 1) {//initialize once.
                curMap.initializeMap(curView);
                sprintf(mapName, "%s%d%s", "../outputs/Maps/Map-v-", curView.getId(), ".png");
                plotMapGNU(mapName, curMap);
                cout << BOLDGREEN << "Map has been initialized...>> " << BOLDRED << curView.getId() << RESET << endl;
            } else {
                cout << BOLDCYAN << "Updating map >> @" << BOLDRED << curView.getId() << RESET << endl;
                //update map.
                curMap.expandMap(curView);

                if (PRINT_GLOBAL_MAP) {
                    sprintf(mapName, "%s%d%s", "../outputs/Maps/Map-v-", curView.getId(), "a-before.png");
                    plotMapGNU(mapName, curMap);
                }

//              //  curMap.cleanMapUsingOdo(curView, Albot.getFromHome());
//                if (PRINT_GLOBAL_MAP) {
//                    sprintf(mapName, "%s%d%s", "../outputs/Maps/Map-v-", curView.getId(), "b-after.png");
//                    plotMapGNU(mapName, curMap);
//                }
            }
        }

        cout << endl << endl << "Take another step? (y/n) "; // Ask user if continue
        cin >> tkStep;

        //save cView as pView for next step.
        curMap.setPreviousView(curView);
    }

    if (COMPUTE_GLOBAL_MAP == true)
        plotMapGNU("../outputs/Maps/Global-Map.png", curMap);
    cout << endl;

    return 0;
}

