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
    ArSimpleConnector connector(&argc, argv);

    Camera Bumblebee;
    View curView;
    curView.setRobotSurfaces(Albot.getRectRobot());
    Map curMap(1500, 1500);
    PathFinder pathFinder;
    AngleAndDistance nextGoal;

    Printer printer;

    Albot.saveTravelInfo(0, 0, "../outputs/surfaces/coordTrans-0");

    SameSurfaceFinderOdo sameSurfaceFinderOdo;

    /*------------------------------------------ Construction & Initialization ------------------------------------------ */

    Albot.connect(argc, argv, &connector);

    Bumblebee.initialize();
    Bumblebee.setV(0);

    /*------------------------------------------ Start Xploring ------------------------------------------ */

    /* -------- Initialization : View 0 ------- */
    Bumblebee.getImage(); // Acquire image from Camera
    // View
    curView.setView(Bumblebee.getTriclops(), Bumblebee.getDepthImage(), Albot.getPos()); // Set view from camera photograph

    Bumblebee.getImage(); // Acquire image from Camera
    // View
    curView.setView(Bumblebee.getTriclops(), Bumblebee.getDepthImage(), Albot.getPos()); // Set view from camera photograph

    curView.markLandmarks();

    //curView.printView();
    char viewName[50];
    sprintf(viewName, "%s%d%s", "../outputs/Views/View-", curView.getId(), ".png");
    plotViewGNU(viewName, curView);

    bool ngFound = pathFinder.findNextGoal(curView, nextGoal);

    cout << ngFound << ": NextG - A & D: " << nextGoal.angle << " & " << nextGoal.distance << endl;

    curMap.initializeMap(curView);
    char mapName[50];
    sprintf(mapName, "%s%d%s", "../outputs/Maps/LS-", curView.getId(), ".png");
    // printer.printMap(mapName,curMap);
    plotMapGNU(mapName, curMap);
    bool initializeLocalSpace = false;

    // Map
    //    curMap.update(curView); // Update the map according to the new view
    //    curMap.display(); // Display Map in output file

    /* -------- Loop ------- */
    char tkStep;
    //   cout << endl << endl << "Shall I go? (y/n) "; // Ask user if continue
    //    cin >> tkStep;
    tkStep = 'y';

    while (ngFound && tkStep != 'n' && tkStep != 'N') {
        /* Increment counters */
        Bumblebee.incV();
        curView.setId(curView.getId() + 1);

        cout << endl << "=================================================="
                << endl << endl;
        cout << "View no. " << Bumblebee.getV() << ":" << endl;

        /* Move Albot using user input */
        if (Bumblebee.getV() != 0) {
            Albot.moveToTheGoal(nextGoal);
        }

        /* This part is unresolved : we must acquire 2 times the image or the camera gives the previous View instead of the new */
        Bumblebee.getImage();
        curView.setView(Bumblebee.getTriclops(), Bumblebee.getDepthImage(), Albot.getPos());

        Bumblebee.getImage(); // Acquire image from camera
        curView.setView(Bumblebee.getTriclops(), Bumblebee.getDepthImage(), Albot.getPos()); // Set view from camera photograph

        curView.markLandmarks();
        //curView.printView();
        sprintf(viewName, "%s%d%s", "../outputs/Views/View-", curView.getId(), ".png");
        plotViewGNU(viewName, curView);


        SameSurfaceInfo recognizedSurfaceInfo = sameSurfaceFinderOdo.recognizeSameSurface(
                curMap.getView().getLandmarks(), curView.getLandmarks(), Albot.getLastLocomotion());

        if(initializeLocalSpace == true) {
            cout<<BOLDMAGENTA<<"Started new local space"<<RESET<<endl;
            curMap.initializeMap(curView);
            sprintf(mapName, "%s%d%s", "../outputs/Maps/LS-", curView.getId(), "a-before.png");
            plotMapGNU(mapName, curMap);
            initializeLocalSpace = false;
        } else {
            curMap.addCVUsingOdo(curView, Albot.getFromHome());

            sprintf(mapName, "%s%d%s", "../outputs/Maps/LS-", curView.getId(), "a-before.png");
            plotMapGNU(mapName, curMap);

            curMap.cleanMapUsingOdo(curView, Albot.getFromHome());

            sprintf(mapName, "%s%d%s", "../outputs/Maps/LS-", curView.getId(), "b-after.png");
            plotMapGNU(mapName, curMap);
        }

        //        curMap.update(curView); // Update the map according to the new view
        //        curMap.display(); // Display Map in output file

        //find the next goal from cv.
        if (nextGoal.distance == 0) {
            cout << BOLDBLUE << "Goal is achieved. Going to find next goal." << RESET << endl;
            ngFound = pathFinder.findNextGoal(curView, nextGoal);
            initializeLocalSpace = true;
        }

        cout << "(Home) angle: " << Albot.getFromHome().angle << " dist: " << Albot.getFromHome().distance << endl;


        cout << endl << endl << "Take another step? (y/n) "; // Ask user if continue
        cin >> tkStep;


    }
    cout << endl;

    return 0;
}

