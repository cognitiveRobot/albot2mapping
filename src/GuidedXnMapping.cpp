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
#include "PointAndSurface.h"

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
    Map localSpace(1500, 1500);
    vector<PointXY> points2D;
    
    ImageProcessing imgTool;
    
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

    //curView.printView();
    char viewName[50], mapName[50], pointFile[50];

    bool initializeLocalSpace = true;
    int localSpaceCounter = 0;

    /* -------- Loop ------- */
    char tkStep = 'y';
    cout << endl << endl << "Do you want to create NEW surface folder? (y/n) "; // Ask user if continue
    cin >> tkStep;
    if (tkStep != 'n' && tkStep != 'N') {        
        sprintf(viewName, "%s%d", "../outputs/surfaces/surfaces-", readFolderNumber("../outputs/folderNumber"));        
        cout <<  viewName << endl;
        mkdir(viewName, 0700);
    }

    bool GLOBAL_MAP = false;
    cout << endl << endl << "Compute Global Map? (y/n) "; // Ask user if continue
    cin >> tkStep;
    if (tkStep != 'n' && tkStep != 'N')
        GLOBAL_MAP = true;

    tkStep = 'y';
    

    while (tkStep != 'n' && tkStep != 'N') {
        /* Increment counters */
        Bumblebee.incV();
        curView.setId(curView.getId() + 1);
        

        /* This part is unresolved : we must acquire 2 times the image or the camera gives the previous View instead of the new */
        Bumblebee.getImage();
        curView.setView(Bumblebee.getTriclops(), Bumblebee.getDepthImage(), Albot.getPos());

        Bumblebee.getImage(); // Acquire image from camera
        curView.setView(Bumblebee.getTriclops(), Bumblebee.getDepthImage(), Albot.getPos()); // Set view from camera photograph

        
        sprintf(pointFile, "%s%d", "../outputs/pointCloud/points2D-", curView.getId());
        curView.constructView(pointFile);
        cout<<"View is formed :)"<<endl;
        waitHere();

        //curView.markLandmarks();
        //curView.printView();
        sprintf(viewName, "%s%d%s", "../outputs/Views/View-", curView.getId(), ".png");
        plotViewGNU(viewName, curView);

        cout << endl << "==================================================" << endl << endl;
        cout << "View no. " << Bumblebee.getV() << ":" << endl;
        plotViewGNU("../outputs/Maps/currentView.png", curView);

        cout << endl << endl << "Initialize local space? (y/n) "; // Ask user if continue
        cin >> tkStep;

        if (tkStep == 'y' or tkStep == 'Y')
            initializeLocalSpace = true;

        if (initializeLocalSpace == true) {
            //save localSpace in a txt file before starting new.
            if (localSpaceCounter != 0) {
                localSpace.addCVUsingOdo(curView, Albot.getLocalSpaceHome());
            sprintf(mapName, "%s%d%s%d%s", "../outputs/Maps/LS-", localSpaceCounter, "-v-", curView.getId(), "a-before.png");
            plotMapGNU(mapName, localSpace);
            
            localSpace.cleanMapUsingOdo(curView, Albot.getLocalSpaceHome());
            sprintf(mapName, "%s%d%s%d%s", "../outputs/Maps/LS-", localSpaceCounter, "-v-", curView.getId(), "b-after.png");
            plotMapGNU(mapName, localSpace);
            
            
                sprintf(mapName, "%s%d", "../outputs/localSpaces/localSpace-", localSpaceCounter);
                localSpace.saveInTxtFile(mapName, localSpace.transformToGlobalMap(curView.getRobotSurfaces(), localSpace.getPathSegments()));
            }
            localSpaceCounter++;
            cout << BOLDMAGENTA << "Started local space >> " << BOLDRED << localSpaceCounter << RESET << endl;
            localSpace.initializeMap(curView);


            Albot.setLocalSpaceHome(0, 0);
            sprintf(mapName, "%s%d%s%d%s", "../outputs/Maps/LS-", localSpaceCounter, "-v-", curView.getId(), ".png");
            plotMapGNU(mapName, localSpace);
            initializeLocalSpace = false;
        } else {//update local space
            cout << BOLDMAGENTA << "Expended local space >> " << BOLDRED << localSpaceCounter << RESET << endl;

            localSpace.addCVUsingOdo(curView, Albot.getLocalSpaceHome());
            sprintf(mapName, "%s%d%s%d%s", "../outputs/Maps/LS-", localSpaceCounter, "-v-", curView.getId(), "a-before.png");
            plotMapGNU(mapName, localSpace);
            
//            cout<<"surfaces after adding.."<<endl;
//            for(unsigned int i=0; i<localSpace.getMap().size(); i++)
//                cout<<i+1<<" surfs: "<<localSpace.getMap()[i].getSurfaces().size()<<endl;

            localSpace.cleanMapUsingOdo(curView, Albot.getLocalSpaceHome());
            sprintf(mapName, "%s%d%s%d%s", "../outputs/Maps/LS-", localSpaceCounter, "-v-", curView.getId(), "b-after.png");
            plotMapGNU(mapName, localSpace);
            
//            cout<<"surfaces after cleaning.."<<endl;
//            for(unsigned int i=0; i<localSpace.getMap().size(); i++)
//                cout<<i+1<<" surfs: "<<localSpace.getMap()[i].getSurfaces().size()<<endl;
        }

        if (GLOBAL_MAP == true) {
            cout << BOLDCYAN << "Updating map >> " << BOLDRED << RESET << endl;
            if (curView.getId() == 1) {//initialize once.
                curMap.initializeMap(curView);
                sprintf(mapName, "%s%d%s", "../outputs/Maps/Map-v-", curView.getId(), ".png");
                plotMapGNU(mapName, curMap);
            } else {
                //update map.
                curMap.addCVUsingOdo(curView, Albot.getFromHome());

                if (PRINT_GLOBAL_MAP) {
                    sprintf(mapName, "%s%d%s", "../outputs/Maps/Map-v-", curView.getId(), "a-before.png");
                    plotMapGNU(mapName, curMap);
                }

                curMap.cleanMapUsingOdo(curView, Albot.getFromHome());
                if (PRINT_GLOBAL_MAP) {
                    sprintf(mapName, "%s%d%s", "../outputs/Maps/Map-v-", curView.getId(), "b-after.png");
                    plotMapGNU(mapName, curMap);
                }
            }
        }

        cout << endl << endl << "Take another step? (y/n) "; // Ask user if continue
        cin >> tkStep;

        /* Move Albot using user input */
        if (tkStep != 'n' && tkStep != 'N') {
            Albot.move();
            localSpace.addPathSegment(Albot.getLastLocomotion());
            curMap.addPathSegment(Albot.getLastLocomotion());
        }


    }
    if (localSpaceCounter != 0) {//for last local space
        sprintf(mapName, "%s%d", "../outputs/localSpaces/localSpace-", localSpaceCounter);
        localSpace.saveInTxtFile(mapName, localSpace.transformToGlobalMap(curView.getRobotSurfaces(), localSpace.getPathSegments()));
    }
    if (GLOBAL_MAP == true)
        plotMapGNU("../outputs/Maps/Global-Map.png", curMap);
    cout << endl;

    return 0;
}

