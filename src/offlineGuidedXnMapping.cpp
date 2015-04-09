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
    Map localSpace(1500, 1500);
    localSpace.setRefForPreviousLS(Surface(0,0,0,300));
    Surface tempRefSurface;


    /*------------------------------------------ Start Xploring ------------------------------------------ */

    //curView.printView();
    char viewName[50], mapName[50], pointFile[50];

    bool initializeLocalSpace = true;
    int localSpaceCounter = 0;

    /* -------- Loop ------- */
    char tkStep = 'y';
    while (tkStep != 'n' && tkStep != 'N' && curView.getId() < 30) {
        /* Increment counters */

        //construct view from points.
        curView.setId(curView.getId() + 1);
        sprintf(pointFile, "%s%d", "../outputs/pointCloud/points2D-", curView.getId());
        curView.constructView(pointFile);
        cout << "View is formed :)" << endl;

        cout << endl << "==================================================" << endl << endl;
        cout << "View no. " << curView.getId() << ":" << endl;
        plotViewGNU("../outputs/Maps/currentView.png", curView);
        cout << endl << endl << "Initialize local space? (y/n) "; // Ask user if continue
        //cin >> tkStep;
                if(curView.getId() == 1 or curView.getId() == 9 or curView.getId() == 21)
                    tkStep = 'y';
                else
                    tkStep = 'n';


        if (tkStep == 'y' or tkStep == 'Y')
            initializeLocalSpace = true;

        if (initializeLocalSpace == true) {
            //save localSpace in a txt file before starting new.
            if (localSpaceCounter != 0) {
                if (REMEMBER_COMMON_VIEW == true) {
                    if (EGOCENTRIC_REFERENCE_FRAME == true)
                        localSpace.addPVUsingOdo(curView, Albot.getLocalSpaceHome());
                    else
                        localSpace.addCVUsingOdo(curView, Albot.getLocalSpaceHome());
                    sprintf(mapName, "%s%d%s%d%s", "../outputs/Maps/LS-", localSpaceCounter, "-v-", curView.getId(), "a-before.png");
                    plotMapGNU(mapName, localSpace);

                    localSpace.cleanMapUsingOdo(curView, Albot.getLocalSpaceHome());
                    sprintf(mapName, "%s%d%s%d%s", "../outputs/Maps/LS-", localSpaceCounter, "-v-", curView.getId(), "b-after.png");
                    plotMapGNU(mapName, localSpace);
                }

               // localSpace.findReferenceSurface(curView,tempRefSurface);
                plotMapGNU("../outputs/Maps/1.png", localSpace);
                localSpace.addCVUsingMultipleRef(curView);
                plotMapGNU("../outputs/Maps/2.png", localSpace);
                waitHere();
                sprintf(mapName, "%s%d", "../outputs/localSpaces/localSpace-", localSpaceCounter);
                localSpace.saveInTxtFile(mapName, localSpace.transformToGlobalMap(curView.getRobotSurfaces(), localSpace.getPathSegments()));
                localSpace.setRefForPreviousLS(tempRefSurface);
            }
            localSpaceCounter++;
            cout << BOLDMAGENTA << "Started local space >> " << BOLDRED << localSpaceCounter << RESET << endl;
            localSpace.initializeMap(curView);
            localSpace.setMapID(localSpaceCounter);


            Albot.setLocalSpaceHome(0, 0);
            sprintf(mapName, "%s%d%s%d%s", "../outputs/Maps/LS-", localSpaceCounter, "-v-", curView.getId(), ".png");
            plotMapGNU(mapName, localSpace);
            initializeLocalSpace = false;
        } else {//update local space
            
            if (EGOCENTRIC_REFERENCE_FRAME == true)
                localSpace.addPVUsingOdo(curView, Albot.getLocalSpaceHome());
            else
                localSpace.addCVUsingOdo(curView, Albot.getLocalSpaceHome());
            
            sprintf(mapName, "%s%d%s%d%s", "../outputs/Maps/LS-", localSpaceCounter, "-v-", curView.getId(), "a-before.png");
            plotMapGNU(mapName, localSpace, true);

            //            cout<<"surfaces after adding.."<<endl;
            //            for(unsigned int i=0; i<localSpace.getMap().size(); i++)
            //                cout<<i+1<<" surfs: "<<localSpace.getMap()[i].getSurfaces().size()<<endl;

            localSpace.cleanMapUsingOdo(curView, Albot.getLocalSpaceHome());
            sprintf(mapName, "%s%d%s%d%s", "../outputs/Maps/LS-", localSpaceCounter, "-v-", curView.getId(), "b-after.png");
            plotMapGNU(mapName, localSpace);

            //            cout<<"surfaces after cleaning.."<<endl;
            //            for(unsigned int i=0; i<localSpace.getMap().size(); i++)
            //                cout<<i+1<<" surfs: "<<localSpace.getMap()[i].getSurfaces().size()<<endl;
            cout << BOLDMAGENTA << "Expended local space >> " << BOLDRED << localSpaceCounter << RESET << endl;
        }
        plotMapGNU("../outputs/Maps/curLocalSpace.png", localSpace);

        
        cout << endl << endl << "Take another step? (y/n) "; // Ask user if continue
        //cin >> tkStep;
        tkStep = 'y';

        /* Move Albot using user input */
        if (tkStep != 'n' && tkStep != 'N') {
            //read odometer info
            sprintf(viewName, "%s%d", "../outputs/surfaces/coordTrans-", curView.getId());
            readOdometry(Albot, viewName);
            localSpace.addPathSegment(Albot.getLastLocomotion());
        }
    }
    if (localSpaceCounter != 0) {//for last local space
        sprintf(mapName, "%s%d", "../outputs/localSpaces/localSpace-", localSpaceCounter);
        localSpace.saveInTxtFile(mapName, localSpace.transformToGlobalMap(curView.getRobotSurfaces(), localSpace.getPathSegments()));
    }
    cout << endl;

    return 0;
}

