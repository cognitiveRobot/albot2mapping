/* 
 * File:   main.cpp
 * Author: Md 
 * Author: Guillaume Diallo-Mulliez
 * Author 2 : Segolene Minjard
 * 
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
#include "GlobalMap.h"
#include "GeometryFuncs.h"

/* ------------------------- Namespaces ------------------------- */

using namespace std;

/* ------------------------- Program ------------------------- */

void print(std::map<int, int> map);

int main(int argc, char** argv) {
    /*------------------------------------------ Variables declaration ------------------------------------------ */
    Robot Albot;

    GlobalMap globalMap;

    Camera Bumblebee;
    View curView;
    curView.setRobotSurfaces(Albot.getRectRobot());
    Map *curMap = new Map(1500, 1500);


    /*------------------------------------------ Start Xploring ------------------------------------------ */

    //curView.printView();
    char viewName[50], mapName[50], pointFile[50];
    int localSpaceCounter = 0;

    /* -------- Loop ------- */
    char tkStep = 'y';
    int numOfStept;
    cout << "How many steps? ";
    cin >> numOfStept;

    while (curView.getId() < numOfStept && tkStep == 'y') {
        /* Increment counters */

        //construct view from points.
        curView.setId(curView.getId() + 1);
        sprintf(pointFile, "%s%s%s%d", "../inputs/", argv[1], "/pointCloud/points2D-", curView.getId());
        curView.constructView(pointFile);



        cout << "View is formed :)" << endl;
        cout << endl << "==================================================" << endl << endl;
        cout << "View no. " << curView.getId() << ":" << endl;
        sprintf(viewName, "%s%d", "../outputs/Maps/view-", curView.getId());
        plotViewGNU(viewName, curView);

        curView.setGap(FindGap(curView.getSurfaces(), curView.getRobotSurfaces()));
        
        if (curView.getId() == 1) {
            curMap->initializeMap(curView);
            /*  } else {
                  View viewOnMap=curMap->computeCVUsingMultipleRef(curView);
                  if(localSpaceChanged(*curMap, viewOnMap)){
                      cout<<"Create new local space"<<endl;
                      globalMap.addMap(*curMap);
                
                      int mapId=curMap->getMapID();
                      delete curMap;
                
                      curMap=new Map(1500, 1500);
                      curMap->setMapID(mapId+1);
                      curMap->initializeMap(curView);
                  }else{
                      curMap->addCv(viewOnMap);   
                  } 
              }*/
        } else if (curView.getId() % 2 == 0) {
            curMap->addCvUsingGap(curView);
        } else {
            globalMap.addMap(*curMap);

            int mapId = curMap->getMapID();
            delete curMap;

            curMap = new Map(1500, 1500);
            curMap->setMapID(mapId + 1);
            curMap->initializeMap(curView);
        }

        /*    Surface robotOrientation=curView.getRobotSurfaces()[0];
            vector<Surface> surfacesConsidered;
            for(unsigned int i=0; i<curView.getSurfaces().size(); i++){
                if(robotOrientation.distFromP1ToPoint(curView.getSurfaces()[i].midPoint().x,curView.getSurfaces()[i].midPoint().y)<6000){
                    surfacesConsidered.push_back(curView.getSurfaces()[i]);
                }
            }*/
        //  Surface gap=FindGap(curView.getSurfaces(), curView.getRobotSurfaces());



        //read odometer info
        sprintf(viewName, "%s%s%s%d", "../inputs/", argv[1], "/surfaces/coordTrans-", curView.getId());
        readOdometry(Albot, viewName);
        curMap->addPathSegment(Albot.getLastLocomotion());
        curMap->setLandmarkSurfaces(curView.getSurfaces());

        cout << endl << endl << "Take another step? (y/n) "; // Ask user if continue
        //cin >> tkStep;

    }

    globalMap.addMap(*curMap);
    delete curMap;

    /*    cout << "Lost cases: "<<curMap.getLostStepsNumber().size()<<endl;
        for(unsigned int i=0; i<curMap.getLostStepsNumber().size(); i++) {
            cout<<curMap.getLostStepsNumber()[i]<<" ";
        }*/
    cout << endl;
    cout << "Number of local spaces : " << globalMap.getMaps().size() << endl;

    return 0;
}

