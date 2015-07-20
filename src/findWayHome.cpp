/* 
 * File:   findWayHome.cpp
 * Author: Segolene Minjard
 * 
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

/* ------------------------- Namespaces ------------------------- */

using namespace std;

/* ------------------------- Program ------------------------- */

void print(std::map<int, int> map);

int main(int argc, char** argv) {

    GlobalMap globalMap;
    int mapId = 0;
    int numView = 1;
    char answer = 'y';

    while (answer == 'y') {
        Map curMap(1500, 1500);
        curMap.setMapID(mapId);
        int numSteps;
        cout << "How many steps? ";
        cin >> numSteps;

        if (globalMap.getMaps().size() > 0) {
            curMap.BuildMap(argv[1], numView, numSteps, &(globalMap.getMaps().back()));
        } else {
            curMap.BuildMap(argv[1], numView, numSteps);
        }

        numView = curMap.getMap().back().getId() + 1;
        globalMap.addMap(curMap);

        char mapName[50];
        vector<Surface> toPlot; //=curMap.getMap()[0].getSurfaces();
        for (unsigned int j = 0; j < curMap.getMap().size(); j++) {
            vector<Surface> tmp = curMap.getMap()[j].getSurfaces();
            toPlot.insert(toPlot.end(), tmp.begin(), tmp.end());
        }
        toPlot.push_back(curMap.getExit());
        toPlot.push_back(curMap.getEntrance());
        sprintf(mapName, "%s%d%s", "../outputs/Maps/Map-", mapId, "-withExitEntrance.png");
        plotSurfacesGNU(mapName, toPlot);

        cout << "Do you want to create another local space ? y/n ";
        cin >> answer;

        mapId++;
    }


    return 0;
}
