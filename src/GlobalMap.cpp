/* 
 * File:   GlobalMap.cpp
 * Author: Segolene Minjard
 * 
 * Created on 29 May 2015, 2:54 PM
 */

#include "GlobalMap.h"
#include "GeometryFuncs.h"
#include "Printer.h"

GlobalMap::GlobalMap() {
}

GlobalMap::~GlobalMap() {
}

void GlobalMap::addMap(const Map map) {
    maps.push_back(map);
}

vector<Map> GlobalMap::getMaps() {
    return maps;
}

void GlobalMap::buildMaps(char* dataset) {
    Robot Albot;
    Map map(1500, 1500);
    
    View v=map.BuildMapWithBoundaries(dataset, 0, 0);
    Map lastMap=map; 
    
    bool keepGoing=true;
    while (keepGoing){
        map=Map(1500, 1500);
        try{
            v=map.BuildMapWithBoundaries(dataset, &v, &lastMap);
        }catch(bool e){
            keepGoing=false;
        }       
        lastMap=map;
    }
    

 /* View curView;
    curView.setRobotSurfaces(Albot.getRectRobot());

    char viewName[50], pointFile[50];

    bool sameLocalSpace = true;
    int numView = 1;
    int numMap = 0;
    bool firstViewInMap = true;

    map.setMapID(numMap);
      
    while (1) {

        //construct view from points.
        curView.setId(numView);
        sprintf(pointFile, "%s%s%s%d", "../inputs/", dataset, "/pointCloud/points2D-", curView.getId());

        struct stat buffer;
        if (stat(pointFile, &buffer) != 0) {
            //No more views
            return;
        }
        curView.constructView(pointFile);

        cout << "View is formed :)" << endl;
        cout << endl << "==================================================" << endl << endl;
        cout << "View no. " << curView.getId() << ":" << endl;
        sprintf(viewName, "%s%d", "../outputs/Maps/view-", curView.getId());
        plotViewGNU(viewName, curView);

        vector < pair<Surface, bool> > viewBoundaries;

        //Using odometer
        if (firstViewInMap) {
            viewBoundaries = curView.findBoundaries();
            curView.setViewBoundaries(viewBoundaries);

            map.initializeMap(curView);
            map.setMapBoundaries(viewBoundaries);
            firstViewInMap = false;

        } else {
            cv::Point2f firstRbtPos = map.getMap()[0].getRobotSurfaces()[0].getP1();
            cv::Point2f thisRbtPos = curView.getRobotSurfaces()[0].getP1();
            for (unsigned int i = 0; i < map.getMapBoundaries().size(); i++) {
                if (!pointsOnSameSideOfSurface(thisRbtPos, firstRbtPos, map.getMapBoundaries()[i].first)) {
                    sameLocalSpace = false;
                    break;
                }
            }
            if (!sameLocalSpace) {
                cout<<"here"<<endl;
                this->addMap(map);
                numMap++;
                map = Map(1500, 1500);
                map.setMapID(numMap);
                map.initializeMap(curView);
                map.computeEntrance(this->getMaps()[numMap - 1]);
                map.addSurfacesAfterEntrance(this->getMaps()[numMap - 1]);

                viewBoundaries = curView.findBoundaries();
                curView.setViewBoundaries(viewBoundaries);
                map.setMapBoundaries(viewBoundaries);
                sameLocalSpace=true;

            } else {
                viewBoundaries = curView.findBoundaries();
                curView.setViewBoundaries(viewBoundaries);
                //Add view to map
                View viewOnMap = map.computeCVUsingOdometer(curView);
                map.addCvAndClean(viewOnMap);
                cout<<"here2"<<endl;
            }
        }

        //Plot
        vector<Surface> surfaceBoundaries;
        for (unsigned int i = 0; i < viewBoundaries.size(); i++) {
            surfaceBoundaries.push_back(viewBoundaries[i].first);
        }

        char plotBoundaries[50];
        sprintf(plotBoundaries, "%s%d%s", "../outputs/Maps/boundaries-", curView.getId(), ".png");
        plotSurfacesGNU(plotBoundaries, surfaceBoundaries);

        //read odometer info
        sprintf(viewName, "%s%s%s%d", "../inputs/", dataset, "/surfaces/coordTrans-", curView.getId());
        readOdometry(Albot, viewName);
        map.addPathSegment(Albot.getLastLocomotion());
        map.setLandmarkSurfaces(curView.getSurfaces());

        numView++;
    }
    cout << endl;

    map.computeExit();*/
}

void GlobalMap::printMaps() {
    for (unsigned int i = 0; i < maps.size(); i++) {
        vector<Surface> toPlotSurfaces;
        vector<Surface> toPlotBoundaries;
        for (unsigned int j = 0; j < maps[i].getMap().size(); j++) {
            vector<Surface> tmp = maps[i].getMap()[j].getSurfaces();
            toPlotSurfaces.insert(toPlotSurfaces.end(), tmp.begin(), tmp.end());
            vector < pair<Surface, bool> > viewBoundaries = maps[i].getMap()[j].getViewBoundaries();
            for (unsigned int k = 0; k < viewBoundaries.size(); k++) {
                if(viewBoundaries[k].second){
                    toPlotBoundaries.push_back(viewBoundaries[k].first);
                }
            }
        }
        toPlotSurfaces.push_back(maps[i].getExit());
        toPlotSurfaces.push_back(maps[i].getEntrance());
        char mapName[50];
        sprintf(mapName, "%s%d%s", "../outputs/Maps/Map-", maps[i].getMapID(), "-withExitEntrance.png");
        plotSurfacesGNU(mapName, toPlotSurfaces);
        sprintf(mapName, "%s%d%s", "../outputs/Maps/Map-", maps[i].getMapID(), "-withBoundaries.png");
        plotSurfacesGNU(mapName, toPlotBoundaries);
    }
}

void GlobalMap::printWaysHome() {
    for (unsigned int i = 0; i < maps.size(); i++) {
        Surface position = maps[i].getExit();
        position.setP1(position.midPoint().x, position.midPoint().y);
        position.rotateAroundP1(-90);
        vector<AngleAndDistance> wayHome = maps[i].FindWayHome();
        vector<Surface> pathHome;
        pathHome.push_back(maps[i].getExit());
        pathHome.push_back(maps[i].getEntrance());

        cout << "WAY HOME : " << endl;
        for (unsigned int j = 0; j < wayHome.size(); j++) {
            cout << j << " - distance : " << wayHome[j].distance << " angle : " << wayHome[j].angle << endl;
            Surface newPosition = makeSurfaceWith(position, wayHome[j].angle, wayHome[j].distance, 400);
            pathHome.push_back(Surface(position.getP1().x, position.getP1().y, newPosition.getP1().x, newPosition.getP1().y));
            position = newPosition;
        }

        char mapName2[50];
        for (unsigned int j = 0; j < maps[i].getMap().size(); j++) {
            vector<Surface> tmp = maps[i].getMap()[j].getSurfaces();
            pathHome.insert(pathHome.end(), tmp.begin(), tmp.end());
        }
        sprintf(mapName2, "%s%d%s", "../outputs/Maps/Map-", maps[i].getMapID(), "-pathHome.png");
        plotSurfacesGNU(mapName2, pathHome);

        if (i != maps.size() - 1) {
            char ans = 'y';
            cout << "Do you want to continue ? y/n " << endl;
            cin>>ans;
            if (ans == 'n') {
                break;
            }
        }
    }
}

bool localSpaceChanged(Map& map, const View& newView) {
    Surface robotOrientation = map.getMap()[0].getRobotSurfaces()[0];
    list<Surface> boundaries = map.getBoundaries();
    cv::Point2f originRobotPos = map.getMap()[0].getRobotSurfaces()[0].getP1();
    cv::Point2f newRobotPos = newView.getRobotSurfaces()[0].getP1();

    char name[50];
    sprintf(name, "%s%d%s", "../outputs/Maps/N-boundaries-", newView.getId() - 1, ".png");
    plotSurfacesGNU(name, vector<Surface>(boundaries.begin(), boundaries.end()));

    if (PointHiddenBySurfaces(newRobotPos, vector<Surface>(boundaries.begin(), boundaries.end()), map.getMap()[0].getRobotSurfaces())) {
        return true;
    }


    updateBoundaries(map, newView);

    return false;
}

void updateBoundaries(Map& map, const View& view) {

    vector<Surface> viewSurfaces = view.getSurfaces(); //findTrustedSurfaces(view);
    if (viewSurfaces.size() == 0)return;

    // Angles are measured from the robot orientation of the first view of the map
    Surface robotOrientation = map.getMap()[0].getRobotSurfaces()[0];
    double firstSurfP1angle = robotOrientation.getAngleFromP1ToPoint(viewSurfaces[0].getP1().x, viewSurfaces[0].getP1().y);
    double firstSurfP2angle = robotOrientation.getAngleFromP1ToPoint(viewSurfaces[0].getP2().x, viewSurfaces[0].getP2().y);
    double lastSurfP1angle = robotOrientation.getAngleFromP1ToPoint(viewSurfaces[viewSurfaces.size() - 1].getP1().x, viewSurfaces[viewSurfaces.size() - 1].getP1().y);
    double lastSurfP2angle = robotOrientation.getAngleFromP1ToPoint(viewSurfaces[viewSurfaces.size() - 1].getP2().x, viewSurfaces[viewSurfaces.size() - 1].getP2().y);
    double viewHighestAngle = max(firstSurfP1angle, firstSurfP2angle);
    double viewLowestAngle = min(lastSurfP1angle, lastSurfP2angle);

    // Check if the angles are inverted or on different sides of robot orientation
    bool zeroInBetween = false;
    double diff = viewHighestAngle - viewLowestAngle;
    if (diff < 0) {
        if ((-diff) > 180) {
            zeroInBetween = true;
            // lowest angle has higher value but it's considered lower because it's the right side angle
        } else {
            swap(viewLowestAngle, viewHighestAngle);
        }
    }

    // Update map's anglesWithoutBoundaries and boundaries
    for (unsigned int i = 0; i < map.getAnglesWithoutBoundaries().size(); i++) {
        double firstAngle = map.getAnglesWithoutBoundaries()[i].first;
        double lastAngle = map.getAnglesWithoutBoundaries()[i].second;

        if (firstAngle == lastAngle) {
            //Empty interval
            continue;
        }

        if ((zeroInBetween && viewHighestAngle > firstAngle && viewLowestAngle < lastAngle)
                || (!zeroInBetween && viewHighestAngle <= lastAngle && viewLowestAngle >= firstAngle)) {

            //The view is entirely in an angle range without boundaries
            map.setAnglesWithoutBoundaries(i, pair<double, double>(firstAngle, viewLowestAngle));
            map.addAnglesWithoutBoundaries(pair<double, double>(viewHighestAngle, lastAngle));
            for (unsigned int j = 0; j < viewSurfaces.size(); j++) {
                map.addBoundary(viewSurfaces[j]);
            }
            continue;

        } else if (viewHighestAngle > firstAngle && (viewLowestAngle <= firstAngle || zeroInBetween)) {

            //High part in an angle range without boundaries
            map.setAnglesWithoutBoundaries(i, pair<double, double>(viewHighestAngle, lastAngle));
            double minAngle = firstAngle;
            double maxAngle = viewHighestAngle;
            findAndAddBoundaries(map, viewSurfaces, minAngle, maxAngle);

        } else if (viewLowestAngle < lastAngle && (viewHighestAngle >= lastAngle || zeroInBetween)) {

            //Low part in an angle range without boundaries
            map.setAnglesWithoutBoundaries(i, pair<double, double>(firstAngle, viewLowestAngle));
            double minAngle = viewLowestAngle;
            double maxAngle = lastAngle;
            findAndAddBoundaries(map, viewSurfaces, minAngle, maxAngle);
        }
    }
}

//Add the surfaces between minAngle and maxAngle (from robot origin orientation) to map's boundaries (without them covering same angles)

vector< pair<double, double> > findAndAddBoundaries(Map& map, const vector<Surface> & viewSurfaces, double minAngle, double maxAngle) {
    Surface robotOrientation = map.getMap()[0].getRobotSurfaces()[0];
    vector< pair<double, double> > freeSpaces;
    freeSpaces.push_back(make_pair(minAngle, maxAngle));

    for (unsigned int i = 0; i < viewSurfaces.size(); i++) {
        double angleP1 = robotOrientation.getAngleFromP1ToPoint(viewSurfaces[i].getP1().x, viewSurfaces[i].getP1().y);
        double angleP2 = robotOrientation.getAngleFromP1ToPoint(viewSurfaces[i].getP2().x, viewSurfaces[i].getP2().y);
        double lowAngle = min(angleP1, angleP2);
        double highAngle = max(angleP1, angleP2);
        if (highAngle - lowAngle > 180) {
            //zero in between
            swap(highAngle, lowAngle);
        }

        for (unsigned int j = 0; j < freeSpaces.size(); j++) {
            double tmpMinAngle = freeSpaces[j].first;
            double tmpMaxAngle = freeSpaces[j].second;

            if (tmpMinAngle < highAngle && highAngle < tmpMaxAngle
                    && tmpMinAngle < lowAngle && lowAngle < tmpMaxAngle) {
                // Surface fits entirely in free space
                freeSpaces[j] = make_pair(tmpMinAngle, lowAngle);
                freeSpaces.push_back(make_pair(highAngle, tmpMaxAngle));

                map.addBoundary(viewSurfaces[i]);
                break;
            } else if (tmpMinAngle < highAngle && highAngle < tmpMaxAngle) {
                // High angle part of surface in free space
                freeSpaces[j] = make_pair(highAngle, tmpMaxAngle);
                map.addBoundary(viewSurfaces[i]);
                break;
            } else if (tmpMinAngle < lowAngle && lowAngle < tmpMaxAngle) {
                // Low angle part of surface in free space
                freeSpaces[j] = make_pair(tmpMinAngle, lowAngle);
                map.addBoundary(viewSurfaces[i]);
                break;
            } else if (lowAngle < tmpMinAngle && tmpMaxAngle < highAngle) {
                // Surface larger than free space
                freeSpaces.erase(freeSpaces.begin() + j);
                map.addBoundary(viewSurfaces[i]);
                break;
            }
        }
    }
}

vector<Surface> findTrustedSurfaces(const View& view) {
    Surface robotPos = view.getRobotSurfaces()[0];
    vector<Surface> surfaces = view.getSurfaces();
    vector<Surface> trustedSurfaces;
    for (unsigned int i = 0; i < surfaces.size(); i++) {
        double distToP1 = robotPos.distFromP1ToPoint(surfaces[i].getP1().x, surfaces[i].getP1().y);
        double distToP2 = robotPos.distFromP1ToPoint(surfaces[i].getP2().x, surfaces[i].getP2().y);
        if (min(distToP1, distToP2) > MIN_DISTANCE_VISION && max(distToP1, distToP2) < MAX_DISTANCE_VISION) {
            trustedSurfaces.push_back(surfaces[i]);
        }
    }
    return trustedSurfaces;
}
