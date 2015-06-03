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

bool localSpaceChanged(Map& map, const View& newView) {
    Surface robotOrientation = map.getMap()[0].getRobotSurfaces()[0];
    list<Surface> boundaries = map.getBoundaries();
    cv::Point2f originRobotPos = map.getMap()[0].getRobotSurfaces()[0].getP1();
    cv::Point2f newRobotPos = newView.getRobotSurfaces()[0].getP1();

    char name[50];
    sprintf(name, "%s%d%s", "../outputs/Maps/N-boundaries-", newView.getId()-1, ".png");
    plotSurfacesGNU(name, vector<Surface>(boundaries.begin(), boundaries.end()));
    
    if (PointHiddenBySurfaces(newRobotPos, vector<Surface>(boundaries.begin(), boundaries.end()), map.getMap()[0].getRobotSurfaces())) {
        return true;
    }

    vector<Surface> viewSurfaces = newView.getSurfaces();
    if (viewSurfaces.size() != 0) {
        updateBoundaries(map, viewSurfaces);
    }

    return false;

    //PointInPolygon version
    /* vector<PointXY> points;  
       if ((map.getLeftBoundaryAngle() + (360 - map.getRightBoundaryAngle())) <= 180) {
           // Origin robot position out of polygon so we add it in
           points.push_back(PointXY(originRobotPos.x, originRobotPos.y));
       }
    
       points.push_back(PointXY(originRobotPos.x, originRobotPos.y));

       for (list<Surface>::iterator i = boundaries.begin(); i != boundaries.end(); i++) {
           points.push_back(PointXY(i->getP1().x, i->getP1().y));
           points.push_back(PointXY(i->getP2().x, i->getP2().y));
       }

       if (!pointInPolygon(newRobotPos.x, newRobotPos.y, points)) {
           return true;
       }*/
}


void updateBoundaries(Map& map, const vector<Surface> & viewSurfaces){

    // Angles or measured from the robot orientation of the first view of the map
    Surface robotOrientation = map.getMap()[0].getRobotSurfaces()[0];
    double firstSurfP1angle=robotOrientation.getAngleFromP1ToPoint(viewSurfaces[0].getP1().x, viewSurfaces[0].getP1().y);
    double firstSurfP2angle=robotOrientation.getAngleFromP1ToPoint(viewSurfaces[0].getP2().x, viewSurfaces[0].getP2().y);
    double lastSurfP1angle=robotOrientation.getAngleFromP1ToPoint(viewSurfaces[viewSurfaces.size() - 1].getP1().x, viewSurfaces[viewSurfaces.size() - 1].getP1().y);
    double lastSurfP2angle=robotOrientation.getAngleFromP1ToPoint(viewSurfaces[viewSurfaces.size() - 1].getP2().x, viewSurfaces[viewSurfaces.size() - 1].getP2().y);
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
            swap(viewLowestAngle,viewHighestAngle);
        }
    }

    // Update map's unknownAngles and boundaries
    for (unsigned int i = 0; i < map.getAnglesWithoutBoundaries().size(); i++) {
        double firstAngle = map.getAnglesWithoutBoundaries()[i].first;
        double lastAngle = map.getAnglesWithoutBoundaries()[i].second;

        if (firstAngle == lastAngle) {
            //Empty interval
            continue;
        }

        if ((zeroInBetween && viewHighestAngle > firstAngle && viewLowestAngle < lastAngle)
                ||( !zeroInBetween && viewHighestAngle <= lastAngle && viewLowestAngle >= firstAngle)) {

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
            double minAngle=firstAngle;
            double maxAngle=viewHighestAngle;
            findAndAddBoundaries(map, viewSurfaces, minAngle, maxAngle);

        } else if (viewLowestAngle < lastAngle && (viewHighestAngle >= lastAngle || zeroInBetween)) {
            
            //Low part in an angle range without boundaries
            map.setAnglesWithoutBoundaries(i, pair<double, double>(firstAngle, viewLowestAngle));
            double minAngle=viewLowestAngle;
            double maxAngle=lastAngle;
            findAndAddBoundaries(map, viewSurfaces, minAngle, maxAngle);
        }
    }
}

//Add the surfaces between minAngle and maxAngle (from robot origin orientation) to map's boundaries (without them covering same angles)
void findAndAddBoundaries(Map& map, const vector<Surface> & viewSurfaces, double minAngle, double maxAngle){
    
    Surface robotOrientation = map.getMap()[0].getRobotSurfaces()[0];
    vector< pair<double,double> > freeSpaces;
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

        for(unsigned int j=0; j<freeSpaces.size(); j++){
            double tmpMinAngle=freeSpaces[j].first;
            double tmpMaxAngle=freeSpaces[j].second;

            if(tmpMinAngle< highAngle && highAngle<tmpMaxAngle
                    && tmpMinAngle< lowAngle && lowAngle<tmpMaxAngle){
                // Surface fits entirely in free space
                freeSpaces[j]=make_pair(tmpMinAngle, lowAngle);
                freeSpaces.push_back(make_pair(highAngle, tmpMaxAngle));
                map.addBoundary(viewSurfaces[i]);
                break;
            }else if(tmpMinAngle< highAngle && highAngle<tmpMaxAngle){
                // High angle part of surface in free space
                freeSpaces[j]=make_pair(highAngle, tmpMaxAngle);
                map.addBoundary(viewSurfaces[i]);
                break;
            }else if(tmpMinAngle< lowAngle && lowAngle<tmpMaxAngle){
                // Low angle part of surface in free space
                freeSpaces[j]=make_pair(tmpMinAngle, lowAngle);
                map.addBoundary(viewSurfaces[i]);
                break;
            }else if(lowAngle<tmpMinAngle && tmpMaxAngle<highAngle){
                // Surface larger than free space
                freeSpaces.erase(freeSpaces.begin()+j);
                map.addBoundary(viewSurfaces[i]);
                break;
            }
        }
    }
}
