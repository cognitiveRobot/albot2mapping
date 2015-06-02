/* 
 * File:   GlobalMap.cpp
 * Author: Segolene Minjard
 * 
 * Created on 29 May 2015, 2:54 PM
 */

#include "GlobalMap.h"
#include "GeometryFuncs.h"

GlobalMap::GlobalMap() {
}


GlobalMap::~GlobalMap() {
}


 void GlobalMap::addMap(const Map map){
     maps.push_back(map);
 }
 
 vector<Map> GlobalMap::getMaps(){
     return maps;
 }
 
 bool localSpaceChanged(Map& map, const View& newView){
     list<Surface> boundaries = map.getBoundaries();
     cv::Point2f originRobotPos=map.getMap()[0].getRobotSurfaces()[0].getP1();
     cv::Point2f newRobotPos=newView.getRobotSurfaces()[0].getP1();
     
     vector<PointXY> points;
     
     if((map.getLeftBoundaryAngle() + (360-map.getRightBoundaryAngle()))<= 180){
         // Origin robot position out of polygon so we add it in
         points.push_back(PointXY(originRobotPos.x,originRobotPos.y));
     }
     
     for (list<Surface>::iterator i = boundaries.begin(); i != boundaries.end(); i++) {
        points.push_back(PointXY(i->getP1().x, i->getP1().y));
        points.push_back(PointXY(i->getP2().x, i->getP2().y));
    }
     return !pointInPolygon(newRobotPos.x, newRobotPos.y, points);
 }
