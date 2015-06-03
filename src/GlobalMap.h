/* 
 * File:   GlobalMap.h
 * Author: Segolene Minjard
 *
 * Created on 29 May 2015, 2:54 PM
 */

#ifndef GLOBALMAP_H
#define	GLOBALMAP_H

#include "Map.h"

using namespace std;

class GlobalMap {
public:
    GlobalMap();
    virtual ~GlobalMap();
    
    void addMap(const Map map);
    
    vector<Map> getMaps();
private:
    vector<Map> maps;
};

/**
 * Returns true if a new map (local space) must begin, false if the robot is 
 * still in the same local space
 * @param map
 * @param newView view to be added to a map

 */
bool localSpaceChanged(Map& map, const View& newView);

/**
 * Adds the surfaces that can be considered as boundaries to map's boundaries
 * @param map
 * @param viewSurfaces surfaces to check
 */
void updateBoundaries(Map& map, const vector<Surface> & viewSurfaces);

/**
 * Adds the surfaces between minAngle and maxAngle to map's boundaries 
 * (without them covering same angles)
 * @param map
 * @param viewSurfaces surfaces to check
 * @param minAngle angle from robot origin orientation from which boundaries
 *  can be added
 * @param maxAngle angle from robot origin orientation until which boundaries 
 * can be added
 */
void findAndAddBoundaries(Map& map, const vector<Surface> & viewSurfaces, double minAngle, double maxAngle );


#endif	/* GLOBALMAP_H */

