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

bool localSpaceChanged(Map& map, const View& newView);

#endif	/* GLOBALMAP_H */

