#ifndef SAME_SURFACE_FINDER_H
#define SAME_SURFACE_FINDER_H

#include "View.h"


#include <iostream>
#include <vector>
#include <utility> 	// pair



struct SameSurfaceInfo {
    int mapSurfaceID; //surface id in the map vector.
    int cvSurfaceID;
    double matchingWeight; //it represents the similarity measure.
};

class SameSurfaceFinder {
public:
    /**
     * Note: every key and value in this map are unique ids.
     * @return a map where the key is the id of a surface in the surfaces1 vector and the value is the id of a surface in the surfaces2 vector
     */
    virtual std::map<int, int> findSameSurfaces(std::vector<Surface> surfaces1,
            std::vector<Surface> surfaces2) = 0;
    
    
    
    
};

#endif /* SAME_SURFACE_FINDER_H */
