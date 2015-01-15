#ifndef SAME_SURFACE_FINDER_ODO_H
#define SAME_SURFACE_FINDER_ODO_H

#include "SameSurfaceFinder.h"

class SameSurfaceFinderOdo {

public:
    bool recognizeSameSurface(vector<Surface> & refSurfaces, std::vector<Surface> pvLandmarks,
        std::vector<Surface> cvLandmarks, AngleAndDistance lastLocomotion);
    
    
   
};

#endif /* SAME_SURFACE_FINDER_ODO_H */