#ifndef SAME_SURFACE_FINDER_ODO_H
#define SAME_SURFACE_FINDER_ODO_H

#include "SameSurfaceFinder.h"

class SameSurfaceFinderOdo {

public:
    SameSurfaceInfo recognizeSameSurface(std::vector<Surface> pvLandmarks,
            std::vector<Surface> cvLandmarks, AngleAndDistance lastLocomotion);
   
};

#endif /* SAME_SURFACE_FINDER_ODO_H */