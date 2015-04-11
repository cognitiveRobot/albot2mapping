#ifndef SAME_SURFACE_FINDER_ODO_H
#define SAME_SURFACE_FINDER_ODO_H

#include "SameSurfaceFinder.h"

class SameSurfaceFinderOdo {

public:
    bool matchSameSurfaces(vector<SameSurfaceInfo> & matchingInfoForAll, const vector<Surface> pvLandmarksOnCV, const vector<Surface> & cvLandmarks, 
            const double & angleTh, const double & distTh);
    bool recognizeSameSurface(vector<ReferenceSurfaces> & allRefSurfaces, std::vector<Surface> pvLandmarks,
        std::vector<Surface> cvLandmarks, AngleAndDistance lastLocomotion);
    bool recognizeAllSameSurface(vector<ReferenceSurfaces> & allRefSurfaces, const vector<View> views, std::vector<Surface> pvLandmarks,
        std::vector<Surface> cvLandmarks, AngleAndDistance lastLocomotion);
    
    
   
};

#endif /* SAME_SURFACE_FINDER_ODO_H */