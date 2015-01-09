/*
    Laser2Surface - Scans laser data and finds surfaces.

    Thomas
*/
#ifndef _LASER2SURFACE_H_
#define _LASER2SURFACE_H_

#include <vector>
#include <cstdlib>
//#include <cmath>


#include "Aria.h"
#include "ariaUtil.h"
#include "ArSensorReading.h"

#include "PointAndSurface.h"
#include "GeometryFuncs.h"


// Indices in the laser vector, describing surface start and end points. Used for splitting.
struct surfaceIndices
{
    int startIndex;
    int endIndex;
};

// Collects a laser reading from the robot, and corrects the XY-rotation
std::vector<PointXY> CollectLaser(ArSick &sick, double maxRange);

//vector<Object> scanAndSaveView(ArSick &sick, int v);

// Finds surfaces from raw laser data.
std::vector<SurfaceT> Laser2Surface(const std::vector<PointXY> & laserPoints,
								   double clusterTh,
								   double minSurfaceSize,
								   double errorTh);

/*
 * Splits a surface recursively until it can/should be split no more
 * After split, both resulting surfaces are still connected
 */
std::vector<surfaceIndices> splitSurface(const std::vector<PointXY> & laserdata,
										 int startIndex,
										 int endIndex,
										 double surfaceSize,
										 double errorThreshold);

/* Finds a splitting point for a surface.
 * First, the length is determined. If it is longer than the threshold, then find the point
 * farthest from the surface. It this distance exceeds the error threshold, then a split would occur
 * at that particular point.
 * Returns the index of the point where the surface should be split.
 */
int findSplittingPoint(const std::vector<PointXY> & laserPoints,
					   int currInd,
		   			   int nextInd,
					   double surfaceSizeTh,
					   double errorTh);

// Remove surfaces that are too small to matter
void removeBogusSurfaces(std::vector<SurfaceT> & surfaces);

// Marks occluding edges (we are certain that a surface ends here, and is not occluded)
void markOccludingEdges(std::vector<SurfaceT> & surfaces, double threshold);

/*
 * Marks boundary surfaces in a vector (single view).
 * The surfaces MUST be ordered by ID, and also geometrically (angle in respect to origin)!
 * => It works only with surfaces directly from the laser scan.
 */
void markBoundarySurfs(std::vector<SurfaceT> & surfaces, bool plotDebugImg);

#endif
