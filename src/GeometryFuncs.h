/*
 *  Helper functions for geometric calculations.
 *
 *  Thomas
 */

#ifndef _GEOMETRYFUNCS_H_
#define _GEOMETRYFUNCS_H_

#include <cmath>
#include <iostream>
#include <vector>
#include "PointAndSurface.h"
#include "Robot.h"


#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* Intersects the (infinite) lines g and h, described by the points G1, G2 and H1, H2.
 * If the lines are parallel, then the result will be (NaN,NaN), which is equal to PointXY::INVALID.
 */
PointXY intersectLines(const PointXY & g1, const PointXY & g2, const PointXY & h1, const PointXY & h2);


/* Gets the angle and distance between newSurf and referenceNew, and applies them to referenceOld.
 * The result is a surface in the old coordinate space.
 * Used for updating the MFIS with new surfaces.
 */
SurfaceT distanceAngleTransform(const SurfaceT & newSurf,
        const SurfaceT & referenceNew,
        const SurfaceT & referenceOld,
        SurfaceTMatch::MatchingSurfaceTEndpoints referencePoints);


/* Extends the surface (tobeextended) to have the length of (extensionReference).
 * Keeps the ID of tobeextended, but copies the occlusion from extensionReference.
 * If extendFromPoint1 is true, then P1 will be fixed and P2 be changed.
 */
const SurfaceT extendSurfaceT(const SurfaceT & toBeExtended,
        const SurfaceT & extensionReference,
        bool extendFromPoint1);

// Gets a point on this surface that is the nearest to the point specified.
PointXY nearestPointOnSurf(const SurfaceT & surf, const PointXY & other, bool limitToEndpoints);

// Returns the smallest distance between two surfaces (limited to their endpoints).
double distBetweenSurfs(const SurfaceT & s1, const SurfaceT & s2);

// Intersects a line with a circle. Returns a vector of points, containing all intersections (0, 1 or 2 points).
std::vector<PointXY> intersectLineWithCircle(const PointXY & p1,
        const PointXY & p2,
        const PointXY & cCenter,
        double r);

// Returns true if a point is "behind" a line. Behind is the side of the line which (movementDir) faces.
bool isBehindLine(const PointXY & pointToCheck, const SurfaceT & line, const PointXY & movementDir);

/* Determines if a point is inside a polygon. Uses ray casting algorithm.
 * NOTE: if the point is EXACTLY ON a vertex or edge, then result is true (inside).
 */
//bool pointInPolygon(const PointXY & point, const std::vector<SurfaceT> & polygon, bool strictly = false);

bool pointInPolygon(const double & pointX, const double & pointY, const vector<PointXY>& points);

/**
 * Returns true if the surface is intersecting the polygon
 * @param surface
 * @param polygon Ordered points of the polygon (clockwise or counterclockwise)
 */
bool surfaceIntersectsPolygon(const Surface& surface, const vector<PointXY>& polygon);

double deg2rad(double degAngle);

double rad2deg(double radAngle);

// Reduces an angle to the smallest difference bewtween two intersecting lines (always <=90 deg)
double normAngleDiff(double degAngle);

/**
 * Return the new position after moving the distance and angle of distanceAngle from lastPos
 */
cv::Point3f getNewPosFromDistanceAngle(cv::Point3f lastPos,const AngleAndDistance & distanceAngle);

/**
 * Applies PCA algorithm to find the main direction of points
 * @param points
 * @return a surface corresponding to the main direction computed
 */
Surface principalComponentAnalysis(vector<PointXY> points);

/**
 * Applies DBSCAN algorithm to cluster points by density
 * @param points
 * @param eps maximum distance between two points
 * @param minPts minimum of points in a cluster
 * @return a vector of clusters
 */
vector<vector<PointXY> > DBSCAN_points(vector<PointXY> *points, float eps, int minPts);

//For DBSCAN
vector<int> regionQuery(vector<PointXY> *points, PointXY *point, float eps);

/**
 * Returns true if p1 and p2 are on the same side of surf
 * Uses orientation of points (both clockwise or counterclockwise)
 */
bool pointsOnSameSideOfSurface (cv::Point2f p1, cv::Point2f p2, Surface surf);

/**
 * Find the tangents of a circle passing by a point
 * @param circleCentre
 * @param point
 * @param radius
 * @return vector containing zero, one or two tangents depending on the configuration
 */
vector<Surface> findTangents(cv::Point2f circleCentre, cv::Point2f point, double radius);

list<PointXY> findPathAStar(const PointXY& startPt, const PointXY& goalPt, vector<Surface> surfaces);

#endif

