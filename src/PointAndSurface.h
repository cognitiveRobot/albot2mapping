/*
    2D point and surface classes.
    Created in order to stop the abuse of vector<double>!

    Thomas
 */

#include <cmath>
#include <vector>
#include <iostream>
#include <limits>

using namespace std;

#ifndef _POINTANDSURFACE_H_
#define _POINTANDSURFACE_H_

#define EPSILON 0.0001

/*
    PointXY describes a point in a 2D coordinate system.
    The x and y values cannot be changed after creation (create a new PointXY instead).
 */

bool isAround(double x, double y);

class PointXY {
private:
    double x, y;

public:

    static const PointXY INVALID;

    // Invalid point

    PointXY() :
    x(std::numeric_limits<double>::quiet_NaN()),
    y(std::numeric_limits<double>::quiet_NaN()) {
    }

    PointXY(double x, double y) :
    x(x), y(y) {
    }
    
//    PointXY(float x, float y) :
//    x(x), y(y) {
//    }
    void set(double a, double b) {
        x = a;
        y = b;
    }

    double getX() const {
        return x;
    }

    double getY() const {
        return y;
    }

    // Euklidean distance from another point

    double distFrom(const PointXY & other) const {
        return sqrt((x - other.x) * (x - other.x)
                + (y - other.y) * (y - other.y));
    }

    // Squared distance: for really fast calculation we omit the square root

    double distFromSq(const PointXY & other) const {
        return (x - other.x) * (x - other.x)
                + (y - other.y) * (y - other.y);
    }

    // Returns false if either x or y corresponds to NaN or inf


    // Two points are equal if their X and Y are equal. Invalid points are always equal.

    bool operator==(const PointXY & other) const {

        return (fabs(other.x - x) < EPSILON && fabs(other.y - y) < EPSILON) || (isnan<double>(x) && isnan<double>(other.x) && isnan<double>(y) && isnan<double>(other.y));
    }

    bool operator!=(const PointXY & other) const {
        return !(other == *this);
    }

    // Add this Point(vector) to another, and return a new Point with the result

    const PointXY operator+(const PointXY & other) const {
        return PointXY(x + other.x, y + other.y);
    }

    // Subtract another Point from this one, and return a new Point with the result

    const PointXY operator-(const PointXY & other) const {
        return PointXY(x - other.x, y - other.y);
    }

    const PointXY operator*(const double scalar) const {
        return PointXY(scalar * x, scalar * y);
    }

    const PointXY operator/(const double scalar) const {
        return PointXY(x / scalar, y / scalar);
    }

    // Scalar product beetween two vectors (a PointXY can be considered as a 2D-vector)

    const float operator*(const PointXY & other) const {
        return (x * other.x + y * other.y);
    }

    // Print the coordinates to a stream

    friend std::ostream & operator<<(std::ostream & os, const PointXY & p) {
        return os << '(' << p.x << ' ' << p.y << ')';
    }
};

/*
    SurfaceT describes a 2D line between two points.
    Contains information on whether its points are occluding edges or not.
    SurfaceT endpoints are read-only, but occlusion information can be changed.
 */


class SurfaceT {
private:
    /*  Not using pointers: don't have to worry about deleting points. Points only take 16 bytes, so that is tolerable.
        Also don't need to worry about dynamic allocation. */
    PointXY p1;
    PointXY p2;
    bool p1Occluding; // true if this point marks an occluding edge (reliable point)
    bool p2Occluding;



    bool boundarySurf; // true if this surface is a boundary of the current local space

    bool isAnObstacle; // whether the surface is an obstalce surface

    // The ID is used to identify the same surface in different coordinate systems
    long id;
    static long idCounter;

    // Pre-calculated values for length and tilt angle (these are rather expensive)
    double _precalcLength;
    double _precalcTiltAngle;

    void doPrecalc();

    friend class MPolygon;


public:
    // An invalid surface, with an ID of -1
    static const SurfaceT INVALID;

    // Construct an empty, invalid surface

    SurfaceT() :
    p1(0, 0), p2(0, 0), p1Occluding(false), p2Occluding(false), boundarySurf(false), id(-1),
    _precalcLength(0), _precalcTiltAngle(0) {
    }

    // If not explicitly specified, every surface gets a new ID

    SurfaceT(const PointXY & p1, const PointXY & p2, bool isAnObstacle = false) :
    p1(p1), p2(p2), isAnObstacle(isAnObstacle), p1Occluding(false), p2Occluding(false), boundarySurf(false), id(++idCounter) {
        doPrecalc();
    }

    // Explicitly set the ID

    SurfaceT(const PointXY & p1, const PointXY & p2, long id) :
    p1(p1), p2(p2), p1Occluding(false), p2Occluding(false), boundarySurf(false), id(id) {
        doPrecalc();
    }

    // Copy this surface from another, but change the ID

    SurfaceT(const SurfaceT & other, long newId) :
    p1(other.p1), p2(other.p2),
    p1Occluding(other.p1Occluding), p2Occluding(other.p2Occluding),
    boundarySurf(other.boundarySurf), id(newId),
    _precalcLength(other._precalcLength), _precalcTiltAngle(other._precalcTiltAngle) {
    }

    SurfaceT switchPoints() {
        SurfaceT result = *this;
        PointXY ptmp;
        ptmp = result.p1;
        result.p1 = result.p2;
        result.p2 = ptmp;

        bool btmp;
        btmp = result.p1Occluding;
        result.p1Occluding = result.p2Occluding;
        result.p2Occluding = btmp;

        result._precalcTiltAngle += 180.;

        return result;
    }

    const PointXY & getP1() const {
        return p1;
    }

    const PointXY & getP2() const {
        return p2;
    }

    double getX1() const {
        return p1.getX();
    } // For convenience

    double getY1() const {
        return p1.getY();
    }

    double getX2() const {
        return p2.getX();
    }

    double getY2() const {
        return p2.getY();
    }

    bool isP1Occluding() const {
        return p1Occluding;
    }

    bool IsAnObstacle() const {
        return isAnObstacle;
    }

    bool isP2Occluding() const {
        return p2Occluding;
    }

    void setP1Occluding(bool isOccluding) {
        p1Occluding = isOccluding;
    }

    void setP2Occluding(bool isOccluding) {
        p2Occluding = isOccluding;
    }

    bool isBoundarySurf() const {
        return boundarySurf;
    }

    void setBoundarySurf(bool isBoundary) {
        boundarySurf = isBoundary;
    }

    long getId() const {
        return id;
    }

    // Returns false if the ID is <0

    bool isValid() const {
        return id >= 0;
    }

    bool isInside(const SurfaceT & other, bool strict = false) const;

    bool contains(const PointXY & point, bool strict = false) const;


    // SurfaceT length (euklidean distance)
    double getLength() const;

    // Gets a point that is in the middle of the surface.
    const PointXY getMiddle() const;

    /* Gets the angle between this surface and another (in degrees!).
     * The angle is between -360 and 360 deg (thus NOT the smallest possible angle: use normAngleDiff if required)
     */
    double getAngleDiffTo(const SurfaceT & other) const;

    /* Returns true if this surface intersects the other, and the intersection point
     * is actually within the surface limits (and not on the infinite line).
     */
    bool intersects(const SurfaceT & other) const;
    bool intersectsStrictly(const SurfaceT & other) const;
    
    vector<SurfaceT> getSurroundingObstacles(double distance);
    


    // Returns a copy of the surface, with the endpoints swapped.
    const SurfaceT getSwappedCopy() const;

    // Print the points to a stream

    friend std::ostream & operator<<(std::ostream & os, const SurfaceT & s) {
        return os << '(' << s.p1 << ' ' << s.p2 << ')';
    }

    bool operator==(const SurfaceT & other) const {

        return ((p1 == other.p1 && p2 == other.p2) || (p1 == other.p2 && p2 == other.p1));
    }

    bool operator!=(const SurfaceT & other) const {
        return !(other == *this);
    }

    void printSurfaceT();



    // Compute the surface normale (returns a PointXY since a PointXY can be also considered as a 2D vector)
    PointXY getPerpendicularVector();
};



// Describes the type and quality of a match between two surfaces.

struct SurfaceTMatch {

    enum SurfaceTMatchType {
        NOT_MATCHING, // The surfaces don't match in any way
        IDENTICAL, // The surfaces match completely
        OLD_CONTAINED_IN_NEW, // The old surface matches a part of the new one (thus is contained)
        NEW_CONTAINED_IN_OLD, // The new surface matches a part of the old one
        PARTIALLY_MATCHING // Both surfaces match a part of the other
    } matchType;

    enum MatchingSurfaceTEndpoints {
        NO_POINTS, // No points match
        OLD1_NEW1_AND_OLD2_NEW2, // old.P1 matches new.P1, and old.P2 matches new.P2
        OLD1_NEW2_AND_OLD2_NEW1, // same, but with surfaces facing in opposite directions
        OLD1_NEW1_ONLY, // Only one point matches
        OLD2_NEW2_ONLY,
        OLD1_NEW2_ONLY,
        OLD2_NEW1_ONLY
    } matchingPoints;

    double matchFitness; // Number describing the quality of the match (higher=better, 0=not matching)
};

#endif

