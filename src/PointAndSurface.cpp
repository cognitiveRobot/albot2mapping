#include "PointAndSurface.h"
#include "GeometryFuncs.h"
using namespace std;

long SurfaceT::idCounter = 0;
const SurfaceT SurfaceT::INVALID = SurfaceT();
const PointXY PointXY::INVALID = PointXY();

bool isAround(double x, double y) {
    return fabs(x - y) < EPSILON;
}

// Pre-calculate length and tilt angle (these are rather expensive)

void SurfaceT::doPrecalc() {
    _precalcLength = p1.distFrom(p2);

    if (_precalcLength == 0) {
        // Avoid acos(inf)
        _precalcTiltAngle = 0;
    } else {
        // Tilt angle is the angle in respect to the x axis of a local coordinate space (used by angleDiffTo)
        PointXY dir = p2 - p1;
        double angle = acos(dir.getX() / _precalcLength);

        // Get the quadrants right
        if (dir.getY() < 0)
            angle = 2 * M_PI - angle;

        _precalcTiltAngle = rad2deg(angle);
    }
}

// SurfaceT length (euklidean distance)

double SurfaceT::getLength() const {
    return _precalcLength;
}

// Gets the angle between this surface and another (in degrees!).

double SurfaceT::getAngleDiffTo(const SurfaceT & other) const {
    // One surface has length 0. For simplicity we say the angle difference is 0.
    if (_precalcLength == 0 || other._precalcLength == 0)
        return 0;

    return other._precalcTiltAngle - _precalcTiltAngle;
}

// Returns a copy of the surface, with the endpoints swapped.

const SurfaceT SurfaceT::getSwappedCopy() const {
    SurfaceT swapped(p2, p1, id);
    swapped.p1Occluding = p2Occluding;
    swapped.p2Occluding = p1Occluding;
    return swapped;
}

// Gets a point that is in the middle of the surface.

const PointXY SurfaceT::getMiddle() const {
    double x = p1.getX() + (p2.getX() - p1.getX()) / 2.0;
    double y = p1.getY() + (p2.getY() - p1.getY()) / 2.0;
    return PointXY(x, y);
}

/* Returns true if this surface intersects the other, and the intersection point
 * is actually within the surface limits (and not on the infinite line).
 */
bool SurfaceT::intersects(const SurfaceT & other) const {
    PointXY pI = intersectLines(p1, p2, other.p1, other.p2);
    return pI.distFromSq(p1) <= _precalcLength * _precalcLength
            && pI.distFromSq(p2) <= _precalcLength * _precalcLength
            && pI.distFromSq(other.p1) <= other._precalcLength * other._precalcLength
            && pI.distFromSq(other.p2) <= other._precalcLength * other._precalcLength;

}

bool SurfaceT::intersectsStrictly(const SurfaceT & other) const {
    PointXY pI = intersectLines(p1, p2, other.p1, other.p2);
    return pI.distFromSq(p1) <= _precalcLength * _precalcLength
            && pI.distFromSq(p2) <= _precalcLength * _precalcLength
            && pI.distFromSq(other.p1) <= other._precalcLength * other._precalcLength
            && pI.distFromSq(other.p2) <= other._precalcLength * other._precalcLength
            && (pI != p1 && pI != p2 && pI != other.p1 && pI != other.p2);

}

/* Returns a perpendicular vector of the surface, with a length of 1 (a 2D point is the same as a vector) */
PointXY SurfaceT::getPerpendicularVector() {

    double y = (p2.getX() - p1.getX());
    double x = (-p2.getY() + p1.getY());
    double l = sqrt(x * x + y * y);


    PointXY nm(x / l, y / l);

    return nm;

}

void SurfaceT::printSurfaceT() {
    cout << "(" << p1.getX() << "," << p1.getY() << ")";
    cout << " - ";
    cout << "(" << p2.getX() << "," << p2.getY() << ")";
    cout << " [" << isAnObstacle << "]";
    cout << endl;
}

bool SurfaceT::contains(const PointXY & point, bool strict) const {
    PointXY v1 = point - p1;
    PointXY v2 = p2 - p1;

    double t1 = v1.getX() * v2.getY() - v1.getY() * v2.getX(); // Colinear if =0
    double lambda;

    if (isAround(t1, 0)) {
        // Colinearity OK, check if INSIDE
        if (isAround(v2.getX(), 0)) {
            lambda = v1.getY() / v2.getY();
        } else {
            lambda = v1.getX() / v2.getX();
        }

        if (strict) {
            return (0. < lambda && lambda < 1. && !isAround(lambda, 0) && !isAround(lambda, 1));
        } else {

            return (0. <= lambda && lambda <= 1.);
        }
    } else {
        return false;
    }


}

bool SurfaceT::isInside(const SurfaceT & other, bool strict) const {
    return (contains(other.p1, strict) || contains(other.p2, strict) || other.contains(p1, strict) || other.contains(p2, strict));
}

vector<SurfaceT> SurfaceT::getSurroundingObstacles(double distance) {
    /*
     * This function returns the rectangle surronding the surface at a distance of "distance"
     */
    PointXY p1 = getP1();
    PointXY p2 = getP2();

    PointXY nm = getPerpendicularVector();

    // Length of the surface
    double lg = sqrt((p1.getX() - p2.getX())*(p1.getX() - p2.getX()) + (p1.getY() - p2.getY())*(p1.getY() - p2.getY()));

    // Middle of the line
    PointXY G = (p1 + p2) / 2.0;

    // Points of the surrounding obstacle
    PointXY U = ((p1 - G)*(1.0 + distance / (lg / 2.0)) + nm * distance) + G;
    PointXY V = ((p1 - G)*(1.0 + distance / (lg / 2.0)) - nm * distance) + G;
    PointXY W = ((p2 - G)*(1.0 + distance / (lg / 2.0)) - nm * distance) + G;
    PointXY X = ((p2 - G)*(1.0 + distance / (lg / 2.0)) + nm * distance) + G;

    vector<SurfaceT> surroundingPolygon(4);

    SurfaceT s1(U, V, true);
    SurfaceT s2(V, W, true);
    SurfaceT s3(W, X, true);
    SurfaceT s4(X, U, true);

    surroundingPolygon[0] = s1;
    surroundingPolygon[1] = s2;
    surroundingPolygon[2] = s3;
    surroundingPolygon[3] = s4;

    return surroundingPolygon;

}
