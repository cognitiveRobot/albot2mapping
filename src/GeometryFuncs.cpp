/*
 *  Helper functions for geometric calculations.
 *
 *  Thomas
 */

#include "GeometryFuncs.h"
#include "ImageProcessing.h"
#include <cstdlib>

using namespace std;

/* Intersects the lines g and h, described by the points G1, G2 and H1, H2.
 * If the lines are parallel, then the coordinates will be NaN or inf, which mark an invalid point.
 */
PointXY intersectLines(const PointXY & g1, const PointXY & g2, const PointXY & h1, const PointXY & h2) {
    /**
     *  Vector intersection
     *  g := gPos  +  uA * gDir
     *  h := hPos  +  uB * hDir
     *  => Set g=h and calculate uA
     */
    PointXY gDir = g2 - g1;
    PointXY hDir = h2 - h1;

    // Error: one of the lines' endpoints match (direction vector has zero length)
    if ((gDir.getX() == 0 && gDir.getY() == 0)
            || (hDir.getX() == 0 && hDir.getY() == 0)) {
        cerr << "WARNING: Cannot intersect lines: line endpoints match (length=0)!" << endl;

        // Lines don't intersect
        return PointXY::INVALID;
    }

    double x1 = g1.getX(), y1 = g1.getY();
    double x2 = g2.getX(), y2 = g2.getY();
    double x3 = h1.getX(), y3 = h1.getY();
    double x4 = h2.getX(), y4 = h2.getY();

    double uA = ((x4 - x3)*(y1 - y3) - (y4 - y3)*(x1 - x3))
            / ((y4 - y3)*(x2 - x1) - (x4 - x3)*(y2 - y1));

    return PointXY(x1 + uA * (x2 - x1),
            y1 + uA * (y2 - y1));
}

/* Gets the angle and distance between newSurf and referenceNew, and applies them to referenceOld.
 * The result is a surface in the old coordinate space.
 * Used for updating the MFIS with new surfaces.
 */
SurfaceT distanceAngleTransform(const SurfaceT & newSurf,
        const SurfaceT & referenceNew,
        const SurfaceT & referenceOld,
        SurfaceTMatch::MatchingSurfaceTEndpoints referencePoints) {
    // Decide which point to use as reference, and whether the surfaces are facing in the same direction
    PointXY refPointNew;
    PointXY refPointOld;
    bool facingOppositeDirection = false;
    switch (referencePoints) {
        case SurfaceTMatch::OLD1_NEW1_AND_OLD2_NEW2:
        case SurfaceTMatch::OLD1_NEW1_ONLY:
            refPointNew = referenceNew.getP1();
            refPointOld = referenceOld.getP1();
            break;

        case SurfaceTMatch::OLD2_NEW2_ONLY:
            refPointNew = referenceNew.getP2();
            refPointOld = referenceOld.getP2();
            break;

        case SurfaceTMatch::OLD1_NEW2_AND_OLD2_NEW1:
        case SurfaceTMatch::OLD1_NEW2_ONLY:
            refPointNew = referenceNew.getP2();
            refPointOld = referenceOld.getP1();
            facingOppositeDirection = true;
            break;

        case SurfaceTMatch::OLD2_NEW1_ONLY:
            refPointNew = referenceNew.getP1();
            refPointOld = referenceOld.getP2();
            facingOppositeDirection = true;
            break;

        default:
            cerr << "WARNING: Reference surface endpoints don't match!";
            return SurfaceT::INVALID;
    }

    /* For each new surface, get the distance and angle relative to the reference surface (for both points).
     * The distance is relative to P1 of the reference,
     * the angle is relative to the direction (p1->p2) of the reference.
     */
    double distanceP1 = newSurf.getP1().distFrom(refPointNew);
    double distanceP2 = newSurf.getP2().distFrom(refPointNew);

    double angleP1 = referenceNew.getAngleDiffTo(SurfaceT(refPointNew, newSurf.getP1()));
    double angleP2 = referenceNew.getAngleDiffTo(SurfaceT(refPointNew, newSurf.getP2()));

    // Construct new points, using angle and distance on the reference in the MFIS

    // Get the (normalized) direction of the MFIS reference surface
    double refDirLength = referenceOld.getLength();
    double refDirX = (referenceOld.getX2() - referenceOld.getX1()) / refDirLength;
    double refDirY = (referenceOld.getY2() - referenceOld.getY1()) / refDirLength;

    // If the reference surfaces are facing opposite each other, invert the direction
    if (facingOppositeDirection) {
        refDirX *= -1;
        refDirY *= -1;
    }

    // Rotate the direction by the angle
    double dirXP1 = cos(deg2rad(angleP1)) * refDirX - sin(deg2rad(angleP1)) * refDirY;
    double dirYP1 = sin(deg2rad(angleP1)) * refDirX + cos(deg2rad(angleP1)) * refDirY;

    double dirXP2 = cos(deg2rad(angleP2)) * refDirX - sin(deg2rad(angleP2)) * refDirY;
    double dirYP2 = sin(deg2rad(angleP2)) * refDirX + cos(deg2rad(angleP2)) * refDirY;

    // Move the distance along the new direction
    double xP1MFIS = refPointOld.getX() + distanceP1 * dirXP1;
    double yP1MFIS = refPointOld.getY() + distanceP1 * dirYP1;

    double xP2MFIS = refPointOld.getX() + distanceP2 * dirXP2;
    double yP2MFIS = refPointOld.getY() + distanceP2 * dirYP2;

    // Construct a new surface for the MFIS and copy the ID
    SurfaceT transformedSurfaceT(PointXY(xP1MFIS, yP1MFIS),
            PointXY(xP2MFIS, yP2MFIS),
            newSurf.getId());

    // Copy the occlusion information
    transformedSurfaceT.setP1Occluding(newSurf.isP1Occluding());
    transformedSurfaceT.setP2Occluding(newSurf.isP2Occluding());
    transformedSurfaceT.setBoundarySurf(newSurf.isBoundarySurf());

    return transformedSurfaceT;
}

/* Extends the surface (tobeextended) to have the length of (extensionReference).
 * Keeps the ID of tobeextended, but changes occlusion information.
 * If extendFromPoint1 is true, then P1 will be fixed and P2 be changed.
 */
const SurfaceT extendSurfaceT(const SurfaceT & toBeExtended,
        const SurfaceT & extensionReference,
        bool extendFromPoint1) {
    if (!toBeExtended.isValid()) {
        cerr << "WARNING: Cannot extend surface: No surface to extend!" << endl;
        return SurfaceT::INVALID;
    }

    double oldLength = toBeExtended.getLength();
    double newLength = extensionReference.getLength();
    PointXY newP1, newP2;

    // Extend the surface along its direction vector
    if (extendFromPoint1) {
        double dirX = (toBeExtended.getX2() - toBeExtended.getX1()) / oldLength;
        double dirY = (toBeExtended.getY2() - toBeExtended.getY1()) / oldLength;
        newP1 = toBeExtended.getP1();
        newP2 = PointXY(newP1.getX() + dirX * newLength,
                newP1.getY() + dirY * newLength);
    } else {
        double dirX = (toBeExtended.getX1() - toBeExtended.getX2()) / oldLength;
        double dirY = (toBeExtended.getY1() - toBeExtended.getY2()) / oldLength;
        newP2 = toBeExtended.getP2();
        newP1 = PointXY(newP2.getX() + dirX * newLength,
                newP2.getY() + dirY * newLength);
    }

    // Change the points and keep the ID
    SurfaceT extendedSurf(newP1, newP2, toBeExtended.getId());

    // If the (longer) surface has both occluding edges, then the extended one will have too.
    if (extensionReference.isP1Occluding() && extensionReference.isP2Occluding()) {
        extendedSurf.setP1Occluding(true);
        extendedSurf.setP2Occluding(true);
    } else {
        // Otherwise keep old occlusion info (usually one occluding edge out of two)
        extendedSurf.setP1Occluding(toBeExtended.isP1Occluding());
        extendedSurf.setP2Occluding(toBeExtended.isP2Occluding());
    }
    extendedSurf.setBoundarySurf(toBeExtended.isBoundarySurf());
    return extendedSurf;
}

// Gets a point on this surface that is the nearest to the point specified.

PointXY nearestPointOnSurf(const SurfaceT & surf, const PointXY & other, bool limitToEndpoints) {
    /* Find point P so that 	P = P1 + u(P2-P1)					(P is on the line P1,P2)
     * Also, 					(P3-P) dot (P2-P1) = 0.				(P3,P is orthogonal to P1,P2)
     * Substituted:				(P3-P1 - u(P2-P1)) dot (P2-P1) = 0
     * Solve for u!
     */

    double x1 = surf.getX1(), y1 = surf.getY1();
    double x2 = surf.getX2(), y2 = surf.getY2();
    double x3 = other.getX(), y3 = other.getY();

    double u = ((x3 - x1) * (x2 - x1) + (y3 - y1) * (y2 - y1))
            / (surf.getLength() * surf.getLength()); // Note: abs(P2-P1) == length

    // Now: p is on the infinite line described by p1 and p2. But is it on the surface segment?
    if (limitToEndpoints && (u < 0.0 || u > 1.0)) {
        // p is not within the surface bounds. Return the nearest end point instead!
        if (surf.getP1().distFromSq(other) < surf.getP2().distFromSq(other))
            return surf.getP1();
        else
            return surf.getP2();
    } else {
        // Substitute to find p
        double xP = x1 + u * (x2 - x1);
        double yP = y1 + u * (y2 - y1);
        return PointXY(xP, yP);
    }
}

/*
 * Returns the smallest distance between two surfaces (limited to their endpoints).
 */
double distBetweenSurfs(const SurfaceT & s1, const SurfaceT & s2) {
    // Distance s1.P1 from SurfaceT s2
    PointXY test = nearestPointOnSurf(s2, s1.getP1(), true);
    double closestDistSq = test.distFromSq(s1.getP1());

    // Distance s1.P2 from SurfaceT s2
    test = nearestPointOnSurf(s2, s1.getP2(), true);
    double testDistSq = test.distFromSq(s1.getP2());
    if (testDistSq < closestDistSq) {
        closestDistSq = testDistSq;
    }

    // Distance SurfaceT s1 from s2.P1
    test = nearestPointOnSurf(s1, s2.getP1(), true);
    testDistSq = test.distFromSq(s2.getP1());
    if (testDistSq < closestDistSq) {
        closestDistSq = testDistSq;
    }

    // Distance SurfaceT s1 from s2.P2
    test = nearestPointOnSurf(s1, s2.getP2(), true);
    testDistSq = test.distFromSq(s2.getP2());
    if (testDistSq < closestDistSq) {
        closestDistSq = testDistSq;
    }
    return sqrt(closestDistSq);
}

/**
 * Intersects a line with a circle. Returns a vector of points, containing all intersections (0, 1 or 2 points).
 */
vector<PointXY> intersectLineWithCircle(const PointXY & p1, const PointXY & p2, const PointXY & cCenter, double r) {
    vector<PointXY> intersectionPoints;

    // First, find the nearest point on the line
    PointXY pN = nearestPointOnSurf(SurfaceT(p1, p2), cCenter, false);
    double d = pN.distFrom(cCenter);

    /* Now, pN, cCenter and pI (intersection point) form a right triangle.
     * The three sides are r, d and m => m is the distance from pN to pI.
     */
    double m = sqrt(r * r - d * d);

    // In this case, the line and circle don't intersect
    if (isnan(m))
        return intersectionPoints;

    // The circle is touching the line, return one point
    if (m == 0) {
        intersectionPoints.push_back(pN);
        return intersectionPoints;
    }

    /* There are two intersection points, namely pN +- m
     * pI = pN +- m*(lineDir)
     */
    PointXY lineDir = p2 - p1;
    double lineLength = p2.distFrom(p1);
    lineDir = PointXY(lineDir.getX() / lineLength, lineDir.getY() / lineLength);

    intersectionPoints.push_back(pN + PointXY(m * lineDir.getX(), m * lineDir.getY()));
    intersectionPoints.push_back(pN - PointXY(m * lineDir.getX(), m * lineDir.getY()));
    return intersectionPoints;
}

/*
 * Returns true if a point is "behind" a line. Behind is the side of the line which (movementDir) faces.
 */
bool isBehindLine(const PointXY & pointToCheck, const SurfaceT & line, const PointXY & movementDir) {
    // Check to which side of the exit we are crossing
    const SurfaceT xAxis = SurfaceT(PointXY(0, 0), PointXY(100, 0));
    double lineTiltAngle = line.getAngleDiffTo(xAxis);
    double cT = cos(deg2rad(lineTiltAngle));
    double sT = sin(deg2rad(lineTiltAngle));

    PointXY transMoveDir = PointXY(cT * movementDir.getX() - sT * movementDir.getY(),
            sT * movementDir.getX() + cT * movementDir.getY());
    bool crossingInPositiveDir = transMoveDir.getY() > 0;

    PointXY transPoint = pointToCheck - line.getMiddle();
    transPoint = PointXY(cT * transPoint.getX() - sT * transPoint.getY(),
            sT * transPoint.getX() + cT * transPoint.getY());

    return (crossingInPositiveDir && transPoint.getY() > 0)
            || (!crossingInPositiveDir && transPoint.getY() <= 0);
}

double deg2rad(double degAngle) {
    return M_PI * degAngle / 180.0;
}

double rad2deg(double radAngle) {
    return (radAngle / M_PI) * 180.0;
}

// Reduces an angle to the smallest difference between two intersecting lines [0, 90]

double normAngleDiff(double degAngle) {
    degAngle = abs(degAngle);

    // Remove any full rotation
    int fullRotations = static_cast<int> (degAngle) / 360;
    degAngle -= (fullRotations * 360.0);

    if (degAngle > 270)
        return 360 - degAngle;

    if (degAngle > 180)
        return degAngle - 180;

    if (degAngle > 90)
        return 180 - degAngle;

    return degAngle;
}


bool pointInPolygon(const double & pointX, const double & pointY, const vector<PointXY>& points){
    int i, j, nvert = points.size();
        bool c = false;

        for(i = 0, j = nvert - 1; i < nvert; j = i++) {
          if( ( (points[i].getY() >= pointY ) != (points[j].getY() >= pointY) ) &&
              (pointX <= (points[j].getX() - points[i].getX()) * (pointY - points[i].getY()) / (points[j].getY() - points[i].getY()) + points[i].getX())
            )
            c = !c;
        }

        return c;
}






/* Determines if a point is inside a polygon. Uses ray casting algorithm.
 * NOTE: if the point is EXACTLY ON a vertex or edge, then result is true (inside).
 */
//bool pointInPolygon(const PointXY & point, const vector<SurfaceT> & polygon, bool strictly) {
//
//    bool first_time = false;
//    bool ray_change_needed;
//    SurfaceT ray;
//
//    while (true) {
//        ray_change_needed = false;
//
//        if (!first_time) { // avoid too much calls to 
//            ray = SurfaceT(point, PointXY(point.getX(), point.getY() + 10000000.0+rand()));
//            //cout<<ray.getX1()<<" "<<ray.getY1()<<" "<<ray.getX2()<<" "<<ray.getY2()<<" "<<endl;
//            first_time = true;
//            //waitHere();
//        } else {
//            ray = SurfaceT(point, PointXY(point.getX() + rand(), point.getY() + rand()));
//        }
//
//
//        /* Intersect the ray with the polygon edges.
//         * If the number of intersections is uneven, then the point is inside.
//         */
//        unsigned int numInters = 0;
//        for (vector<SurfaceT>::const_iterator it = polygon.begin(); it != polygon.end(); ++it) {
//            
//            // Special case 2: Directly on a vertex
//            if (point == it->getP1() || point == it->getP2()) {
//                return !strictly;
//            }
//
//            // Special case 3 : On an edge
//            if (it->contains(point)) {
//                return !strictly;
//            }
//
//            if (ray.contains(it->getP1()) || ray.contains(it->getP2())) {
//                ray_change_needed = true;
//                break;
//            }

//            // Otherwise, calculate the intersection.
//            if (it->intersects(ray)) {
//                numInters++;
//            }
//        }
//        
//        //cout<<ray_change_needed<<endl;
//
//        if (!ray_change_needed) {
//            return numInters % 2 != 0;
//        }
//    }
//}