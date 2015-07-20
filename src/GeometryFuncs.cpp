/*
 *  Helper functions for geometric calculations.
 *
 *  Thomas
 */

#include "GeometryFuncs.h"
#include "ImageProcessing.h"
#include "Printer.h"
#include "Node.h"
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

//it finds anypoint (pointX, pointY) is inside or outside of the polygon (surfaces, and robotPosition).
//if any point lies on the vertex of the polygon or on the edge of the polygon then it returns false.
// http://stackoverflow.com/questions/11716268/point-in-polygon-algorithm
// http://www.ecse.rpi.edu/~wrf/Research/Short_Notes/pnpoly.html

bool pointInPolygon(const double & pointX, const double & pointY, const vector<PointXY>& points) {
    int i, j, nvert = points.size();
    bool c = false;

    for (i = 0, j = nvert - 1; i < nvert; j = i++) {
        if (((points[i].getY() >= pointY) != (points[j].getY() >= pointY)) &&
                (pointX <= (points[j].getX() - points[i].getX()) * (pointY - points[i].getY()) / (points[j].getY() - points[i].getY()) + points[i].getX())
                )
            c = !c;
    }

    return c;
}

bool surfaceIntersectsPolygon(const Surface& surface, const vector<PointXY>& polygon) {
    for (unsigned int i = 0; i < polygon.size() - 1; i++) {
        if (surface.intersects(Surface(polygon[i].getX(), polygon[i].getY(), polygon[i + 1].getX(), polygon[i + 1].getY()))) {
            return true;
        }
    }
    return false;
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

Surface principalComponentAnalysis(vector<PointXY> pointsToAnalyze) {
    int numPoints = pointsToAnalyze.size();

    // Substract the mean
    /*   double xMean = 0;
       double yMean = 0;
       for (unsigned int i = 0; i < numPoints; i++) {
           xMean += pointsToAnalyze[i].getX();
           yMean += pointsToAnalyze[i].getY();
       }
       xMean = xMean / numPoints;
       yMean = yMean / numPoints;

       PointXY points [numPoints];

       for (unsigned int i = 0; i < numPoints; i++) {
           points[i] = PointXY(pointsToAnalyze[i].getX() - xMean, pointsToAnalyze[i].getY() - yMean);
       }

       // Calculate covariance matrix
       double covarianceMatrix [2][2] = {
           {0, 0},
           {0, 0}};
       for (unsigned int i = 0; i < numPoints; i++) {
           covarianceMatrix[0][0] += (points[i].getX() - xMean) * (points[i].getX() - xMean);
           covarianceMatrix[0][1] += (points[i].getX() - xMean) * (points[i].getY() - yMean);
           covarianceMatrix[1][1] += (points[i].getY() - yMean) * (points[i].getY() - yMean);
       }
       covarianceMatrix[0][0] = covarianceMatrix[0][0] / (numPoints - 1);
       covarianceMatrix[0][1] = covarianceMatrix[0][1] / (numPoints - 1);
       covarianceMatrix[1][1] = covarianceMatrix[1][1] / (numPoints - 1);
       covarianceMatrix[1][0] = covarianceMatrix[0][1]; //cov(x,y)=cov(y,x)
     */
    cv::Mat data_pts(numPoints, 2, CV_64FC1);
    for (unsigned int i = 0; i < data_pts.rows; ++i) {
        data_pts.at<double>(i, 0) = pointsToAnalyze[i].getX();
        data_pts.at<double>(i, 1) = pointsToAnalyze[i].getY();
    }

    cv::PCA pca_analysis(data_pts, cv::Mat(), CV_PCA_DATA_AS_ROW);

    PointXY eigenVec(pca_analysis.eigenvectors.at<double>(0, 0), pca_analysis.eigenvectors.at<double>(0, 1));
    double eigenVal = pca_analysis.eigenvalues.at<double>(0, 0);

    double coordsAlongVector [numPoints];
    coordsAlongVector[0] = pointsToAnalyze[0].getX() * eigenVec.getX() + pointsToAnalyze[0].getY() * eigenVec.getY();
    int indexMinCoord = 0;
    int indexMaxCoord = 0;
    double minCoord = coordsAlongVector[0];
    double maxCoord = coordsAlongVector[0];
    for (unsigned int i = 1; i < numPoints; i++) {
        coordsAlongVector[i] = pointsToAnalyze[i].getX() * eigenVec.getX() + pointsToAnalyze[i].getY() * eigenVec.getY();
        if (coordsAlongVector[i] < minCoord) {
            minCoord = coordsAlongVector[i];
            indexMinCoord = i;
        } else if (coordsAlongVector[i] > maxCoord) {
            maxCoord = coordsAlongVector[i];
            indexMaxCoord = i;
        }
    }
    long halfLength = (maxCoord - minCoord) / 2;
    PointXY start(pca_analysis.mean.at<double>(0, 0), pca_analysis.mean.at<double>(0, 1));
    return Surface(start.getX() - halfLength * eigenVec.getX(), start.getY() - halfLength * eigenVec.getY(), start.getX() + halfLength * eigenVec.getX(), start.getY() + halfLength * eigenVec.getY());
}

/* DBSCAN - density-based spatial clustering of applications with noise */
vector<vector<PointXY> > DBSCAN_points(vector<PointXY> *points, float eps, int minPts) {
    vector<vector<PointXY> > clusters;
    vector<bool> clustered;
    vector<int> noise;
    vector<bool> visited;
    vector<int> neighborPts;
    vector<int> neighborPts_;
    int c;

    int nbPoints = points->size();

    //init clustered and visited
    for (int k = 0; k < nbPoints; k++) {
        clustered.push_back(false);
        visited.push_back(false);
    }

    //C =0;
    c = 0;
    clusters.push_back(vector<PointXY>()); //will stay empty?

    //for each unvisted point P in dataset points
    for (int i = 0; i < nbPoints; i++) {
        if (!visited[i]) {
            //Mark P as visited
            visited[i] = true;
            neighborPts = regionQuery(points, &points->at(i), eps);
            if (neighborPts.size() < minPts)
                //Mark P as Noise
                noise.push_back(i);
            else {
                clusters.push_back(vector<PointXY>());
                c++;
                //expand cluster
                // add P to cluster c
                clusters[c].push_back(points->at(i));
                clustered[i] = true;
                //for each point P' in neighborPts
                for (int j = 0; j < neighborPts.size(); j++) {
                    //if P' is not visited
                    if (!visited[neighborPts[j]]) {
                        //Mark P' as visited
                        visited[neighborPts[j]] = true;
                        neighborPts_ = regionQuery(points, &points->at(neighborPts[j]), eps);
                        if (neighborPts_.size() >= minPts) {
                            neighborPts.insert(neighborPts.end(), neighborPts_.begin(), neighborPts_.end());
                        }
                    }
                    // if P' is not yet a member of any cluster
                    // add P' to cluster c
                    if (!clustered[neighborPts[j]])
                        clusters[c].push_back(points->at(neighborPts[j]));
                    clustered[neighborPts[j]] = true;
                }
            }

        }
    }
    return clusters;
}
//For DBSCAN

vector<int> regionQuery(vector<PointXY> *points, PointXY *keypoint, float eps) {
    float dist;
    vector<int> retKeys;
    for (int i = 0; i < points->size(); i++) {
        dist = sqrt(pow((keypoint->getX() - points->at(i).getX()), 2) + pow((keypoint->getY() - points->at(i).getY()), 2));
        if (dist <= eps && dist != 0.0f) {
            retKeys.push_back(i);
        }
    }
    return retKeys;
}

bool pointsOnSameSideOfSurface(cv::Point2f p1, cv::Point2f p2, Surface surf) {
    int orientP1 = orientation(p1, surf.getP1(), surf.getP2());
    int orientP2 = orientation(p2, surf.getP1(), surf.getP2());
    return orientP1 == orientP2;
}

vector<Surface> findTangents(cv::Point2f circleCentre, cv::Point2f point, double radius) {
    vector<Surface> tangents;

    if (radius == 0) {
        cout << "radius = 0" << endl;
        return tangents;
    }

    //shift and scale
    double nx = (point.x - circleCentre.x) / radius;
    double ny = (point.y - circleCentre.y) / radius;
    double xy = nx * nx + ny*ny;

    if (xy == 1) {
        //Point lies at circumference, one tangent
        cout << "1 tangent" << endl;
        PointXY radiusVect(point.x - circleCentre.x, point.y - circleCentre.y);
        PointXY tangentVect(-radiusVect.getY(), radiusVect.getX());
        tangents.push_back(Surface(point.x, point.y, point.x + tangentVect.getX(), point.y + tangentVect.getY()));
        return tangents;
    }

    if (xy < 1) {
        //Point lies inside the circle, no tangent
        cout << "no tangent r= " << radius << " p= " << circleCentre.x << " " << circleCentre.y << endl;
        return tangents;
    }

    //Two tangents
    double d = ny * sqrt(xy - 1);
    double tx0 = (nx - d) / xy;
    double tx1 = (nx + d) / xy;
    double yt0;
    double yt1;
    if (ny != 0) {
        yt0 = circleCentre.y + radius * (1 - tx0 * nx) / ny;
        yt1 = circleCentre.y + radius * (1 - tx1 * nx) / ny;
    } else {
        //point at the center horizontal, y=0
        d = radius * sqrt(1 - tx0 * tx0);
        yt0 = circleCentre.y + d;
        yt1 = circleCentre.y - d;
    }
    //restore scale and position
    double xt0 = circleCentre.x + radius*tx0;
    double xt1 = circleCentre.x + radius*tx1;

    vector<PointXY> points;
    points.push_back(PointXY(point.x, point.y));
    points.push_back(PointXY(circleCentre.x, circleCentre.y));
    vector<Surface> surf;
    surf.push_back(Surface(point.x, point.y, xt0, yt0));
    surf.push_back(Surface(point.x, point.y, xt1, yt1));
    plotPointsAndSurfacesGNU("../outputs/Maps/tangent.png", points, surf);

    tangents.push_back(Surface(point.x, point.y, xt0, yt0));
    tangents.push_back(Surface(point.x, point.y, xt1, yt1));

    return tangents;
}
/*
vector<Node*> findPathAStar(Node *start, Node *goal, vector<Surface> surfaces){
    vector<Node*> closedSet;
    vector<Node*> openSet;
    openSet.push_back(start);
    vector<Node*> path;
    Node* currentNode=start;
    
    
    return path;
}
    closedset := the empty set    // The set of nodes already evaluated.
    openset := {start}    // The set of tentative nodes to be evaluated, initially containing the start node
    came_from := the empty map    // The map of navigated nodes.
 
    g_score := map with default value of Infinity
    g_score[start] := 0    // Cost from start along best known path.
    // Estimated total cost from start to goal through y.
    f_score = map with default value of Infinity
    f_score[start] := g_score[start] + heuristic_cost_estimate(start, goal)
     
    while openset is not empty
        current := the node in openset having the lowest f_score[] value
        if current = goal
            return reconstruct_path(came_from, goal)
         
        remove current from openset
        add current to closedset
        for each neighbor in neighbor_nodes(current)
            if neighbor in closedset
                continue
            tentative_g_score := g_score[current] + dist_between(current,neighbor)
 
            if neighbor not in openset or tentative_g_score < g_score[neighbor] 
                came_from[neighbor] := current
                g_score[neighbor] := tentative_g_score
                f_score[neighbor] := g_score[neighbor] + heuristic_cost_estimate(neighbor, goal)
                if neighbor not in openset
                    add neighbor to openset
 
    return failure

function reconstruct_path(came_from,current)
    total_path := [current]
    while current in came_from:
        current := came_from[current]
        total_path.append(current)
    return total_path
 * */