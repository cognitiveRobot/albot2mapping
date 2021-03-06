#include <iostream>
#include <cmath>

#include "Surface.h"
#include "Constants.h"


using namespace std;

/* Functions definitions for Surfaces class */

int Surface::idCounter = 0;

Surface::Surface() {
    this->id = Surface::getUniqueId();
}

Surface::Surface(float X1, float Y1, float X2, float Y2) {
    P1.x = X1;
    P1.y = Y1;
    P2.x = X2;
    P2.y = Y2;
    this->id = Surface::getUniqueId();
}

Surface::~Surface() {

}

bool Surface::operator==(Surface const& b) {
    if (this->getP1().x == b.getP1().x
            && this->getP1().y == b.getP1().y
            && this->getP2().x == b.getP2().x
            && this->getP2().y == b.getP2().y) {
        return true;
    }
    return false;
}

void Surface::display() const {
    cout << "X1: " << P1.x << " Y1: " << P1.y << " X2: " << P2.x << " Y2: " << P2.y << endl;
    cout << "L: " << this->length() << endl;
}

double Surface::length() const {
    return sqrt((P2.x - P1.x)*(P2.x - P1.x)+(P2.y - P1.y)*(P2.y - P1.y));
}

cv::Point2f Surface::midPoint() const {
    cv::Point2f mp;
    //  cout<<"p2.x "<<P2.x<<" p1.y "<<P1.x<<endl;
    mp.x = (P2.x + P1.x) / 2.0;
    mp.y = (P2.y + P1.y) / 2.0;

    //  cout<<"mp.x "<<mp.x<<" mp.y "<<mp.y<<endl;
    return mp;
}

int Surface::getId() const {
    return this->id;
}

int Surface::getUniqueId() {
    return ++Surface::idCounter;
}

void Surface::addPoint(cv::Point2f newPoint) {
    points.push_back(newPoint);
}

void Surface::coordTransf(cv::Point2f newCenter, float hX, float hY) {

    for (unsigned int i = 0; i < points.size(); i++) {

        // Change the center of the frame reference
        points[i].x -= newCenter.x;
        points[i].y -= newCenter.y;

        // Homothetic transformation to adapt to axes
        points[i].x *= hX;
        points[i].y *= hY;

    }

}

void Surface::rotate(cv::Point3f Center) {
    float distance;
    float teta, angle;

    teta = 0;
    angle = Center.z;
    angle *= M_PI;
    angle /= (float) 180;

    P1.x = P1.x;
    P1.y = P1.y;

    distance = sqrt(P1.x * P1.x + P1.y * P1.y); // Getting distance from center

    if (P1.x < 0) {
        teta = atan(P1.y / P1.x) + M_PI;
    } else
        teta = atan(P1.y / P1.x);

    // Set angle for rotated point
    teta += angle;

    // Set new position
    P1.x = distance * cos(teta);
    P1.y = distance * sin(teta);

    P2.x = P2.x;
    P2.y = P2.y;

    distance = sqrt(P2.x * P2.x + P2.y * P2.y); // Getting distance from center

    if (P2.x < 0) {
        teta = atan(P2.y / P2.x) + M_PI;
    } else
        teta = atan(P2.y / P2.x);

    // Set angle for rotated point
    teta += angle;

    // Set new position
    P2.x = distance * cos(teta);
    P2.y = distance * sin(teta);

}

//rotates P2 according to the angle.

void Surface::rotateAroundP1(double angle) {
    float r;
    float teta;

    P2.x = P2.x - P1.x;
    P2.y = P2.y - P1.y;
    P2.y = -P2.y;

    r = sqrt(P2.x * P2.x + P2.y * P2.y); // Getting distance from center

    if (P2.x < 0) {
        if (P2.y < 0) {
            teta = atan(P2.y / P2.x) - M_PI;
        } else {
            teta = atan(P2.y / P2.x) + M_PI;
        }
    } else
        teta = atan(P2.y / P2.x);

    // Set angle for rotated point
    angle *= M_PI;
    angle /= (float) 180;
    teta += -angle;

    // Set new position
    P2.x = r * cos(teta);
    P2.y = r * sin(teta);

    P2.x = P2.x + P1.x;
    P2.y = P1.y - P2.y;
}

//transforms and returns this surface P1 and P2 (which are in old coordinate) to a new coordinate
//whose center and angle with respect to old coordinate frame are given.

Surface Surface::transFrom(double newX, double newY, double angle) const {

    float x1, y1, x2, y2; //float gives 7digit precision. which is enough for us.
    double a = P1.x - newX; //x-x0
    double b = P1.y - newY; //y-y0

    x1 = a * cos(angle) + b * sin(angle);
    y1 = b * cos(angle) - a * sin(angle);

    double c = P2.x - newX; //x-x0
    double d = P2.y - newY; //y-y0

    x2 = c * cos(angle) + d * sin(angle);
    y2 = d * cos(angle) - c * sin(angle);

    Surface transformed(x1, y1, x2, y2);

    return transformed;
}

Surface Surface::transformB(double newX, double newY, double angle) const {

    float x1, y1, x2, y2; //float gives 7digit precision. which is enough for us.

    //rotation
    x1 = P1.x * cos(angle) - P1.y * sin(angle);
    y1 = P1.y * cos(angle) + P1.x * sin(angle);
    //translation
    x1 += newX;
    y1 += newY;

    //rotation
    x2 = P2.x * cos(angle) - P2.y * sin(angle);
    y2 = P2.y * cos(angle) + P2.x * sin(angle);
    //translation
    x2 += newX;
    y2 += newY;

    Surface transformed(x1, y1, x2, y2);

    return transformed;
}

std::vector<cv::Point2f> Surface::getPoints() const {
    return points;
}

void Surface::reset() {
    points.clear();
    this->id = Surface::getUniqueId();
}

std::vector<cv::Point2f> Surface::getAllPoints() {
    std::vector<cv::Point2f> allPoints;
    allPoints.push_back(P1);
    if (P2.x != P1.x) {
        double slope = (P2.y - P1.y) / (P2.x - P1.x);
        double b = P1.y - (slope * P1.x);
        for (int i = P1.x; i < P2.x; i += 10) {
            allPoints.push_back(cv::Point2f(i, slope * i + b));
        }
    }
    allPoints.push_back(P2);
    return allPoints;
}

void Surface::set(float X1, float Y1, float X2, float Y2) {
    P1.x = X1;
    P1.y = Y1;
    P2.x = X2;
    P2.y = Y2;
}

void Surface::setP1(float X, float Y) {
    P1.x = X;
    P1.y = Y;
}

void Surface::setP2(float X, float Y) {
    P2.x = X;
    P2.y = Y;
}

cv::Point2f Surface::getP1() const {
    return P1;
}

cv::Point2f Surface::getP2() const {
    return P2;
}

void Surface::setSurfaceSimple() {

    /* Set the ends of the surface */
    if (points.size() > 1) {
        setP1(points[0].x, points[0].y);
        setP2(points[points.size() - 1].x, points[points.size() - 1].y);
    }

}

void Surface::setP1Occluding(bool occluding) {
    p1Occluding = occluding;
}

bool Surface::getP1Occluding() {
    return p1Occluding;
}

void Surface::setP2Occluding(bool occluding) {
    p2Occluding = occluding;
}

bool Surface::getP2Occluding() {
    return p2Occluding;
}


//it uses OpenCV line fitting algorithm to set a surfaces from a vector of points

void Surface::setSurface() {

    cv::Vec4f vecLine; // Orientation vector
    std::vector < cv::Point2f > myPoints = points;
    float distance = 0; // Size of the surface

    /* Set surface size by measuring the longest distance between 2 points */
    for (unsigned i = 0; i < myPoints.size(); i++) {
        for (unsigned j = 0; j < myPoints.size(); j++) {
            if (distance
                    < sqrt(
                    (myPoints[i].x - myPoints[j].x)
                    * (myPoints[i].x - myPoints[j].x)
                    + (myPoints[i].y - myPoints[j].y)
                    * (myPoints[i].y - myPoints[j].y)))
                distance = sqrt(
                    (myPoints[i].x - myPoints[j].x)
                    * (myPoints[i].x - myPoints[j].x)
                    + (myPoints[i].y - myPoints[j].y)
                    * (myPoints[i].y - myPoints[j].y));
        }
    }

    /* Set the orientation vector to fit with the set of points*/
    cv::fitLine(myPoints, vecLine, CV_DIST_L2, 0, 0.01, 0.01);

    /* Set the ends of the surface */
    setP1(myPoints[0].x, myPoints[0].y);
    setP2(distance * vecLine[0] + P1.x, distance * vecLine[1] + P1.y);

}

void Surface::setColors(std::vector<Color> colors) {
    this->colors = colors;
    this->averageColor = Color::calculateAverageColor(this->colors);
}

Color Surface::getAverageColor() {
    return this->averageColor;
}

std::vector<Color> Surface::getColors() {
    return colors;
}

double Surface::distFromP1ToPoint(const float & a, const float & b) const {
    return sqrt((P1.x - a)*(P1.x - a)+(P1.y - b)*(P1.y - b));
}

double Surface::distFromP2ToPoint(const float & a, const float & b) const {
    return sqrt((P2.x - a)*(P2.x - a)+(P2.y - b)*(P2.y - b));
}

double Surface::distFromMiddlePointToPoint(const float & a, const float & b) const {
    return sqrt((this->midPoint().x - a)*(this->midPoint().x - a)+(this->midPoint().y - b)*(this->midPoint().y - b));
}

double Surface::getAngleWithXaxis() const {
    double x21 = P2.x - P1.x;
    double y21 = P2.y - P1.y;

    //cout<<"x21 "<<x21<<" y21 "<<y21<<endl;

    if (y21 == 0) {
        return 0;
    }

    double angle1 = acos(x21 / length());
    //cout<<angle1<<endl;

    if (y21 < 0)
        angle1 = 2 * M_PI - angle1;

    //cout<<angle1<<endl;
    return ((180 / M_PI) * angle1);
}

double Surface::getAngleWithSurface(Surface s) const {


    return s.getAngleWithXaxis() - this->getAngleWithXaxis();
}

//it finds angle between this surface (direction p1 to p2) and p1 and given point. 

double Surface::getAngleFromP1ToPoint(const double & a, const double & b) const {
    double x21 = P2.x - P1.x;
    double y21 = P2.y - P1.y;
    double x31 = a - P1.x;
    double y31 = b - P1.y;

    double la = length();
    double lb = this->distFromP1ToPoint(a, b);

    double r1 = x21 / la;
    double angle1 = acos(r1);

    double r2 = x31 / lb;
    double angle2 = acos(r2);
    
 //   cout<<""<<endl; //It doesn't make sense but if I delete this line the findWayHome crashes...

    if (lb == 0)
        return 0;

    if (y21 < 0)
        angle1 = 2 * M_PI - angle1;
    angle1 = ((180 / M_PI) * angle1);

    if (y31 < 0)
        angle2 = 2 * M_PI - angle2;
    angle2 = ((180 / M_PI) * angle2);

    double diff = angle2 - angle1;
    
    if (diff > 0)
        return diff;
    else
        return 360 + diff;
}

//it finds angle between this surface (direction p2 to p1) and p2 and given point. 

double Surface::getAngleFromP2ToPoint(const double & a, const double & b) const {
    double x21 = P1.x - P2.x;
    double y21 = P1.y - P2.y;
    double x31 = a - P2.x;
    double y31 = b - P2.y;

    double la = length();
    double lb = this->distFromP2ToPoint(a, b);

    double r1 = x21 / la;
    double angle1 = acos(r1);

    double r2 = x31 / lb;
    double angle2 = acos(r2);

    if (lb == 0)
        return 0;

    if (y21 < 0)
        angle1 = 2 * M_PI - angle1;
    angle1 = ((180 / M_PI) * angle1);

    if (y31 < 0)
        angle2 = 2 * M_PI - angle2;
    angle2 = ((180 / M_PI) * angle2);

    double diff = angle2 - angle1;

    if (diff > 0)
        return diff;
    else
        return 360 + diff;
}

//given that angle, distance, and reference direction, find the x and y co-ordinate with respect to this surface.
//refDirection 1 means, from p1 to p2 of this surface is the direction of reference.

//for testing.
//double x1, y1, x2, y2, ang, dist;
//        vector<Surface> cvLocated;
//        for(unsigned int i=1; i<curView.getSurfaces().size(); i++) {
//            ang = curView.getSurfaces()[0].getAngleFromP2ToPoint(curView.getSurfaces()[i].getP1().x,curView.getSurfaces()[i].getP1().y);
//            dist = curView.getSurfaces()[0].distFromP2ToPoint(curView.getSurfaces()[i].getP1().x,curView.getSurfaces()[i].getP1().y);
//            
//            curView.getSurfaces()[0].locatePointAt(x1,y1,ang,dist,2);
//            
//            ang = curView.getSurfaces()[0].getAngleFromP2ToPoint(curView.getSurfaces()[i].getP2().x,curView.getSurfaces()[i].getP2().y);
//            dist = curView.getSurfaces()[0].distFromP2ToPoint(curView.getSurfaces()[i].getP2().x,curView.getSurfaces()[i].getP2().y);
//            
//            curView.getSurfaces()[0].locatePointAt(x2,y2,ang,dist,2);
//            
//            cvLocated.push_back(Surface(x1,y1,x2,y2));
//        }
//        plotSurfacesGNU("../outputs/Maps/cvLocated.png",cvLocated);
//        waitHere();

void Surface::locatePointAt(double & x, double & y, const double & ang, const double & distance, const int & refDirection) {
    double angle = ang * CONVERT_TO_RADIAN;
    if (refDirection == 1) {
        x = ((this->getP2().x - this->getP1().x) / this->length()) * cos(angle)-((this->getP2().y - this->getP1().y) / this->length()) * sin(angle);
        y = ((this->getP2().x - this->getP1().x) / this->length()) * sin(angle)+((this->getP2().y - this->getP1().y) / this->length()) * cos(angle);

        x = x * distance + this->getP1().x;
        y = y * distance + this->getP1().y;
    } else {
        x = ((this->getP1().x - this->getP2().x) / this->length()) * cos(angle)-((this->getP1().y - this->getP2().y) / this->length()) * sin(angle);
        y = ((this->getP1().x - this->getP2().x) / this->length()) * sin(angle)+((this->getP1().y - this->getP2().y) / this->length()) * cos(angle);

        x = x * distance + this->getP2().x;
        y = y * distance + this->getP2().y;
    }
}

//given that angle, distance of two points, and reference direction, find the surface with respect to this surface.

Surface Surface::makeASurfaceAt(const double & ang1, const double & dist1, const double & ang2, const double & dist2, const int & refDirection) {
    double x1, y1, x2, y2;
    //point1
    locatePointAt(x1, y1, ang1, dist1, refDirection);
    //point2
    locatePointAt(x2, y2, ang2, dist2, refDirection);

    return Surface(x1, y1, x2, y2);
}

void Surface::shiftOneEnd(const double & ang, const double & distance, const int & refDirection) {
    double x, y;
    //locate the shifting point;
    locatePointAt(x, y, ang, distance, refDirection);

    if (refDirection == 1)
        this->setP2(x, y);
    else
        this->setP1(x, y);
}

SurfaceT Surface::ToSurfaceT() const {
    return SurfaceT(PointXY(P1.x, P1.y), PointXY(P2.x, P2.y));
}

void Surface::orderEndpoints(Surface robotOrientation) {
    double angleP1 = robotOrientation.getAngleFromP1ToPoint(P1.x, P1.y);
    double angleP2 = robotOrientation.getAngleFromP1ToPoint(P2.x, P2.y);
    if (angleP1 > 180) angleP1 -= 360;
    if (angleP2 > 180) angleP2 -= 360;
    if (angleP2 > angleP1) {
        swap(P1, P2);
    }
}

//http://www.geeksforgeeks.org/check-if-two-given-line-segments-intersect/

bool Surface::intersects(Surface other) const {
    cv::Point2f p1 = P1;
    cv::Point2f q1 = P2;
    cv::Point2f p2 = other.getP1();
    cv::Point2f q2 = other.getP2();

    if (min(p2.x, q2.x) > max(p1.x, q1.x) || min(p2.y, q2.y) > max(p1.y, q1.y)
            || min(p1.x, q1.x) > max(p2.x, q2.x) || min(p1.y, q1.y) > max(p2.y, q2.y)) {
        //Surfaces are not in the same x and y ranges
        return false;
    }

    // Find the four orientations needed for general and
    // special cases
    int o1 = orientation(p1, q1, p2);
    int o2 = orientation(p1, q1, q2);
    int o3 = orientation(p2, q2, p1);
    int o4 = orientation(p2, q2, q1);

    // General case
    if (o1 != o2 && o3 != o4)
        return true;

    // Special Cases
    // p1, q1 and p2 are colinear and p2 lies on segment p1q1
    if (o1 == 0 && onSegment(p1, p2, q1)) return true;

    // p1, q1 and p2 are colinear and q2 lies on segment p1q1
    if (o2 == 0 && onSegment(p1, q2, q1)) return true;

    // p2, q2 and p1 are colinear and p1 lies on segment p2q2
    if (o3 == 0 && onSegment(p2, p1, q2)) return true;

    // p2, q2 and q1 are colinear and q1 lies on segment p2q2
    if (o4 == 0 && onSegment(p2, q1, q2)) return true;

    return false; // Doesn't fall in any of the above cases
}

PointXY Surface::projectPointOnSurface(double x, double y) {
    /*  double coordOnSurface=cos(getAngleFromP1ToPoint(x,y));
      double newX=(P2.x-P1.x)*coordOnSurface;
      double newY=(P2.y-P1.y)*coordOnSurface;
     */

    double m = (P2.y - P1.y) / (P2.x - P1.x);
    double b = P1.y - (m * P1.x);

    double newX = (m * y + x - m * b) / (m * m + 1);
    double newY = (m * m * y + m * x + b) / (m * m + 1);

    return PointXY(newX, newY);
}

double Surface::distFromPoint(double x, double y) {
    return abs((P2.y - P1.y) * x - (P2.x - P1.x) * y + P2.x * P1.y - P2.y * P1.x) / sqrt((P2.y - P1.y)*(P2.y - P1.y)+(P2.x - P1.x)*(P2.x - P1.x));
}

// Reference Surfaces

ReferenceSurfaces::ReferenceSurfaces() {
    refPoint = 0;
}

ReferenceSurfaces::ReferenceSurfaces(const Surface & surf1, const Surface & surf2) {
    mapSurface = surf1;
    viewSurface = surf2;
}

ReferenceSurfaces::~ReferenceSurfaces() {

}

Surface ReferenceSurfaces::getMapSurface() {
    return mapSurface;
}

void ReferenceSurfaces::setMapSurface(const Surface & surf1) {
    mapSurface = surf1;
}

Surface ReferenceSurfaces::getViewSurface() {
    return viewSurface;
}

void ReferenceSurfaces::setViewSurface(const Surface & surf1) {
    viewSurface = surf1;
}

int ReferenceSurfaces::getRefPoint() {
    return refPoint;
}

void ReferenceSurfaces::setRefPoint(const int & i) {
    refPoint = i;
}

void ReferenceSurfaces::display() {
    cout << "MapId: " << mapSurface.getId() << " CVId: " << viewSurface.getId() << " RefPoint: " << refPoint << endl;
}


///////// Some functions //////////

bool SortBasedOnLength(Surface surf1, Surface surf2) {
    return surf1.length() < surf2.length();
}

void displaySurfaces(const std::vector<Surface> & surfaces) {
    for (unsigned int i = 0; i < surfaces.size(); i++) {
        surfaces[i].display();
    }
}

vector<Surface> convertSurfaceT2Surface(const vector<SurfaceT> & surfs) {
    vector<Surface> surfaces;
    for (unsigned int i = 0; i < surfs.size(); i++) {
        surfaces.push_back(Surface(surfs[i].getX1(), surfs[i].getY1(), surfs[i].getX2(), surfs[i].getY2()));
    }
    return surfaces;
}

//it transforms pv to cv

vector<Surface> transform(const vector<Surface> & pvSurfaces, const double & angle, const double & distance) {

    double ang = angle * CONVERT_TO_RADIAN; // degree to randian.
    //find cv center in the pv coordinate frame.
    //need to convert robot position from mm to cm.
    double newX = (distance / 1.0) * sin(-ang); //x= d*cos(th) = d*cos(90-angle) = d*sin(angle) //as aris give - value for right turn
    double newY = (distance / 1.0) * cos(-ang); //y=d*sin(th)=d*sin(90-angle)=d*cos(angle)

    //transform pvLandmarks to cv coordinate frame.
    std::vector<Surface> pvSurfacesOnCV;
    for (unsigned int i = 0; i < pvSurfaces.size(); i++) {
        pvSurfacesOnCV.push_back(pvSurfaces[i].transFrom(newX, newY, ang));
    }

    return pvSurfacesOnCV;
}

//it transforms cv to pv

vector<Surface> transformB(const vector<Surface> & cvSurfaces, const double & angle, const double & distance) {
    double ang = angle * CONVERT_TO_RADIAN; // degree to randian.
    //find cv center in the pv coordinate frame.
    //need to convert robot position from mm to cm.
    double newX = (distance / 1.0) * sin(-ang); //x= d*cos(th) = d*cos(90-angle) = d*sin(angle) //as aris give - value for right turn
    double newY = (distance / 1.0) * cos(-ang); //y=d*sin(th)=d*sin(90-angle)=d*cos(angle)

    //transform cvLandmarks to pv coordinate frame.
    std::vector<Surface> cvSurfacesOnPV;
    for (unsigned int i = 0; i < cvSurfaces.size(); i++) {
        cvSurfacesOnPV.push_back(cvSurfaces[i].transformB(newX, newY, ang));
    }

    return cvSurfacesOnPV;

}

// Given three colinear points p, q, r, the function checks if
// point q lies on line segment 'pr'

bool onSegment(cv::Point2f p, cv::Point2f q, cv::Point2f r) {
    if (q.x <= max(p.x, r.x) && q.x >= min(p.x, r.x) &&
            q.y <= max(p.y, r.y) && q.y >= min(p.y, r.y))
        return true;

    return false;
}

// To find orientation of ordered triplet (p, q, r).
// The function returns following values
// 0 --> p, q and r are colinear
// 1 --> Clockwise
// 2 --> Counterclockwise

int orientation(cv::Point2f p, cv::Point2f q, cv::Point2f r) {
    // See 10th slides from following link for derivation of the formula
    // http://www.dcs.gla.ac.uk/~pat/52233/slides/Geometry1x1.pdf
    int val = (q.y - p.y) * (r.x - q.x) -
            (q.x - p.x) * (r.y - q.y);

    if (val == 0) return 0; // colinear

    return (val > 0) ? 1 : 2; // clock or counterclock wise
}