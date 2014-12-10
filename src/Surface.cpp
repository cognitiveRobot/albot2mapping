#include <iostream>
#include <cmath>

#include "Surface.h"

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

void Surface::display() const{
    cout<<"X1: "<<P1.x<<" Y1: "<<P1.y<<" X2: "<<P2.x<<" Y2: "<<P2.y<<endl;
    cout<<"L: "<<this->length()<<endl;
}

double Surface::length() const{
    return sqrt((P2.x-P1.x)*(P2.x-P1.x)+(P2.y-P1.y)*(P2.y-P1.y));
}

cv::Point2f Surface::midPoint() {
    cv::Point2f mp;
  //  cout<<"p2.x "<<P2.x<<" p1.y "<<P1.x<<endl;
    mp.x = (P2.x + P1.x) / 2.0;
    mp.y = (P2.y + P1.y) / 2.0;
    
  //  cout<<"mp.x "<<mp.x<<" mp.y "<<mp.y<<endl;
    return mp;
}

int Surface::getId() {
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

//transforms and returns this surface P1 and P2 (which are in old coordinate) to a new coordinate
//whose center and angle with respect to old coordinate frame are given.

Surface Surface::transFrom(double newX, double newY, double angle) {
    
    float x1, y1, x2, y2; //float gives 7digit precision. which is enough for us.
    double a = P1.x - newX; //x-x0
    double b = P1.y - newY; //y-y0

    x1 = a * cos(angle) + b * sin(angle);
    y1 = b * cos(angle) - a * sin(angle);

    double c = P2.x - newX; //x-x0
    double d = P2.y - newY; //y-y0

    x2 = c * cos(angle) + d * sin(angle);
    y2 = d * cos(angle) - c * sin(angle);
    
    Surface transformed(x1,y1,x2,y2);
    
    return transformed;
}


Surface Surface::transformB(double newX, double newY, double angle) {
    
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
    
    Surface transformed(x1,y1,x2,y2);
    
    return transformed;
}

std::vector<cv::Point2f> Surface::getPoints() const{
    return points;
}

void Surface::reset() {
    points.clear();
    this->id = Surface::getUniqueId();
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

cv::Point2f Surface::getP1() const{
    return P1;
}

cv::Point2f Surface::getP2() const{
    return P2;
}

void Surface::setSurfaceSimple() {

    /* Set the ends of the surface */
    if(points.size() > 1) {
    setP1(points[0].x, points[0].y);
    setP2(points[points.size()-1].x,points[points.size()-1].y);
    }

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

double Surface::distFromP1ToPoint(const float & a, const float & b) {
    return sqrt((P1.x-a)*(P1.x-a)+(P1.y-b)*(P1.y-b));
}

double Surface::distFromP2ToPoint(const float & a, const float & b) {
    return sqrt((P2.x-a)*(P2.x-a)+(P2.y-b)*(P2.y-b));
}

double Surface::getAngleWithXaxis() {
    double x21 = P2.x - P1.x;
    double y21 = P2.y - P1.y;
    
   // cout<<"x21 "<<x21<<" y21 "<<y21<<endl;

    double angle1 = acos(x21 / length());
    //cout<<angle1<<endl;
    
    if (y21 < 0)
        angle1 = 2 * M_PI - angle1;

   // cout<<angle1<<endl;
    return ((180 / M_PI) * angle1);
}

double Surface::getAngleWithSurface(Surface s) {


    return s.getAngleWithXaxis() - getAngleWithXaxis();
}



///////// Some functions //////////
bool SortBasedOnLength(Surface surf1, Surface surf2) {
    return surf1.length() < surf2.length();
}

void displaySurfaces(const std::vector<Surface> & surfaces) {
    for(unsigned int i=0; i<surfaces.size(); i++) {
        surfaces[i].display();
    }
}