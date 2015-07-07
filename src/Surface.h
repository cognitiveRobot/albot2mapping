#ifndef SURFACE_H
#define SURFACE_H

/* ------------------------- Basic Includes ------------------------- */
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <assert.h>
#include <iostream>
#include <sstream>
#include <cmath>
#include <vector>

/* ------------------------- Open CV includes ------------------------- */
#include <opencv/cv.h>
#include <opencv2/opencv.hpp>

/* ------------------------- Own includes ------------------------- */
#include "Color.h"
#include "PointAndSurface.h"


/* Class of surfaces that are to be displayed on the map */

class Surface {
private:
    static int idCounter; // unique id counter, change after every object creation
    int id; // unique identification of this object

    static int getUniqueId();

    std::vector<cv::Point2f> points; // Points constituting the surfaces
    cv::Point2f P1, P2; // Ends of the surface's surface
    std::vector<Color> colors; // Color areas with a width of View::step
    Color averageColor;

public:
    Surface();
    Surface(float X1, float Y1, float X2, float Y2);
    ~Surface();
    
    bool operator==(Surface const& b);

    int getId() const;
    
    
    void display() const;
    
    double length() const;
    cv::Point2f midPoint() const;

    void addPoint(cv::Point2f newPoint);
    
    // Transforming coordinates of the points into the new frame or reference centered on newCenter and with the axes being Xax*x and Yax*y
    void coordTransf(cv::Point2f newCenter, float hX, float hY); 
    void rotate(cv::Point3f Center); // Rotate P1 & P2 according to the angle Center.z
    void rotateAroundP1(double angle); //rotates P2 according to the angle.
    std::vector<cv::Point2f> getPoints() const;
    /** internally re-creates this surface, resetting the points and assigning a new id */
    void reset();
    
    std::vector<cv::Point2f> getAllPoints();
    
    //transforms and returns this surface P1 and P2 (which are in old coordinate) to a new coordinate
    //whose center and angle with respect to old coordinate frame are given.
    //from pv on to cv.
    Surface transFrom(double newX, double newY, double angle) const;
    
    Surface transformB(double newX, double newY, double angle) const;

    void set(float X1, float Y1, float X2, float Y2);
    void setP1(float X, float Y);
    void setP2(float X, float Y);
    cv::Point2f getP1() const;
    cv::Point2f getP2() const;

    void setSurfaceSimple(); //It forms a surface from first and last point
    void setSurface(); // Surface : average line of points. It is given by it's 2 ends P1 & P2
    
    double getAngleWithXaxis() const;
    double getAngleWithSurface(Surface s) const;
    
    double getAngleFromP1ToPoint(const double & a, const double & b) const;
    double getAngleFromP2ToPoint(const double & a, const double & b) const;
    
    double distFromP1ToPoint(const float & a, const float & b) const;
    double distFromP2ToPoint(const float & a, const float & b) const;
    double distFromMiddlePointToPoint(const float & a, const float & b) const;
    
    //given that angle, distance, and reference direction, find the x and y co-ordinate with respect to this surface.
    void locatePointAt(double & x, double & y, const double & ang, const double & distance, const int & refDirection);
    //given that angle, distance of two points, and reference direction, find the surface with respect to this surface.
    Surface makeASurfaceAt(const double & ang1, const double & dist1, const double & ang2, const double & dist2, const int & refDirection);
    void shiftOneEnd(const double & ang, const double & distance, const int & refDirection);

    void setColors(std::vector<Color> colors);
    std::vector<Color> getColors();
    Color getAverageColor();
    
    SurfaceT ToSurfaceT() const;
    
    void orderEndpoints(Surface robotOrientation);
    
    bool intersects(Surface other);
    
    PointXY projectPointOnSurface(double x, double y);
    
    double distFromPoint(double x, double y);
};
bool onSegment(cv::Point2f p, cv::Point2f q, cv::Point2f r);
int orientation(cv::Point2f p, cv::Point2f q, cv::Point2f r);
//
bool SortBasedOnLength(Surface surf1, Surface surf2);
void displaySurfaces(const std::vector<Surface> & surfaces);

//it transform pv to cv
vector<Surface> transform(const vector<Surface> & pvSurfaces, const double & angle, const double & distance);
//it transform cv to pv
vector<Surface> transformB(const vector<Surface> & cvSurfaces, const double & angle, const double & distance);

//convert SurfaceT to Surface
vector<Surface> convertSurfaceT2Surface(const vector<SurfaceT> & surfs);



//Reference Surface Class.

class ReferenceSurfaces {
private:
    Surface mapSurface;
    Surface viewSurface;
    int refPoint;
    
public:
    ReferenceSurfaces();
ReferenceSurfaces(const Surface & surf1, const Surface & surf2) ;
~ReferenceSurfaces() ;

    Surface getMapSurface() ;
   
    void setMapSurface(const Surface & surf1) ;
   
    
    Surface getViewSurface();
  
    void setViewSurface(const Surface & surf1) ;
    
    
    int getRefPoint() ;
  
    void setRefPoint(const int & i) ;
    
    void display();
   
};

#endif /* SURFACE_H */
