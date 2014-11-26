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

    int getId();
    
    
    void display() const;
    
    double length() const;
    cv::Point2f midPoint();

    void addPoint(cv::Point2f newPoint);
    void coordTransf(cv::Point2f newCenter, float hX, float hY); // Transforming coordinates of the points into the new frame or reference centered on newCenter and with the axes being Xax*x and Yax*y
    void rotate(cv::Point3f Center); // Rotate P1 & P2 according to the angle Center.z
    std::vector<cv::Point2f> getPoints() const;
    /** internally re-creates this surface, resetting the points and assigning a new id */
    void reset();
    
    //transforms and returns this surface P1 and P2 (which are in old coordinate) to a new coordinate
    //whose center and angle with respect to old coordinate frame are given.
    Surface transFrom(double newX, double newY, double angle);

    void set(float X1, float Y1, float X2, float Y2);
    void setP1(float X, float Y);
    void setP2(float X, float Y);
    cv::Point2f getP1() const;
    cv::Point2f getP2() const;

    void setSurfaceSimple(); //It forms a surface from first and last point
    void setSurface(); // Surface : average line of points. It is given by it's 2 ends P1 & P2
    
    double getAngleWithXaxis();
    double getAngleWithSurface(Surface s);
    
    double distFromP1ToPoint(const float & a, const float & b);
      double distFromP2ToPoint(const float & a, const float & b);

    void setColors(std::vector<Color> colors);
    std::vector<Color> getColors();
    Color getAverageColor();
};

//
bool SortBasedOnLength(Surface surf1, Surface surf2);
void displaySurfaces(const std::vector<Surface> & surfaces);

#endif /* SURFACE_H */
