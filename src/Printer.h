#ifndef PRINTER_H
#define PRINTER_H

/* ------------------------- Basic Includes ------------------------- */

#include <iostream>

#include <cmath>
#include <vector>


/* ------------------------- Open CV includes ------------------------- */
#include <opencv/cv.h>
#include <opencv2/opencv.hpp>

#include "Surface.h"
#include "View.h"
#include "Constants.h"


class Printer {
private:
    int sizeX;
    int sizeY;
    
    cv::Mat canvas;
    
public:
    Printer();
    ~Printer();
    
    cv::Mat getCanvas();
    
    void plotPoint(const cv::Point2f & point);
    void plotPoints(const std::vector<cv::Point2f>& points);
    void plotLine(const cv::Point2f & point1, const cv::Point2f & point2, const Color & color);
    void plotRobot();
    void plotLines( const std::vector<Surface> & surfaces, const Color & color);
    
    void printSurfaces(const char * filename, const std::vector<Surface>& surfaces);
    
    void printView(const char* filename, const View & aView);
    
};



#endif /* PRINTER_H */