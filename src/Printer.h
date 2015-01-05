#ifndef PRINTER_H
#define PRINTER_H

/* ------------------------- Basic Includes ------------------------- */

#include <iostream>
#include <fstream>
#include <cstdlib>
#include <cmath>
#include <vector>


/* ------------------------- Open CV includes ------------------------- */
#include <opencv/cv.h>
#include <opencv2/opencv.hpp>

#include "Surface.h"
#include "View.h"
#include "Constants.h"
#include "Map.h"
#include "PointAndSurface.h"


//gnuplot constant
static const char * GNUPLOT_PATH = "/usr/local/bin/gnuplot";
const unsigned int PLOT_RESOLUTION_X = 2400;
const unsigned int PLOT_RESOLUTION_Y = 2400;
const double PLOT_BORDER_FACTOR = 0.05; // Times the original with/height of the image



class Printer {
private:
    int sizeX;
    int sizeY;
    
    cv::Mat canvas;
    
public:
    Printer();
    ~Printer();
    
    void cleanCanvas();
    
    cv::Mat getCanvas();
    
    void plotPoint(const cv::Point2f & point);
    void plotPoints(const std::vector<cv::Point2f>& points);
    void plotLine(const cv::Point2f & point1, const cv::Point2f & point2, const Color & color);
    void plotRobot();
    void plotLines( const std::vector<Surface> & surfaces, const Color & color);
    
    void printSurfaces(const char * filename, const std::vector<Surface>& surfaces);
    
    void printView(const char* filename, const View & aView);
    
    void printMap(const char* filename, const Map & curMap);
    
};

void plotSurfacesGNU(const vector<Surface> & someSurfaces);
void plotMapGNU(const char * filename, const Map & map);
void plotViewGNU(const char * filename, const View & view);
void plotPointsGNU(const char * filename, const vector<PointXY> & points);

//write a txt file
void writeASCIIPoints2D(const char *filename, const vector<PointXY> & points);




#endif /* PRINTER_H */