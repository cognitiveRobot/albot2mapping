/* 
 * File:   View.h
 * Author: guest
 *
 * Created on June 11, 2013, 9:58 PM
 */

#ifndef VIEW_H
#define	VIEW_H

/* ------------------------- Basic Includes ------------------------- */
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <assert.h>
#include <iostream>
#include <sstream>
#include <cmath>
#include <math.h>

/* ------------------------- Open CV includes ------------------------- */
#include <opencv/cv.h>
#include <opencv2/opencv.hpp>

/* ------------------------- Own Headers ------------------------ */
#include "Surface.h"
#include "Camera.h"
#include "Color.h"
#include "Constants.h"
#include "Robot.h"
#include "ImageProcessing.h"

/* ------------------------- Namespaces ------------------------- */
using namespace std;
//using namespace cv;

struct SortSurfacesByX {
    vector<Surface> robotSurfaces;

    SortSurfacesByX(const vector<Surface>& rbtSurfaces) {
        robotSurfaces = rbtSurfaces;
    }

    bool operator()(const Surface& a, Surface& b) {
        double angleA = robotSurfaces[0].getAngleFromP1ToPoint(a.getP1().x, a.getP1().y);
        if (angleA > 180) angleA -= 360;
        double angleB = robotSurfaces[0].getAngleFromP1ToPoint(b.getP1().x, b.getP1().y);
        if (angleB > 180) angleB -= 360;
        return angleB < angleA;
    }
};

/* View class contains :
 * Robot position and orientation
 * Vector of surfaces
 */

class View {
private:
    static const int step;

    int Id;
    int boundY, boundW, boundH; // boundX declared locally since it changes
    cv::Point3f robot;
    vector<Surface> surfaces;

    vector<Surface> landmarks;

    vector<Surface> robotSurfaces;
    
    vector<Surface> viewBoundaries;
    vector<Surface> viewExits;
    vector<Surface> surfacesInBoundaries;

    Surface rPositionInPV;

    Surface gap;
    bool hasGap;

    //contains color info of all pixels of ROI image
    Color colors[COLOR_IMAGE_WIDTH][COLOR_IMAGE_HEIGHT];

    /*Display*/
    int sizeX;
    int sizeY;

    void readColors(char* filename);
    void saveAreaColors(std::vector<Color> areaColors, const char * filename);
    void matToColorMatrix(cv::Mat mat,
            Color colors[COLOR_IMAGE_WIDTH][COLOR_IMAGE_HEIGHT]);
    void markAndSave(cv::Mat mat, const char * filename);
    void writeColors(Color colors[COLOR_IMAGE_WIDTH][COLOR_IMAGE_HEIGHT],
            const char * filename);



public:
    View();
    View(vector<Surface> someSurfaces);
    ~View();

    void setId(int value);
    int getId() const;
    void setRobotPos(float X, float Y, float angle);
    cv::Point3f getRobotPos();

    void setRPositionInPV(const Surface & surf);
    Surface getRPositionInPV();
    
     void setViewBoundaries(vector<Surface> boundaries);
    vector<Surface>  getViewBoundaries() const;
    
    void setViewExits(vector<Surface> exits);
    vector<Surface>  getViewExits() const;
    
     vector<Surface>  getSurfacesInBoundaries() const;


    void constructView(const char* filename);

    void setRobotSurfaces(const vector<Surface> & surfaces);
    vector<Surface> getRobotSurfaces() const;

    void setView(TriclopsContext triclops, TriclopsImage16 depthImage,
            cv::Point3f robotPos); // Setting View from camera photograph
    Color getAverageColor(int boundX, int boundY, int boundW, int boundH);
    Color calculateAverageColor(std::vector<Color> colors);
    float distance(cv::Point2f A, cv::Point2f B); // Get the distance between 2 points of a snapshot to know if they belong to the same surface
    void addSurface(Surface surface); //add a single surface to the existing list.
    void addSurfaces(vector<Surface> someSurfaces); //add few surfaces to the existing list.

    void setSurfaces(); // Set the surface for each Surface

    void setSurfaces(const vector<Surface> & someSurfaces);
    vector<Surface> getSurfaces() const;

    void rotate(); // Rotate each Surface according to angle robot.z
    void translate(); // Translate each Surface according to robot.x & robot.y
    void cleanView(); // Delete irrelevant Surfaces

    void clearView();

    void saveSurfaces(vector<Surface> surfaces, char * filename);

    bool markLandmarks();
    vector<Surface> getLandmarks() const;
    void setLandmarks(vector<Surface> someLandmarks);

    //old method to save view in a jpg file.
    cv::Mat display();

    //save view to a jpg file.
    void printView();

    Surface getGap() const;

    void setGap(Surface gap);

    bool getHasGap() const;

    void setHasGap(bool hasGap);

    /**
     * Select the surfaces close to the gap, cluster them and 
     * compute the main direction of each cluster using PCA (for each side)
     * @return one vector of main directions for each side
     */
    pair<vector<Surface>, vector<Surface> > computeExitBordersDirections();
    
    /**
     * Find the exits and boundaries of the view
     * Exit : 2 points where we can see further surfaces inbetween 
     * and the distance to those surfaces is higher than the exit length
     */
    void findBoundaries();
    
    void findBoundariesWithThreshold();

};

int readFolderNumber(const char* fileName);
void readALocalSpace(View & cView, const char* fileName);
void readAView(View & cView, const char* fileName);
void readOdometry(Robot & albot, const char* fileName);

View makeViewFromSurfaces(const vector<Surface> & someSurfaces);

#endif	/* VIEW_H */

