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

/* ------------------------- Namespaces ------------------------- */
using namespace std;
//using namespace cv;

#define DISPARITY_HEIGHT 240.
#define DISPARITY_WIDTH 320.

/* View class contains :
 * Robot position and orientation
 * Vector of surfaces
 */

class View {

private:
	static const unsigned MINIMUM_SURFACE_POINTS;
	static const double DEPTH_DIFF_TRESHOLD;
	static const int COLOR_DIFF_TRESHOLD;
	static const int step;

	int Id;
	int boundY, boundW, boundH; // boundX declared locally since it changes
	cv::Point3f robot;
	vector<Surface> surfaces;
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
	~View();

	void setId(int value);
	int getId();
	void setRobotPos(float X, float Y, float angle);
	cv::Point3f getRobotPos();
	void setView(TriclopsContext triclops, TriclopsImage16 depthImage,
			cv::Point3f robotPos);        // Setting View from camera photograph
	Color getAverageColor(int boundX, int boundY, int boundW, int boundH);
	Color calculateAverageColor(std::vector<Color> colors);
	float distance(cv::Point2f A, cv::Point2f B); // Get the distance between 2 points of a snapshot to know if they belong to the same surface
	void addSurface(Surface surface);
	void setSurfaces();                 // Set the surface for each Surface
	void rotate();            // Rotate each Surface according to angle robot.z
	void translate();  // Translate each Surface according to robot.x & robot.y
	void cleanView();                   // Delete irrelevant Surfaces
	vector<Surface> getSurfaces();
	void clearView();

	void saveSurfaces(vector<Surface> surfaces, char * filename);

	cv::Mat display();

};

#endif	/* VIEW_H */

