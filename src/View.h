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
#include "Obstacle.h"
#include "Camera.h"
#include "Color.h"
#include "Constants.h"
#include "ImageReader.h"

/* ------------------------- Namespaces ------------------------- */
using namespace std;
//using namespace cv;

#define DISPARITY_HEIGHT 240.
#define DISPARITY_WIDTH 320.

/* View class contains :
 * Robot position and orientation
 * Vector of obstacles
 */

class View {

private:
	static const unsigned MINIMUM_OBSTABLE_POINTS;
	static const double DEPTH_DIFF_TRESHOLD;
	static const int COLOR_DIFF_TRESHOLD;

	int Id;
	cv::Point3f robot;
	vector<Obstacle> Obst;
	Color colors[COLOR_IMAGE_WIDTH][COLOR_IMAGE_HEIGHT];

	/*Display*/
	int step;
	cv::Mat drawing;
	int sizeX;
	int sizeY;

	void readColors(char* filename);

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
	float distance(cv::Point2f A, cv::Point2f B); // Get the distance between 2 points of a snapshot to know if they belong to the same obstacle
	void addObst(Obstacle newObst);
	void setSurfaces();                 // Set the surface for each Obstacle
	void rotate();            // Rotate each Obstacle according to angle robot.z
	void translate();  // Translate each Obstacle according to robot.x & robot.y
	void cleanView();                   // Delete irrelevant Obstacles
	vector<Obstacle> getObsts();
	void clearView();

	void saveSurfaces(vector<Obstacle> obstacles, char * filename);

	cv::Mat display();

};

#endif	/* VIEW_H */

