#ifndef OBSTACLE_H
#define OBSTACLE_H

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

/* ------------------------- Own includes ------------------------- */
#include "Color.h"

/* Class of obstacles that are to be displayed on the map */

class Obstacle {
private:
	int id; // unique identification of this object
	std::vector<cv::Point2f> points;    // Points constituting the obstacles
	cv::Point2f P1, P2;             // Ends of the obstacle's surface
	Color color;
	static int idCounter; // unique id counter, change after every object creation

public:

	Obstacle();
	~Obstacle();

	static int getUniqueId();

	void addPoint(cv::Point2f newPoint);
	void coordTransf(cv::Point2f newCenter, float hX, float hY); // Transforming coordinates of the points into the new frame or reference centered on newCenter and with the axes being Xax*x and Yax*y
	void rotate(cv::Point3f Center); // Rotate P1 & P2 according to the angle Center.z
	std::vector<cv::Point2f> getPoints();
	void clearPoints();

	void setP1(float X, float Y);
	void setP2(float X, float Y);
	cv::Point2f getP1();
	cv::Point2f getP2();

	void setSurface(); // Surface : average line of points. It is given by it's 2 ends P1 & P2

	void setColor(Color color);
};

#endif /* OBSTACLE_H */
