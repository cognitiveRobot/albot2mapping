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

/* ------------------------- Headers ------------------------ */
#include "Camera.h"

/* ------------------------- Namespaces ------------------------- */
using namespace std;
using namespace cv;

#define DISPARITY_HEIGHT 240.
#define DISPARITY_WIDTH 320.

/* Class of obstacles that are to be displayed on the map */

class Obstacle {
private:

	vector<Point2f> points;    // Points constituting the obstacles
	Point2f P1, P2;             // Ends of the obstacle's surface
	uint16_t color; 			// Color representation in RGB565

public:

	Obstacle();
	~Obstacle();

	void addPoint(Point2f newPoint);
	void coordTransf(Point2f newCenter, float hX, float hY); // Transforming coordinates of the points into the new frame or reference centered on newCenter and with the axes being Xax*x and Yax*y
	void rotate(Point3f Center); // Rotate P1 & P2 according to the angle Center.z
	vector<Point2f> getPoints();
	void clearPoints();

	void setP1(float X, float Y);
	void setP2(float X, float Y);
	Point2f getP1();
	Point2f getP2();

	void setSurface(); // Surface : average line of points. It is given by it's 2 ends P1 & P2

	void setColor(uint16_t color);
};

/* View class contains :
 * Robot position and orientation
 * Vector of obstacles
 */

class View {

private:
	static const unsigned MINIMUM_OBSTABLE_POINTS;

	int Id;
	Point3f robot;
	vector<Obstacle> Obst;

	/*Display*/
	int step;
	Mat drawing;
	int sizeX;
	int sizeY;

	void readColors(char* filename);

public:
	static const int COLOR_IMAGE_WIDTH, COLOR_IMAGE_HEIGHT;

	View();
	~View();

	void setId(int value);
	int getId();
	void setRobotPos(float X, float Y, float angle);
	Point3f getRobotPos();
	void setView(TriclopsContext triclops, TriclopsImage16 depthImage,
			Point3f robotPos);            // Setting View from camera photograph
	float distance(Point2f A, Point2f B); // Get the distance between 2 points of a snapshot to know if they belong to the same obstacle
	void addObst(Obstacle newObst);
	void setSurfaces();                 // Set the surface for each Obstacle
	void rotate();            // Rotate each Obstacle according to angle robot.z
	void translate();  // Translate each Obstacle according to robot.x & robot.y
	void cleanView();                   // Delete irrelevant Obstacles
	vector<Obstacle> getObsts();
	void clearView();

	void saveSurfaces(vector<Obstacle> obstacles, char * filename);

	Mat display();

};

#endif	/* VIEW_H */

