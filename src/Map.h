/* 
 * File:   Mapper.h
 * Author: Guillaume Diallo-Mulliez
 *
 * Created on May 30, 2013, 1:22 AM
 */

#ifndef MAPPER_H
#define	MAPPER_H

#include "View.h"

/* ------------------------- Namespaces ------------------------- */
using namespace std;
//using namespace cv;

#define DISPARITY_HEIGHT 240.
#define DISPARITY_WIDTH 320.

class Map {

private:

	View currentView;
	vector<Surface> surfaces;
	vector<cv::Point3f> rbtPos;
	cv::Mat drawing;
	int sizeX;
	int sizeY;
	int M;

public:

	Map(int _sizeX, int _sizeY);
	~Map();

	void update(View newView);        // Update the map according to the newView
	bool isBehind(Surface Old, Surface New, cv::Point3f rbtPos); /* NOT SURE IF DONE CORRECTLY */ // Check if Old and New surfaces are concealing each other
	void coordTransf(cv::Point3f *target, cv::Point3f newCenter, double hX,
			double hY); // Transform the coordinates of *target with newCenter and homothetic transformation in X & Y directions
	void rotate(cv::Point2f* target, cv::Point2f Center, float angle); // Rotate *target point of angle around Center
	void display();                                // Display map in output file
	View getView();
};

#endif	/* MAPPER_H */

