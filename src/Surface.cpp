#include "Surface.h"

/* Functions definitions for Surfaces class */

Surface::Surface() {
	this->id = Surface::getUniqueId();
}
Surface::~Surface() {

}

int Surface::idCounter = 0;

int Surface::getUniqueId() {
	return Surface::idCounter++;
}

void Surface::addPoint(cv::Point2f newPoint) {
	points.push_back(newPoint);
}

void Surface::coordTransf(cv::Point2f newCenter, float hX, float hY) {

	for (unsigned int i = 0; i < points.size(); i++) {

		// Change the center of the frame reference
		points[i].x -= newCenter.x;
		points[i].y -= newCenter.y;

		// Homothetic transformation to adapt to axes
		points[i].x *= hX;
		points[i].y *= hY;

	}

}

void Surface::rotate(cv::Point3f Center) {
	float distance;
	float teta, angle;

	teta = 0;
	angle = Center.z;
	angle *= M_PI;
	angle /= (float) 180;

	P1.x = P1.x;
	P1.y = P1.y;

	distance = sqrt(P1.x * P1.x + P1.y * P1.y);  // Getting distance from center

	if (P1.x < 0) {
		teta = atan(P1.y / P1.x) + M_PI;
	} else
		teta = atan(P1.y / P1.x);

	// Set angle for rotated point
	teta += angle;

	// Set new position
	P1.x = distance * cos(teta);
	P1.y = distance * sin(teta);

	P2.x = P2.x;
	P2.y = P2.y;

	distance = sqrt(P2.x * P2.x + P2.y * P2.y);  // Getting distance from center

	if (P2.x < 0) {
		teta = atan(P2.y / P2.x) + M_PI;
	} else
		teta = atan(P2.y / P2.x);

	// Set angle for rotated point
	teta += angle;

	// Set new position
	P2.x = distance * cos(teta);
	P2.y = distance * sin(teta);

}

std::vector<cv::Point2f> Surface::getPoints() {
	return points;
}

void Surface::clearPoints() {
	points.clear();
}

void Surface::setP1(float X, float Y) {
	P1.x = X;
	P1.y = Y;
}

void Surface::setP2(float X, float Y) {
	P2.x = X;
	P2.y = Y;
}

cv::Point2f Surface::getP1() {
	return P1;
}

cv::Point2f Surface::getP2() {
	return P2;
}

void Surface::setSurface() {

	cv::Vec4f vecLine;                          // Orientation vector
	std::vector < cv::Point2f > myPoints = points;
	float distance = 0;                     // Size of the surface

	/* Set surface size by measuring the longest distance between 2 points */
	for (unsigned i = 0; i < myPoints.size(); i++) {
		for (unsigned j = 0; j < myPoints.size(); j++) {
			if (distance
					< sqrt(
							(myPoints[i].x - myPoints[j].x)
									* (myPoints[i].x - myPoints[j].x)
									+ (myPoints[i].y - myPoints[j].y)
											* (myPoints[i].y - myPoints[j].y)))
				distance = sqrt(
						(myPoints[i].x - myPoints[j].x)
								* (myPoints[i].x - myPoints[j].x)
								+ (myPoints[i].y - myPoints[j].y)
										* (myPoints[i].y - myPoints[j].y));
		}
	}

	/* Set the orientation vector to fit with the set of points*/
	cv::fitLine(myPoints, vecLine, CV_DIST_L2, 0, 0.01, 0.01);

	/* Set the ends of the surface */
	setP1(myPoints[0].x, myPoints[0].y);
	setP2(distance * vecLine[0] + P1.x, distance * vecLine[1] + P1.y);

}

void Surface::setColors(std::vector<Color> colors) {
	this->colors = colors;
	this->averageColor = Color::calculateAverageColor(this->colors);
}

Color Surface::getAverageColor() {
	return this->averageColor;
}
