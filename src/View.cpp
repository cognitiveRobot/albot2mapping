//#include <bits/stl_vector.h>
#include <fstream>
#include "View.h"

/* Functions definitions for View class */

const unsigned View::MINIMUM_OBSTABLE_POINTS = 3;

View::View() {
	Id = 0;
	step = DISPARITY_WIDTH / 100;
	sizeX = 500;
	sizeY = 500;
}

View::~View() {

}

void View::setId(int value) {
	Id = value;
}
int View::getId() {
	return Id;
}

void View::setRobotPos(float X, float Y, float angle) {
	robot.x = X;
	robot.y = Y;
	robot.z = angle;
}

cv::Point3f View::getRobotPos() {
	return robot;
}

void View::setView(TriclopsContext triclops, TriclopsImage16 depthImage,
		cv::Point3f robotPos) {
	clearView();
	cout << endl << "Getting view from image" << endl;

	cv::RNG rng(time (NULL));
	Obstacle tmpObst = Obstacle();
	double avgZ = 0;
	double preAvgZ = 0;
	int nbPoints = 0, n = 0;
	int disparity;
	float x, y, z, leftX, rightX;
	int boundX, boundY, boundW, boundH;
	boundW = step;
	boundH = 25;
	boundY = DISPARITY_HEIGHT / 2;
	int count = 0;
	std::vector<Color> obstColors;

	// read colors
	char filenameColor[50];
	sprintf(filenameColor, "%s%d%s", "../outputs/color/color-", Id, ".ppm");
	ImageReader imageReader;
	imageReader.readPPM(filenameColor, this->colors);

	for (boundX = 0; boundX < DISPARITY_WIDTH - step; boundX += step) { //For each slices of the image
		count++;
		avgZ = 0;
		n = 0;
		nbPoints = 0;

		disparity =
				depthImage.data[boundY * depthImage.rowinc + boundX + boundW];
		triclopsRCD16ToXYZ(triclops, boundY, boundX, disparity, &x, &y, &z);
		leftX = x;
		triclopsRCD16ToXYZ(triclops, boundY, boundX + boundW, disparity, &x, &y,
				&z);
		rightX = x;

		/* Get the average depth value avgZ */
		for (int j = 0; j < boundH; j++) {
			for (int k = 0; k < boundW; k++) {

				disparity = depthImage.data[(j + boundY) * depthImage.rowinc / 2
						+ (k + boundX)];
				if (disparity != 0 && disparity < 65280 && disparity > 10) {
					triclopsRCD16ToXYZ(triclops, j + boundY, k + boundX,
							disparity, &x, &y, &z);
					if (z < 10) {
						nbPoints++;
						avgZ += z;
					}
				}
				n++;

			}
		}
		avgZ /= nbPoints;

		Color avgColor = getAverageColor(boundW, boundH);

		/* Set the new points for the View */
		cv::Point2f newPoint(boundX + 1 / 2, avgZ);

		if (avgZ == avgZ) {
			// TODO: add color distinction to clustering of objects
			if (abs(avgZ - preAvgZ) >= 0.15) { // Far enough from previous depth value
				cv::Point2f Center((float) DISPARITY_WIDTH / 2, 0.15);
				tmpObst.coordTransf(Center, sizeX / DISPARITY_WIDTH, 100); // Adapt the coordinates
				avgColor = calculateAverageColor(obstColors); // calculate obstacle color
				tmpObst.setColor(avgColor);
				addObst(tmpObst); 				// Add the Obstacle to the View
				tmpObst.clearPoints(); 			// Clear the temporary Obstacle
				obstColors.clear(); 			// clear colors
			}
			// point and color information are added independent of having a new obstacle or not
			tmpObst.addPoint(newPoint);
			obstColors.push_back(avgColor);
			preAvgZ = avgZ;
		}

	}

	//save surfaces like laser
	char sname[50];
	sprintf(sname, "%s%d", "../outputs/surfaces/surfaces-", Id);
	saveSurfaces(Obst, sname);

	// Update the position of the robot for this new view
	robot.x = robotPos.x;
	robot.y = robotPos.y;
	robot.z = robotPos.z;

}

Color View::getAverageColor(int boundW, int boundH) {
	return Color(0, 0, 0);
}

Color View::calculateAverageColor(std::vector<Color> colors) {
	long sumRed = 0, sumGreen = 0, sumBlue = 0;
	for (std::vector<Color>::iterator it = colors.begin(); it != colors.end();
			++it) {
		sumRed += it->red;
		sumGreen += it->green;
		sumBlue += it->blue;
	}
	return Color(sumRed / colors.size(), sumGreen / colors.size(),
			sumBlue / colors.size());
}

void View::setSurfaces() {
	for (unsigned int i = 0; i < Obst.size(); i++) { // For each obstacle
		if (Obst[i].getPoints().size() > MINIMUM_OBSTABLE_POINTS) // If the Obstacle is relevant
			Obst[i].setSurface(); // Set surface
	}
}

void View::rotate() {
	for (unsigned int i = 0; i < Obst.size(); i++) { // For each obstacle
		Obst[i].rotate(robot); // Rotate according to robot angle
	}
}

void View::translate() {
	/* Translate the ends P1 & P2 of each Obstacle according to robot position */
	for (unsigned int i = 0; i < Obst.size(); i++) {
		Obst[i].setP1((float) Obst[i].getP1().x + robot.x / 10,
				(float) Obst[i].getP1().y + robot.y / 10);
		Obst[i].setP2((float) Obst[i].getP2().x + robot.x / 10,
				(float) Obst[i].getP2().y + robot.y / 10);
	}
}

void View::cleanView() {
	vector<Obstacle> tmpObsts;

	for (unsigned int i = 0; i < Obst.size(); i++) { // For each obstacle
		if (Obst[i].getPoints().size() > 4) { // If there is enough points in 1 obstacle...
			tmpObsts.push_back(Obst[i]);
		}
	}
	Obst.clear();
	Obst = tmpObsts;
}

void View::addObst(Obstacle newObst) {
	Obst.push_back(newObst);
}

vector<Obstacle> View::getObsts() {
	return Obst;
}

void View::clearView() {
	Obst.clear();
}

cv::Mat View::display() {
	vector<Obstacle> tmp1Obst;   // Temporary obstacles vector to update Obst
	drawing = cv::Mat::zeros(cv::Size(sizeX, sizeY), CV_8UC3);
	drawing.setTo(cv::Scalar(255, 255, 255));
	cv::Point2f P;
	Obstacle curObst;

	// Draw the robot
	cv::Point2f rbt0(sizeX / 2, sizeY - 15);
	cv::Point2f rbt1(rbt0.x - 15, rbt0.y - 15);
	cv::Point2f rbt2(rbt0.x + 15, rbt0.y + 15);
	cv::Point2f rbt3(rbt0.x, rbt0.y - 30);
	cv::rectangle(drawing, rbt1, rbt2, cv::Scalar(0, 0, 255), 3, 8, 0);
	cv::line(drawing, rbt0, rbt3, cv::Scalar(0, 0, 0), 2, 8, 0);

	for (unsigned int i = 0; i < Obst.size(); i++) {
		curObst = Obst[i]; // Transform the coordinates to have the right frame of reference

		if (curObst.getPoints().size() > 4) { // If there is enough points in 1 obstacle...
			tmp1Obst.push_back(Obst[i]);
		}
	}
	Obst.clear();
	Obst = tmp1Obst;

	// Draw the obstacles
	cout << Obst.size() << " Obstacles in this view" << endl;
	unsigned int numline = 0;
	for (unsigned int i = 0; i < Obst.size(); i++)          // For each obstacle
			{
		curObst = Obst[i]; // Transform the coordinates to have the right frame of reference

		if (curObst.getPoints().size() > 4) { // If there is enough points in 1 obstacle...
			curObst.setSurface();            // Construct a line with the points
			// Adapt the point to the openCV Mat drawing
			curObst.setP1(rbt0.x + curObst.getP1().x,
					rbt0.y - curObst.getP1().y);
			curObst.setP2(rbt0.x + curObst.getP2().x,
					rbt0.y - curObst.getP2().y);

			cv::line(drawing, curObst.getP1(), curObst.getP2(),
					cv::Scalar(0, 0, 255), 3, 8, 0);    // Draw that line
			numline++;

//tmp1Obst.push_back(Obst[i]);
		}
//curObst = tmp1Obst[i];

		// Draw the points
//cout << curObst.getPoints().size() << "Obstacles points in this view" << endl;
		for (unsigned int j = 0; j < curObst.getPoints().size(); j++) {
			P.x = rbt0.x + curObst.getPoints()[j].x;
			P.y = rbt0.y - curObst.getPoints()[j].y;
			cv::circle(drawing, P, 1, cv::Scalar(255, 0, 0), 1, 8, 0);
		}

	}
	cout << numline << " Number of lines in this view" << endl;
//Obst.clear();
	//Obst = tmp1Obst;

	// Display the view in a file
	char filename[50];
	sprintf(filename, "%s%d%s", "../outputs/Views/View", Id, ".jpg");
	cv::imwrite(filename, drawing);

	return drawing;
}

void View::saveSurfaces(vector<Obstacle> obstacles, char * filename) {
	cout << "Saving surface file" << endl;
	ofstream outFile(filename, ios::out);

	// Output ASCII header (row and column)
	outFile << obstacles.size() << " " << 4 << endl;

	// 8 digits should be more than enough
	// outFile << fixed;
	//outFile.precision(10);

	for (int i = 0; i < int(obstacles.size()); i++) {
		//outFile << obstacles[i].getID() << " ";
		if (obstacles[i].getPoints().size() > 4) {
			outFile << obstacles[i].getPoints()[0].x << " ";
			outFile << obstacles[i].getPoints()[0].y << " ";
			outFile
					<< obstacles[i].getPoints()[obstacles[i].getPoints().size()
							- 1].x << " ";
			outFile
					<< obstacles[i].getPoints()[obstacles[i].getPoints().size()
							- 1].y << endl;
		}
	}

	outFile.close();
	cout << "Surface file saved" << endl;
}
