//#include <bits/stl_vector.h>
#include <fstream>
#include "View.h"

const unsigned View::MINIMUM_OBSTABLE_POINTS = 3;
const double View::DEPTH_DIFF_TRESHOLD = 0.15;
const int View::COLOR_DIFF_TRESHOLD = 100;

View::View() {
	Id = 0;

	step = DISPARITY_WIDTH / 100;
	boundW = step;
	boundH = 25;
	boundY = DISPARITY_HEIGHT / 2;

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
	Surface tmpObst = Surface();
	double avgZ = 0;
	double preAvgZ = 0;
	int nbPoints = 0, n = 0;
	int disparity;
	float x, y, z, leftX, rightX;
	int count = 0;
	Color prevAreaColor;
	std::vector<Color> areaColors;
	std::vector<Color> obstColors;

	// read colors
	char filename[50];
	sprintf(filename, "%s%d%s", "../outputs/color/color-", Id, ".ppm");
	cv::Mat mat = cv::imread(filename);
	sprintf(filename, "%s%d%s", "../outputs/color/colorarea-", Id, ".jpg");
	markAndSave(mat, filename);
	matToColorMatrix(mat, this->colors);

	for (int boundX = 0; boundX < DISPARITY_WIDTH - step; boundX += step) { //For each slice of the image
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

		Color areaColor = getAverageColor(boundX, boundY, boundW, boundH);
		areaColors.push_back(areaColor);
//		printf("Avg color for (x=%d, w=%d, y=%d, h=%d): r%d g%d b%d\n", boundX,
//				boundY, boundW, boundH, areaColor.red, areaColor.green,
//				areaColor.blue);

		/* Set the new points for the View */
		cv::Point2f newPoint(boundX + 1 / 2, avgZ);

		int totalColorDiff = 0;
		if (boundX > 0) { // not the first slice
			int redDiff = abs(areaColor.red - prevAreaColor.red);
			int greenDiff = abs(areaColor.green - prevAreaColor.green);
			int blueDiff = abs(areaColor.blue - prevAreaColor.blue);
			totalColorDiff = redDiff + greenDiff + blueDiff;
		}
		if (abs(avgZ - preAvgZ) < DEPTH_DIFF_TRESHOLD // Close enough to previous depth value
		&& totalColorDiff < COLOR_DIFF_TRESHOLD) { // and close enough to previous color
			tmpObst.addPoint(newPoint); // Consider it belongs to same Surface
			obstColors.push_back(areaColor);
		} else { // new surface
			cv::Point2f Center((float) DISPARITY_WIDTH / 2, 0.15);
			tmpObst.coordTransf(Center, sizeX / DISPARITY_WIDTH, 100); // Adapt the coordinates
			Color obstColor = Color::calculateAverageColor(obstColors);
//			printf("Surface color: r%d g%d b%d\n", obstColor.red,
//					obstColor.green, obstColor.blue);
			tmpObst.setColor(obstColor);
			addObst(tmpObst); 				// Add the Surface to the View
			tmpObst.clearPoints(); 			// Clear the temporary Surface
			obstColors.clear(); 			// clear colors
			tmpObst.addPoint(newPoint);
			obstColors.push_back(areaColor);
		}
		preAvgZ = avgZ;
		prevAreaColor = areaColor;
	}

	// save area colors
	sprintf(filename, "%s%d%s", "../outputs/color/areacolors-", Id, ".jpg");
	saveAreaColors(areaColors, filename);

	//save surfaces like laser
	sprintf(filename, "%s%d", "../outputs/surfaces/surfaces-", Id);
	saveSurfaces(Obst, filename);

	// Update the position of the robot for this new view
	robot.x = robotPos.x;
	robot.y = robotPos.y;
	robot.z = robotPos.z;

}

Color View::getAverageColor(int boundX, int boundY, int boundW, int boundH) {
	vector<Color> areaColors;
	for (int x = boundX; x < boundX + boundW; x++) {
		for (int y = boundY; y < boundY + boundH; y++) {
			areaColors.push_back(this->colors[x][y]);
		}
	}
	Color avgColor = Color::calculateAverageColor(areaColors);
//	printf(
//			"Built Avg color for (x=%d, w=%d, y=%d, h=%d) - %d pixels: r%d g%d b%d\n",
//			boundX, boundY, boundW, boundH, areaColors.size(), avgColor.red,
//			avgColor.green, avgColor.blue);
	return avgColor;
}

void View::setSurfaces() {
	for (unsigned int i = 0; i < Obst.size(); i++) { // For each surface
		if (Obst[i].getPoints().size() > MINIMUM_OBSTABLE_POINTS) // If the Surface is relevant
			Obst[i].setSurface(); // Set surface
	}
}

void View::rotate() {
	for (unsigned int i = 0; i < Obst.size(); i++) { // For each surface
		Obst[i].rotate(robot); // Rotate according to robot angle
	}
}

void View::translate() {
	/* Translate the ends P1 & P2 of each Surface according to robot position */
	for (unsigned int i = 0; i < Obst.size(); i++) {
		Obst[i].setP1((float) Obst[i].getP1().x + robot.x / 10,
				(float) Obst[i].getP1().y + robot.y / 10);
		Obst[i].setP2((float) Obst[i].getP2().x + robot.x / 10,
				(float) Obst[i].getP2().y + robot.y / 10);
	}
}

void View::cleanView() {
	vector<Surface> tmpObsts;

	for (unsigned int i = 0; i < Obst.size(); i++) { // For each surface
		if (Obst[i].getPoints().size() > 4) { // If there is enough points in 1 surface...
			tmpObsts.push_back(Obst[i]);
		}
	}
	Obst.clear();
	Obst = tmpObsts;
}

void View::addObst(Surface newObst) {
	Obst.push_back(newObst);
}

vector<Surface> View::getObsts() {
	return Obst;
}

void View::clearView() {
	Obst.clear();
}

cv::Mat View::display() {
	vector<Surface> tmp1Obst;  // Temporary surfaces vector to update Obst
	cv::Mat drawing = cv::Mat::zeros(cv::Size(sizeX, sizeY), CV_8UC3);
	drawing.setTo(cv::Scalar(255, 255, 255));
	cv::Point2f P;
	Surface curObst;

	// Draw the robot
	cv::Point2f rbt0(sizeX / 2, sizeY - 15);
	cv::Point2f rbt1(rbt0.x - 15, rbt0.y - 15);
	cv::Point2f rbt2(rbt0.x + 15, rbt0.y + 15);
	cv::Point2f rbt3(rbt0.x, rbt0.y - 30);
	cv::rectangle(drawing, rbt1, rbt2, cv::Scalar(0, 0, 255), 3, 8, 0);
	cv::line(drawing, rbt0, rbt3, cv::Scalar(0, 0, 0), 2, 8, 0);

	for (unsigned int i = 0; i < Obst.size(); i++) {
		curObst = Obst[i]; // Transform the coordinates to have the right frame of reference

		if (curObst.getPoints().size() > 4) { // If there is enough points in 1 surface...
			tmp1Obst.push_back(Obst[i]);
		}
	}
	Obst.clear();
	Obst = tmp1Obst;

	// Draw the surfaces
	cout << Obst.size() << " Surfaces in this view" << endl;
	unsigned int numline = 0;
	for (unsigned int i = 0; i < Obst.size(); i++) { // For each surface
		curObst = Obst[i]; // Transform the coordinates to have the right frame of reference

		if (curObst.getPoints().size() > 4) { // If there is enough points in 1 surface...
			curObst.setSurface();        // Construct a line with the points
			// Adapt the point to the openCV Mat drawing
			curObst.setP1(rbt0.x + curObst.getP1().x,
					rbt0.y - curObst.getP1().y);
			curObst.setP2(rbt0.x + curObst.getP2().x,
					rbt0.y - curObst.getP2().y);

			cv::line(drawing, curObst.getP1(), curObst.getP2(),
					cv::Scalar(curObst.color.red, curObst.color.green,
							curObst.color.blue), 3, 8, 0); // Draw that line
			numline++;

		}

		// Draw the points
		//cout << curObst.getPoints().size() << "Surfaces points in this view" << endl;
		for (unsigned int j = 0; j < curObst.getPoints().size(); j++) {
			P.x = rbt0.x + curObst.getPoints()[j].x;
			P.y = rbt0.y - curObst.getPoints()[j].y;
			cv::circle(drawing, P, 1, cv::Scalar(255, 0, 0), 1, 8, 0);
		}

	}
	cout << numline << " Number of lines in this view" << endl;

	// Display the view in a file
	char filename[50];
	sprintf(filename, "%s%d%s", "../outputs/Views/View", Id, ".jpg");
	cv::imwrite(filename, drawing);

	return drawing;
}

void View::saveSurfaces(vector<Surface> surfaces, char * filename) {
	cout << "Saving surface file" << endl;
	ofstream outFile(filename, ios::out);

	// Output ASCII header (row and column)
	outFile << surfaces.size() << " " << 4 << endl;

	// 8 digits should be more than enough
	// outFile << fixed;
	//outFile.precision(10);

	for (int i = 0; i < int(surfaces.size()); i++) {
		//outFile << surfaces[i].getID() << " ";
		if (surfaces[i].getPoints().size() > 4) {
			outFile << surfaces[i].getPoints()[0].x << " ";
			outFile << surfaces[i].getPoints()[0].y << " ";
			outFile
					<< surfaces[i].getPoints()[surfaces[i].getPoints().size()
							- 1].x << " ";
			outFile
					<< surfaces[i].getPoints()[surfaces[i].getPoints().size()
							- 1].y << endl;
		}
	}

	outFile.close();
	cout << "Surface file saved" << endl;
}

void View::matToColorMatrix(cv::Mat img,
		Color colors[COLOR_IMAGE_WIDTH][COLOR_IMAGE_HEIGHT]) {
	int width = img.cols, height = img.rows, channels = img.channels();
	for (int y = 0; y < height; y++) {
		for (int x = 0; x < width; x++) {
			cv::Vec3b pixel = img.at < cv::Vec3b > (y, x * channels);
			int blue = pixel[0];
			int green = pixel[1];
			int red = pixel[2];
			colors[x][y].setRGB(red, green, blue);
		}
	}
}

void View::markAndSave(cv::Mat mat, const char * filename) {
	// crop
	cv::Rect bounds(0, boundY, DISPARITY_WIDTH, boundH);
	cv::Mat boundedMat = mat(bounds);
	// draw lines
	cv::line(boundedMat, cv::Point2f(0, boundY),
			cv::Point2f(DISPARITY_WIDTH, boundY), cv::Scalar(255, 255, 0), 2);
	cv::line(boundedMat, cv::Point2f(0, boundY + boundH),
			cv::Point2f(DISPARITY_WIDTH, boundY + boundH),
			cv::Scalar(255, 255, 0), 2);
	// save
	imwrite(filename, boundedMat);
}

void View::saveAreaColors(std::vector<Color> areaColors,
		const char * filename) {
	cv::Mat mat = cv::Mat::zeros(cv::Size(DISPARITY_WIDTH, boundH), CV_8UC3);
	mat.setTo(cv::Scalar(255, 255, 255));

	// Draw the area colors
	int boundX = 0;
	for (std::vector<Color>::iterator colorptr = areaColors.begin();
			colorptr != areaColors.end(); ++colorptr) {
		cv::Rect rect(boundX, 0, boundW, boundH);
		cv::rectangle(mat, rect,
				cv::Scalar(colorptr->red, colorptr->green, colorptr->blue),
				CV_FILLED);
		boundX += step;
	}

	// Display the view in a file
	imwrite(filename, mat);

	printf("Saved area colors\n");
}

