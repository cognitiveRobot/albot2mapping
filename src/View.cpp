//#include <bits/stl_vector.h>
#include <iostream>
#include <fstream>
#include "View.h"
#include "Constants.h"
#include "Printer.h"
#include "Output.h"

//const unsigned View::MINIMUM_SURFACE_POINTS = 2;
//const double View::DEPTH_DIFF_TRESHOLD = 0.15;
//const int View::COLOR_DIFF_TRESHOLD = 100;
const int View::step = DISPARITY_WIDTH / 100;

View::View() {
    Id = 0;

    boundW = step;
    boundH = 25;
    boundY = DISPARITY_HEIGHT / 2;

    sizeX = 500;
    sizeY = 500;
}

View::View(vector<Surface> someSurfaces) {
    surfaces = someSurfaces;

    //for display view size.
    sizeX = 500;
    sizeY = 500;
}

View::~View() {

}

void View::setId(int value) {
    Id = value;
}

int View::getId() const {
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

void View::setRobotSurfaces(const vector<Surface> & surfaces) {
    this->robotSurfaces = surfaces;
}

vector<Surface> View::getRobotSurfaces() const {
    return robotSurfaces;
}

void View::setView(TriclopsContext triclops, TriclopsImage16 depthImage,
        cv::Point3f robotPos) {
    clearView();
    cout << endl << "Getting view from image" << endl;

    cv::RNG rng(time(NULL));
    Surface tmpSurface;
    double avgZ = 0;
    double preAvgZ = 0;
    int nbPoints = 0, n = 0;
    int disparity;
    float x, y, z, leftX, rightX;
    int count = 0;
    Color prevAreaColor;
    std::vector<Color> areaColors;
    std::vector<Color> surfaceColors;

    // read colors
    char filename[50];
    sprintf(filename, "%s%d%s", "../outputs/color/color-", Id, ".ppm");
    cv::Mat mat = cv::imread(filename);
    sprintf(filename, "%s%d%s", "../outputs/color/colorarea-", Id, ".ppm");
    markAndSave(mat, filename);
    matToColorMatrix(mat, this->colors); //saving color info for the whole image.

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
            tmpSurface.addPoint(newPoint); // Consider it belongs to same Surface
            surfaceColors.push_back(areaColor);
        } else { // new surface
            cv::Point2f Center((float) DISPARITY_WIDTH / 2, 0.15);
            tmpSurface.coordTransf(Center, sizeX / DISPARITY_WIDTH, 100); // Adapt the coordinates
            Color surfaceColor = Color::calculateAverageColor(surfaceColors);
            //			printf("Surface color: r%d g%d b%d\n", surfaceColor.red,
            //					surfaceColor.green, surfaceColor.blue);
            tmpSurface.setColors(surfaceColors);
            if (tmpSurface.getPoints().size() > 1) {
                tmpSurface.setSurfaceSimple();
                addSurface(tmpSurface); // Add the Surface to the View
            }
            tmpSurface.reset(); // Clear the temporary Surface
            surfaceColors.clear(); // clear colors
            tmpSurface.addPoint(newPoint);
            surfaceColors.push_back(areaColor);
        }
        preAvgZ = avgZ;
        prevAreaColor = areaColor;
    }

    // save area colors
    sprintf(filename, "%s%d%s", "../outputs/color/areacolors-", Id, ".jpg");
    saveAreaColors(areaColors, filename);

    //save surfaces like laser
    sprintf(filename, "%s%d", "../outputs/surfaces/surfaces-", Id);
    saveSurfaces(this->surfaces, filename);

    //setting surfaces
    setSurfaces();

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
    for (unsigned int i = 0; i < this->surfaces.size(); i++) { // For each surface
        if (this->surfaces[i].getPoints().size() >= MINIMUM_SURFACE_POINTS) // If the Surface is relevant
            this->surfaces[i].setSurface(); // Set surface
    }
}

void View::setSurfaces(const vector<Surface> & someSurfaces) {
    this->surfaces = someSurfaces;
}

void View::rotate() {
    for (unsigned int i = 0; i < this->surfaces.size(); i++) { // For each surface
        this->surfaces[i].rotate(robot); // Rotate according to robot angle
    }
}

void View::translate() {
    /* Translate the ends P1 & P2 of each Surface according to robot position */
    for (unsigned int i = 0; i < this->surfaces.size(); i++) {
        this->surfaces[i].setP1(
                (float) this->surfaces[i].getP1().x + robot.x / 10,
                (float) this->surfaces[i].getP1().y + robot.y / 10);
        this->surfaces[i].setP2(
                (float) this->surfaces[i].getP2().x + robot.x / 10,
                (float) this->surfaces[i].getP2().y + robot.y / 10);
    }
}

void View::cleanView() {
    vector<Surface> tmpSurfaces;

    for (unsigned int i = 0; i < this->surfaces.size(); i++) { // For each surface
        if (this->surfaces[i].getPoints().size() >= MINIMUM_SURFACE_POINTS) { // If there is enough points in 1 surface...
            tmpSurfaces.push_back(this->surfaces[i]);
        }
    }
    this->surfaces.clear();
    this->surfaces = tmpSurfaces;
}

void View::addSurface(Surface surface) {
    this->surfaces.push_back(surface);
}

void View::addSurfaces(vector<Surface> someSurfaces) {
    for (unsigned int i = 0; i < someSurfaces.size(); i++) {
        this->surfaces.push_back(someSurfaces[i]);
    }
}

//const is must. bcz it needs to call from a const object

vector<Surface> View::getSurfaces() const {
    return this->surfaces;
}

void View::clearView() {
    this->surfaces.clear();
}

cv::Mat View::display() {
    vector<Surface> tmp1Surfaces; // Temporary surfaces vector to update this->surfaces
    cv::Mat drawing = cv::Mat::zeros(cv::Size(sizeX, sizeY), CV_8UC3);
    drawing.setTo(cv::Scalar(255, 255, 255));
    cv::Point2f P;
    Surface currSurface;

    // Draw the robot
    cv::Point2f rbt0(sizeX / 2, sizeY - 15);
    cv::Point2f rbt1(rbt0.x - 15, rbt0.y - 15);
    cv::Point2f rbt2(rbt0.x + 15, rbt0.y + 15);
    cv::Point2f rbt3(rbt0.x, rbt0.y - 30);
    cv::rectangle(drawing, rbt1, rbt2, cv::Scalar(0, 0, 255), 3, 8, 0);
    cv::line(drawing, rbt0, rbt3, cv::Scalar(0, 0, 0), 2, 8, 0);

    for (unsigned int i = 0; i < this->surfaces.size(); i++) {
        currSurface = this->surfaces[i]; // Transform the coordinates to have the right frame of reference

        if (currSurface.getPoints().size() >= MINIMUM_SURFACE_POINTS) { // If there is enough points in 1 surface...
            tmp1Surfaces.push_back(this->surfaces[i]);
        }
    }
    this->surfaces.clear();
    this->surfaces = tmp1Surfaces;

    // Draw the surfaces
    cout << this->surfaces.size() << " Surfaces in this view" << endl;
    unsigned int numline = 0;
    for (unsigned int i = 0; i < this->surfaces.size(); i++) { // For each surface
        currSurface = this->surfaces[i]; // Transform the coordinates to have the right frame of reference

        if (currSurface.getPoints().size() >= MINIMUM_SURFACE_POINTS) { // If there is enough points in 1 surface...
            currSurface.setSurface(); // Construct a line with the points
            // Adapt the point to the openCV Mat drawing
            currSurface.setP1(rbt0.x + currSurface.getP1().x,
                    rbt0.y - currSurface.getP1().y);
            currSurface.setP2(rbt0.x + currSurface.getP2().x,
                    rbt0.y - currSurface.getP2().y);

            cv::line(drawing, currSurface.getP1(), currSurface.getP2(),
                    cv::Scalar(currSurface.getAverageColor().red,
                    currSurface.getAverageColor().green,
                    currSurface.getAverageColor().blue), 3, 8, 0); // Draw that line
            numline++;

        }

        // Draw the points
        //cout << currSurface.getPoints().size() << "Surfaces points in this view" << endl;
        for (unsigned int j = 0; j < currSurface.getPoints().size(); j++) {
            P.x = rbt0.x + currSurface.getPoints()[j].x;
            P.y = rbt0.y - currSurface.getPoints()[j].y;
            cv::circle(drawing, P, 1, cv::Scalar(255, 0, 0), 1, 8, 0);
        }

    }
    cout << numline << " Number of lines in this view" << endl;

    //    //draw all landmarks
    for (unsigned int i = 0; i < this->landmarks.size(); i++) { // For each surface
        currSurface.setP1(rbt0.x + landmarks[i].getP1().x,
                rbt0.y - landmarks[i].getP1().y);
        currSurface.setP2(rbt0.x + landmarks[i].getP2().x,
                rbt0.y - landmarks[i].getP2().y);
        cv::line(drawing, currSurface.getP1(), currSurface.getP2(),
                cv::Scalar(0, 0, 255), 3, 8, 0); // Draw that line
    }
    cout << "Number of landmarks in this view " << this->landmarks.size() << endl;

    // Display the view in a file
    char filename[50];
    sprintf(filename, "%s%d%s", "../outputs/Views/View", Id, ".jpg");
    cv::imwrite(filename, drawing);

    return drawing;
}

void View::printView() {
    Printer printer;
    char filename[50];
    sprintf(filename, "%s%d%s", "../outputs/Views/View", Id, ".jpg");
    printer.printView(filename, *this);
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
        if (surfaces[i].getPoints().size() >= MINIMUM_SURFACE_POINTS) {
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
    cv::imwrite(filename, boundedMat);
}

//it saves average colors of all slices 

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
    cv::imwrite(filename, mat);

    printf("Saved area colors\n");
}



//it finds landmark surfaces from the surface list.

bool View::markLandmarks() {
    cout << endl << endl << "Marking landmarks" << endl;
    if (this->surfaces.size() < 1) {
        cout << "Strange: there is no surfaces in the view. Check whether surfaces has been set. " << endl;
        return false;
    } else {

        Surface curRobotPose(0, 0, 0, 300); //Current Robot facing direction. +Y axis.
        Surface surfDirection;
        double dirAngle;
        Surface tempSurf;
        landmarks.clear(); //remove previous view landmarks.
        for (unsigned int i = 0; i<this->surfaces.size(); i++) {
            tempSurf = this->surfaces[i];
            tempSurf.setSurfaceSimple();
            surfDirection.set(0, 0, tempSurf.midPoint().x, tempSurf.midPoint().y);
            // surfDirection.display();
            dirAngle = abs(curRobotPose.getAngleWithSurface(surfDirection));
            cout << "dirAngle: " << dirAngle << endl;
            //cout << "Dist: " << surfDirection.length() << endl;

            if (dirAngle > LANDMARK_DIRECTION)//&& surfDirection.length() < LANDMARK_DISTANCE)
                landmarks.push_back(tempSurf);
            else {
                surfDirection.set(0, 0, tempSurf.getP1().x, tempSurf.getP1().y);
                dirAngle = abs(curRobotPose.getAngleWithSurface(surfDirection));
                if (dirAngle > LANDMARK_DIRECTION)//&& surfDirection.length() < LANDMARK_DISTANCE)
                    landmarks.push_back(tempSurf);
                else {
                    surfDirection.set(0, 0, tempSurf.getP2().x, tempSurf.getP2().y);
                    dirAngle = abs(curRobotPose.getAngleWithSurface(surfDirection));
                    if (dirAngle > LANDMARK_DIRECTION)//&& surfDirection.length() < LANDMARK_DISTANCE)
                        landmarks.push_back(tempSurf);
                }
            }

        }
        cout << "Landmarks: " << landmarks.size() << endl;
        if (landmarks.size() > 0)
            return true;
        else
            return false;

    }
}

vector<Surface> View::getLandmarks() const {
    return landmarks;
}

void View::setLandmarks(vector<Surface> someLandmarks) {
    landmarks = someLandmarks;
}

void View::setRPositionInPV(const Surface & surf) {
    rPositionInPV = surf;
}

Surface View::getRPositionInPV() {
    return rPositionInPV;
}

void View::constructView(const char* filename) {
            
        vector<PointXY> points2D = readASCIIPoints2D(filename);
        
        char viewName[50];
        sprintf(viewName, "%s%d%s", "../outputs/pointCloud/points2D-", this->getId(),".png");
        plotPointsAndSurfacesGNU(viewName,points2D,this->getRobotSurfaces());
        
        ImageProcessing imgTool;
        //vizualize point cloud.
        sprintf(viewName, "%s%d%s", "../outputs/pointCloud/pointCloud-", this->getId(),".pcd");
        //imgTool.visualizePointCloud(viewName);
       imgTool.segEuclideanClusters(viewName); 
       //imgTool.segRegionGrowing(viewName);
}

int readFolderNumber(const char* fileName) {
    
    ifstream inputFile(fileName, ios::in);
    int number = 0;
    if (inputFile.is_open()) {
        
        inputFile >> number;
    } else
        cout << "Error opening " << fileName << " .." << endl;
    inputFile.close();
    
    ofstream outFile(fileName, ios::out);
    if (outFile.is_open()) {
        
        outFile << number+1;
    } else
        cout << "Error opening " << fileName << " .." << endl;
    outFile.close();
    
    return number;

}

void readAView(View & cView, const char* fileName) {
    vector<Surface> surfaces;
    ifstream inputFile(fileName, ios::in);
    if (inputFile.is_open()) {
        cout << "Reading view...." << endl;
        float x1, y1, x2, y2;
        string data;

        inputFile >> data; //for row
        inputFile >> data; //for column

        inputFile >> x1;
        inputFile >> y1;
        inputFile >> x2;
        inputFile >> y2;

        while (!inputFile.eof()) {
            surfaces.push_back(Surface(x1, y1, x2, y2));
            //cout << "x " << x1 << " y " << y1 << " x2: " << x2 << " y2: " << y2 << endl;

            inputFile >> x1;
            inputFile >> y1;
            inputFile >> x2;
            inputFile >> y2;
        }
    } else
        cout << "Error opening " << fileName << " .." << endl;

    cView.setSurfaces(surfaces);
    cout << "View is read. Num of surfaces: " << surfaces.size() << endl;
}

void readOdometry(Robot & albot, const char* fileName) {

    ifstream inputFile(fileName, ios::in);
    if (inputFile.is_open()) {
        cout << "Reading odo...." << endl;
        float dist, angle;
        string data;

        inputFile >> data; //for row
        inputFile >> data; //for column

        inputFile >> dist;
        inputFile >> angle;
        
        albot.setLastLocomotion(dist,angle);
        cout << "Odometry is read. Dist: " << dist <<" angle: " << angle << endl;
        
    } else
        cout << "Error opening " << fileName << " .." << endl;
    
}

void readALocalSpace(View & cView, const char* fileName) {
    vector<Surface> surfaces;
    ifstream inputFile(fileName, ios::in);
    if (inputFile.is_open()) {
        cout << "Reading view...." << endl;
        float x1, y1, x2, y2;
        string data;



        while (!inputFile.eof()) {

            inputFile >> data;
            if (data.compare("@CPosition:") == 0) {
                //reading odometry information
                inputFile >> x1;
                inputFile >> y1;
                inputFile >> x2;
                inputFile >> y2;
                cView.setRPositionInPV(Surface(x1, y1, x2, y2));
                //cout << "rPosition -- x: " << x1 << " y: " << y1 << " x2: " << x2 << " y2: " << y2 << endl;
            }
            if (data.compare("@AllSurfaces:") == 0) {
                //reading odometry information
                inputFile >> x1;
                inputFile >> y1;
                inputFile >> x2;
                inputFile >> y2;

                while (!inputFile.eof()) {
                    surfaces.push_back(Surface(x1, y1, x2, y2));
                    //cout << "x " << x1 << " y " << y1 << " x2: " << x2 << " y2: " << y2 << endl;

                    inputFile >> x1;
                    inputFile >> y1;
                    inputFile >> x2;
                    inputFile >> y2;
                }
            }
        }


    } else
        cout << "Error opening " << fileName << " .." << endl;

    cView.setSurfaces(surfaces);
    cout << "View is read. Num of surfaces: " << surfaces.size() << endl;
    //return surfaces;
}