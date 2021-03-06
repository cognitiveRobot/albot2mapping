#include "Printer.h"
#include "Color.h"

#include "Surface.h"


#include "View.h"
#include "Map.h"
#include "ImageProcessing.h"
#include "PointAndSurface.h"

Printer::Printer() {
    sizeX = 500;
    sizeY = 500;

    canvas = cv::Mat::zeros(cv::Size(sizeX, sizeY), CV_8UC3);

    canvas.setTo(cv::Scalar(255, 255, 255));

}

Printer::~Printer() {

}

void Printer::cleanCanvas() {
    this->canvas = cv::Mat::zeros(cv::Size(sizeX, sizeY), CV_8UC3);

    this->canvas.setTo(cv::Scalar(255, 255, 255));
}

cv::Mat Printer::getCanvas() {
    return canvas;
}

//it plots a single point in green.

void Printer::plotPoint(const cv::Point2f& point) {
    cv::circle(canvas, point, 1, cv::Scalar(0, 255, 0), 1, 8, 0);
}

void Printer::plotPoints(const std::vector<cv::Point2f>& points) {
    cv::Point2f P;
    for (unsigned int i = 0; i < points.size(); i++) {
        P.x = sizeX / 2 + points[i].x;
        P.y = sizeY - 15 - points[i].y;

        this->plotPoint(P);
    }
}

void Printer::plotLine(const cv::Point2f & point1, const cv::Point2f & point2, const Color & color) {
    cv::Point2f p1, p2;
    p1.x = sizeX / 2 + point1.x;
    p1.y = sizeY - 15 - point1.y;

    p2.x = sizeX / 2 + point2.x;
    p2.y = sizeY - 15 - point2.y;

    cv::line(canvas, p1, p2, cv::Scalar(color.red, color.green, color.blue), 2, 8, 0);
}

//it adds a rectangle at 0,0 poisition with +y facing.

void Printer::plotRobot() {
    // Draw the robot
    cv::Point2f rbt0(sizeX / 2, sizeY - 15);
    cv::Point2f rbt1(rbt0.x - 15, rbt0.y - 15);
    cv::Point2f rbt2(rbt0.x + 15, rbt0.y + 15);
    cv::Point2f rbt3(rbt0.x, rbt0.y - 30);

    cv::rectangle(canvas, rbt1, rbt2, cv::Scalar(0, 0, 255), 3, 8, 0);

    //plot a line for facing
    this->plotLine(rbt0, rbt3, Color(0, 0, 0));
}

void Printer::plotLines(const std::vector<Surface>& surfaces, const Color & color) {

    cv::Point2f point1, point2;
    for (unsigned int i = 0; i < surfaces.size(); i++) {
        //        point1.x = sizeX/2 + surfaces[i].getP1().x;
        //        point1.y = sizeY - 15 - surfaces[i].getP1().y;
        //        
        //        point2.x = sizeX/2 + surfaces[i].getP2().x;
        //        point2.y = sizeY - 15 - surfaces[i].getP2().y;

        this->plotLine(surfaces[i].getP1(), surfaces[i].getP2(), color);


    }
}

void Printer::printSurfaces(const char* filename, const std::vector<Surface>& surfaces) {
    this->plotLines(surfaces, Color(255, 0, 0));

    if (!cv::imwrite(filename, canvas)) {
        cout << BOLDRED << "Failed to write " << filename << RESET << endl;
    }
}
//

void Printer::printView(const char* filename, const View & aView) {
    this->plotRobot();

    this->plotLines(aView.getSurfaces(), Color(255, 0, 0));

    //plot points of surfaces.
    for (unsigned int i = 0; i < aView.getSurfaces().size(); i++) {
        if (aView.getSurfaces()[i].getPoints().size() >= MINIMUM_SURFACE_POINTS)
            this->plotPoints(aView.getSurfaces()[i].getPoints());
    }

    cout << "Num of cvLandmarks: " << aView.getSurfaces().size() << endl;
    displaySurfaces(aView.getSurfaces());

    cout << "Num of pvlandmarksOnCV: " << aView.getLandmarks().size() << endl;
    displaySurfaces(aView.getLandmarks());
    //plot landmarks.
    this->plotLines(aView.getLandmarks(), Color(0, 0, 255));


    if (!cv::imwrite(filename, canvas)) {
        cout << BOLDRED << "Failed to write " << filename << RESET << endl;
    }
}

//save map

void Printer::printMap(const char* filename, const Map & curMap) {
    //starting new map. so clear canvas in case it's dirty:)
    this->cleanCanvas();
    vector<View> allViews = curMap.getMap();
    Color color;
    for (unsigned int i = 0; i < allViews.size(); i++) {
        this->plotLines(allViews[i].getSurfaces(), color.mix(color.random()));
    }

    if (!cv::imwrite(filename, canvas)) {
        cout << BOLDRED << "Failed to write " << filename << RESET << endl;
    }

}






//some functions

void findPlottingRange(double & minX, double & maxX, double & minY, double & maxY, const vector<Surface> & someSurfaces) {
    for (unsigned int j = 0; j < someSurfaces.size(); j++) {
        minX = min(minX, (double) someSurfaces[j].getP1().x);
        minX = min(minX, (double) someSurfaces[j].getP2().x);

        maxX = max(maxX, (double) someSurfaces[j].getP1().x);
        maxX = max(maxX, (double) someSurfaces[j].getP2().x);

        minY = min(minY, (double) someSurfaces[j].getP1().y);
        minY = min(minY, (double) someSurfaces[j].getP2().y);

        maxY = max(maxY, (double) someSurfaces[j].getP1().y);
        maxY = max(maxY, (double) someSurfaces[j].getP2().y);
    }
}

void addBorder(FILE * fgnup, const char * filename, double & minX, double & maxX, double & minY, double & maxY) {
    // Make sure x and y have the same range so the image isn't skewed
    double xRange = maxX - minX;
    double yRange = maxY - minY;
    double diff = yRange - xRange;
    if (diff > 0) {
        minX -= diff / 2.0;
        maxX += diff / 2.0;
    } else {
        minY -= -diff / 2.0;
        maxY += -diff / 2.0;
    }

    // Add a border to the image
    double border = (maxX - minX) * PLOT_BORDER_FACTOR; // x and y now have the same range
    minX -= border;
    maxX += border;
    minY -= border;
    maxY += border;

    fprintf(fgnup, "set terminal png size %d,%d nocrop linewidth 10\n", PLOT_RESOLUTION_X, PLOT_RESOLUTION_Y);
    fprintf(fgnup, "set output \"%s\"\n", filename);
    fprintf(fgnup, "set yrange[%g:%g]\n", minY, maxY);
    fprintf(fgnup, "set xrange[%g:%g]\n", minX, maxX);
}

void addSurfaces(FILE * fgnup, vector<Surface> someSurfaces) {
    //plotting surfaces
    for (unsigned int j = 0; j < someSurfaces.size(); j++) {
        fprintf(fgnup, "%g ", someSurfaces[j].getP1().x);
        fprintf(fgnup, "%g\n", someSurfaces[j].getP1().y);
        fprintf(fgnup, "%g ", someSurfaces[j].getP2().x);
        fprintf(fgnup, "%g\n\n", someSurfaces[j].getP2().y);

    }
    fprintf(fgnup, "e\n");
}

void addSurfacesAndRobot(FILE * fgnup, vector<Surface> someSurfaces, vector<Surface> robotSurfaces) {
    //plotting surfaces
    for (unsigned int j = 0; j < someSurfaces.size(); j++) {
        fprintf(fgnup, "%g ", someSurfaces[j].getP1().x);
        fprintf(fgnup, "%g\n", someSurfaces[j].getP1().y);
        fprintf(fgnup, "%g ", someSurfaces[j].getP2().x);
        fprintf(fgnup, "%g\n\n", someSurfaces[j].getP2().y);

    }
    //plotting robot
    for (unsigned int j = 0; j < robotSurfaces.size(); j++) {
        fprintf(fgnup, "%g ", robotSurfaces[j].getP1().x);
        fprintf(fgnup, "%g\n", robotSurfaces[j].getP1().y);
        fprintf(fgnup, "%g ", robotSurfaces[j].getP2().x);
        fprintf(fgnup, "%g\n\n", robotSurfaces[j].getP2().y);

    }
    fprintf(fgnup, "e\n");
}

void plotSurfacesGNU(const char * filename, const vector<Surface> & someSurfaces) {
    //open a file.
    FILE * fgnup = popen(GNUPLOT_PATH, "w");
    if (!fgnup) {
        cerr << "ERROR: " << GNUPLOT_PATH << " not found" << endl;
        return;
    } else {
        cout << filename << " is opened. :)" << endl;
    }

    //find plotting range for view surfaces.
    double minX = 0, minY = 0, maxX = 0, maxY = 0;
    findPlottingRange(minX, maxX, minY, maxY, someSurfaces);

    //add border to the image.
    addBorder(fgnup, filename, minX, maxX, minY, maxY);

    fprintf(fgnup, "plot ");
    fprintf(fgnup, "\"-\" ti \"Surfaces\" with lines 1\n");

    addSurfaces(fgnup, someSurfaces);

    fflush(fgnup);
    fclose(fgnup);

}

void plotSurfacesGNU(const char * filename, const vector<vector<Surface> > & someSurfaces) {
    //open a file.
    FILE * fgnup = popen(GNUPLOT_PATH, "w");
    if (!fgnup) {
        cerr << "ERROR: " << GNUPLOT_PATH << " not found" << endl;
        return;
    } else {
        cout << filename << " is opened. :)" << endl;
    }

    //find plotting range for view surfaces.
    double minX = 0, minY = 0, maxX = 0, maxY = 0;
    for (unsigned int i = 0; i < someSurfaces.size(); i++) {
        findPlottingRange(minX, maxX, minY, maxY, someSurfaces[i]);
    }

    //add border to the image.
    addBorder(fgnup, filename, minX, maxX, minY, maxY);

    fprintf(fgnup, "plot ");
    int lastI = -1;
    for (unsigned int i = 0; i < someSurfaces.size() - 1; i++) {
        fprintf(fgnup, "\"-\" ti \"Surfaces\" with lines %d, \\\n", i + 1);
        lastI = i;
    }
    fprintf(fgnup, "\"-\" ti \"Surfaces\" with lines %d\n", lastI + 5);

    //add surfaces.
    for (unsigned int i = 0; i < someSurfaces.size(); i++) {
        addSurfaces(fgnup, someSurfaces[i]);
    }

    fflush(fgnup);
    fclose(fgnup);
}

void plotViewGNU(const char * filename, const View & view, bool printID) {
    //open a file.
    FILE * fgnup = popen(GNUPLOT_PATH, "w");
    if (!fgnup) {
        cerr << "ERROR: " << GNUPLOT_PATH << " not found" << endl;
        return;
    } else {
        cout << filename << " is opened. :)" << endl;
    }

    // Get the plotting range
    double minX = 0, minY = 0, maxX = 0, maxY = 0;

    //find plotting range for view surfaces.
    findPlottingRange(minX, maxX, minY, maxY, view.getSurfaces());
    //find plotting range for landmarks.
    if (view.getLandmarks().size() > 0) {
        findPlottingRange(minX, maxX, minY, maxY, view.getLandmarks());
    }

    //add border to the image.
    addBorder(fgnup, filename, minX, maxX, minY, maxY);

    if (view.getLandmarks().size() > 0) {
        for (unsigned int j = 0; j < view.getLandmarks().size(); j++) {
            fprintf(fgnup, "set label \"%d\" at %g,%g\n", view.getLandmarks()[j].getId(),
                    view.getLandmarks()[j].midPoint().x + 500, view.getLandmarks()[j].midPoint().y + 500);
        }
    }

    //to print id
    if (printID == true) {
        for (unsigned int j = 0; j < view.getSurfaces().size(); j++) {
            fprintf(fgnup, "set label \"%d\" at %g,%g\n", view.getSurfaces()[j].getId(),
                    view.getSurfaces()[j].midPoint().x + 500, view.getSurfaces()[j].midPoint().y + 500);
        }
    }

    fprintf(fgnup, "plot ");
    if (view.getLandmarks().size() > 0) {
        fprintf(fgnup, "\"-\" ti \"Surfaces&Robot\" with lines 1, \\\n");
        fprintf(fgnup, "\"-\" ti \"Landmarks\" with lines 0\n");
    } else {
        fprintf(fgnup, "\"-\" ti \"Surfaces&Robot\" with lines 1\n");
    }

    //plotting surfaces & robot
    addSurfacesAndRobot(fgnup, view.getSurfaces(), view.getRobotSurfaces());
    //plotting landmark
    if (view.getLandmarks().size() > 0) {
        addSurfaces(fgnup, view.getLandmarks());
    }


    fflush(fgnup);
    fclose(fgnup);
}




//argument printID needs to set true to print ids of each surface. by default it's false.

void plotViewsGNU(const char * filename, const vector<View> & views, bool printID) {
    FILE * fgnup = popen(GNUPLOT_PATH, "w");
    if (!fgnup) {
        cerr << "ERROR: " << GNUPLOT_PATH << " not found" << endl;
        return;
    }

    cout << "Num of Views: " << views.size() << endl;
    // Get the plotting range
    double minX = 0, minY = 0, maxX = 0, maxY = 0;
    for (unsigned int i = 0; i < views.size(); i++) {
        findPlottingRange(minX, maxX, minY, maxY, views[i].getSurfaces());
        findPlottingRange(minX, maxX, minY, maxY, views[i].getRobotSurfaces());
    }

    //add border to the image.
    addBorder(fgnup, filename, minX, maxX, minY, maxY);


    //to print id
    if (printID == true) {
        for (unsigned int i = 0; i < views.size(); i++) {
            for (unsigned int j = 0; j < views[i].getSurfaces().size(); j++) {
                fprintf(fgnup, "set label \"%d\" at %g,%g\n", views[i].getSurfaces()[j].getId(),
                        views[i].getSurfaces()[j].midPoint().x + 500, views[i].getSurfaces()[j].midPoint().y + 500);
            }
        }
    }

    fprintf(fgnup, "plot ");
    int lastI = -1;
    for (unsigned int i = 0; i < views.size() - 1; i++) {
        fprintf(fgnup, "\"-\" ti \"Surfaces&Robot\" with lines %d, \\\n", i + 1);
        lastI = i;
    }
    fprintf(fgnup, "\"-\" ti \"Surfaces&Robot\" with lines %d\n", lastI + 5);



    // Plot Objects
    for (unsigned int i = 0; i < views.size(); i++) {
        //plotting surfaces & robot
        addSurfacesAndRobot(fgnup, views[i].getSurfaces(), views[i].getRobotSurfaces());
    }

    fflush(fgnup);
    fclose(fgnup);
}

//argument printID needs to set true to print ids of each surface. by default it's false.
/*
void plotViewBoundariesGNU(const char * filename, const vector<View> & views, bool printID) {
    FILE * fgnup = popen(GNUPLOT_PATH, "w");
    if (!fgnup) {
        cerr << "ERROR: " << GNUPLOT_PATH << " not found" << endl;
        return;
    }

    cout << "Num of Views: " << views.size() << endl;
    // Get the plotting range
    double minX = 0, minY = 0, maxX = 0, maxY = 0;
    for (unsigned int i = 0; i < views.size(); i++) {
        findPlottingRange(minX, maxX, minY, maxY, views[i].getSurfaces());
        findPlottingRange(minX, maxX, minY, maxY, views[i].getRobotSurfaces());
    }

    //add border to the image.
    addBorder(fgnup, filename, minX, maxX, minY, maxY);

    vector<vector < Surface> > allViewsBoundaries;
    for (unsigned int i = 0; i < views.size(); i++) {
        vector<Surface> boundarySurfaces;
        vector < pair<Surface, bool> > boundaries = views[i].getViewBoundaries();
        for (unsigned int j = 0; j < boundaries.size(); j++) {
            if (boundaries[j].second) {
                boundarySurfaces.push_back(boundaries[j].first);
            }
        }
        allViewsBoundaries.push_back(boundarySurfaces);
    }

    //to print id
    if (printID == true) {
        for (unsigned int i = 0; i < views.size(); i++) {
            for (unsigned int j = 0; j <  allViewsBoundaries[i].size(); j++) {
                fprintf(fgnup, "set label \"%d\" at %g,%g\n",  allViewsBoundaries[i][j].getId(),
                        allViewsBoundaries[i][j].midPoint().x + 500,  allViewsBoundaries[i][j].midPoint().y + 500);
            }
        }
    }

    fprintf(fgnup, "plot ");
    int lastI = -1;
    for (unsigned int i = 0; i < views.size() - 1; i++) {
        fprintf(fgnup, "\"-\" ti \"Surfaces&Robot\" with lines %d, \\\n", i + 1);
        lastI = i;
    }
    fprintf(fgnup, "\"-\" ti \"Surfaces&Robot\" with lines %d\n", lastI + 5);



    // Plot Objects
    for (unsigned int i = 0; i < views.size(); i++) {
        //plotting surfaces & robot
        addSurfacesAndRobot(fgnup,  allViewsBoundaries[i], views[i].getRobotSurfaces());
    }

    fflush(fgnup);
    fclose(fgnup);
}*/

void plotMapGNU(const char * filename, const Map & map, bool printID) {
    vector<View> views = map.getMap();
    plotViewsGNU(filename, views, printID);
}

void plotPointsAndSurfacesGNU(const char * filename, const vector<PointXY> & points, const vector<Surface> & robotSurfaces) {
    FILE * fgnup = popen(GNUPLOT_PATH, "w");
    if (!fgnup) {
        cerr << "ERROR: " << GNUPLOT_PATH << " not found" << endl;
        return;
    }

    // Get the plotting range
    double minX = 0, minY = 0, maxX = 0, maxY = 0;
    for (unsigned int i = 0; i < points.size(); i++) {
        minX = min(minX, points[i].getX());
        maxX = max(maxX, points[i].getX());
        minY = min(minY, points[i].getY());
        maxY = max(maxY, points[i].getY());
    }


    // Make sure x and y have the same range so the image isn't skewed
    double xRange = maxX - minX;
    double yRange = maxY - minY;
    double diff = yRange - xRange;
    if (diff > 0) {
        minX -= diff / 2.0;
        maxX += diff / 2.0;
    } else {
        minY -= -diff / 2.0;
        maxY += -diff / 2.0;
    }

    // Add a border to the image
    double border = (maxX - minX) * PLOT_BORDER_FACTOR; // x and y now have the same range
    minX -= border;
    maxX += border;
    minY -= border;
    maxY += border;


    fprintf(fgnup, "set terminal png size %d,%d nocrop linewidth 10\n", PLOT_RESOLUTION_X, PLOT_RESOLUTION_Y);
    fprintf(fgnup, "set output \"%s\"\n", filename);
    fprintf(fgnup, "set yrange[%g:%g]\n", minY, maxY);
    fprintf(fgnup, "set xrange[%g:%g]\n", minX, maxX);

    fprintf(fgnup, "plot \"-\" ti \"points\" with points 3 19, \\\n");
    fprintf(fgnup, "\"-\" ti \"Robot\" with lines 1\n");
    //fprintf(fgnup, "\"-\" ti \"Points\" with points 1 2\n");
    //fprintf(fgnup, "\"-\" ti \"Occluding edges\" with points 2 4\n");


    // Plot points
    for (unsigned int i = 0; i < points.size(); i++) {
        fprintf(fgnup, "%g ", points[i].getX());
        fprintf(fgnup, "%g\n", points[i].getY());
    }
    fprintf(fgnup, "e\n");

    //ploting robot
    for (unsigned int j = 0; j < robotSurfaces.size(); j++) {
        fprintf(fgnup, "%g ", robotSurfaces[j].getP1().x);
        fprintf(fgnup, "%g\n", robotSurfaces[j].getP1().y);
        fprintf(fgnup, "%g ", robotSurfaces[j].getP2().x);
        fprintf(fgnup, "%g\n\n", robotSurfaces[j].getP2().y);
    }
    fprintf(fgnup, "e\n");

    fflush(fgnup);
    fclose(fgnup);
}

// Write a vector of points to a file

void writeASCIIPoints2D(const char *filename, const vector<PointXY> & points) {
    ofstream outFile(filename, ios::out);

    // Output ASCII header (row and column)
    outFile << points.size() << " " << 2 << endl;

    // 8 digits should be more than enough
    outFile << fixed;
    outFile.precision(10);

    for (vector<PointXY>::const_iterator it = points.begin(); it != points.end(); ++it) {
        outFile << (*it).getX() << " ";
        outFile << (*it).getY() << endl;

    }

    outFile.close();
}

vector<PointXY> readASCIIPoints2D(const char *fileName) {
    vector<PointXY> points;
    ifstream inputFile(fileName, ios::in);
    if (inputFile.is_open()) {
        cout << "Reading view...." << endl;
        double x1, y1;
        string data;

        inputFile >> data;
        inputFile >> data;

        inputFile >> x1;
        inputFile >> y1;

        while (!inputFile.eof()) {
            points.push_back(PointXY(x1, y1));

            //reading odometry information
            inputFile >> x1;
            inputFile >> y1;
        }


    } else
        cout << "Error opening " << fileName << " .." << endl;


    cout << "Points are read. Num of points: " << points.size() << endl;
    //return surfaces;

    return points;
}