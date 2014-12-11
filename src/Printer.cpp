#include "Printer.h"
#include "Color.h"

#include "Surface.h"


#include "View.h"
#include "Map.h"

Printer::Printer () {
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
    cv::Point2f  p1, p2;
    p1.x = sizeX/2 +point1.x;
    p1.y = sizeY - 15 - point1.y;
    
     p2.x = sizeX/2 +point2.x;
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
    this->plotLine(rbt0,rbt3,Color(0,0,0));
}

void Printer::plotLines(const std::vector<Surface>& surfaces, const Color & color) {
    
    cv::Point2f point1, point2;
    for(unsigned int i=0; i<surfaces.size(); i++) {
//        point1.x = sizeX/2 + surfaces[i].getP1().x;
//        point1.y = sizeY - 15 - surfaces[i].getP1().y;
//        
//        point2.x = sizeX/2 + surfaces[i].getP2().x;
//        point2.y = sizeY - 15 - surfaces[i].getP2().y;
        
        this->plotLine(surfaces[i].getP1(),surfaces[i].getP2(),color);
        
        
    }
}


void Printer::printSurfaces(const char* filename, const std::vector<Surface>& surfaces) {
    this->plotLines(surfaces, Color(255,0,0));
   
    if(!cv::imwrite(filename,canvas)) {
         cout<<BOLDRED<<"Failed to write "<<filename<<RESET<<endl;
     }
}
//
void Printer::printView(const char* filename, const View & aView) {
    this->plotRobot();
    
    this->plotLines(aView.getSurfaces(), Color(255,0,0));
    
    //plot points of surfaces.
    for(unsigned int i=0; i<aView.getSurfaces().size(); i++) {
        if(aView.getSurfaces()[i].getPoints().size() >= MINIMUM_SURFACE_POINTS)
            this->plotPoints(aView.getSurfaces()[i].getPoints());
    }
    
    cout<<"Num of cvLandmarks: "<<aView.getSurfaces().size()<<endl;
    displaySurfaces(aView.getSurfaces());
    
    cout<<"Num of pvlandmarksOnCV: "<<aView.getLandmarks().size()<<endl;
    displaySurfaces(aView.getLandmarks());
    //plot landmarks.
    this->plotLines(aView.getLandmarks(),Color(0,0,255));
    
    
    if(!cv::imwrite(filename,canvas)) {
         cout<<BOLDRED<<"Failed to write "<<filename<<RESET<<endl;
     }
}

//save map
 void Printer::printMap(const char* filename, const Map & curMap) {
     //starting new map. so clear canvas in case it's dirty:)
     this->cleanCanvas();
     vector<View> allViews = curMap.getMap();
     Color color;
     for(unsigned int i=0; i<allViews.size(); i++) {
         this->plotLines(allViews[i].getSurfaces(), color.mix(color.random()));
     }
     
     if(!cv::imwrite(filename,canvas)) {
         cout<<BOLDRED<<"Failed to write "<<filename<<RESET<<endl;
     }
     
 }
 
 void plotViewGNU(const char * filename, const View & view) {
    FILE * fgnup = popen(GNUPLOT_PATH, "w");
    if (!fgnup) {
        cerr << "ERROR: " << GNUPLOT_PATH << " not found" << endl;
        return;
    }

    //vector<View> views = map.getMap();
    
    // Get the plotting range
    double minX = 0, minY = 0, maxX = 0, maxY = 0;

        for (unsigned int j = 0; j < view.getSurfaces().size(); j++) {
            minX = min(minX, (double) view.getSurfaces()[j].getP1().x);
            minX = min(minX, (double) view.getSurfaces()[j].getP2().x);

            maxX = max(maxX, (double) view.getSurfaces()[j].getP1().x);
            maxX = max(maxX, (double) view.getSurfaces()[j].getP2().x);

            minY = min(minY, (double) view.getSurfaces()[j].getP1().y);
            minY = min(minY, (double) view.getSurfaces()[j].getP2().y);

            maxY = max(maxY, (double) view.getSurfaces()[j].getP1().y);
            maxY = max(maxY, (double) view.getSurfaces()[j].getP2().y);
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

    fprintf(fgnup, "plot ");
    int lastI = -1;
//    for (unsigned int i = 0; i < views.size()-1; i++) {
//        fprintf(fgnup, "\"-\" ti \"Objects\" with lines %d, \\\n", i + 1);
//        lastI = i;
//    }
    fprintf(fgnup, "\"-\" ti \"Objects\" with lines %d\n", lastI + 2);





        //ploting surfaces
        for (unsigned int j = 0; j < view.getSurfaces().size(); j++) {
            fprintf(fgnup, "%g ", view.getSurfaces()[j].getP1().x);
            fprintf(fgnup, "%g\n", view.getSurfaces()[j].getP1().y);
            fprintf(fgnup, "%g ", view.getSurfaces()[j].getP2().x);
            fprintf(fgnup, "%g\n\n", view.getSurfaces()[j].getP2().y);

        }
        //ploting robot
        for (unsigned int j = 0; j < view.getRobotSurfaces().size(); j++) {
            //if(Objects[i].getASRNo() == 1) {
            fprintf(fgnup, "%g ", view.getRobotSurfaces()[j].getP1().x);
            fprintf(fgnup, "%g\n", view.getRobotSurfaces()[j].getP1().y);
            fprintf(fgnup, "%g ", view.getRobotSurfaces()[j].getP2().x);
            fprintf(fgnup, "%g\n\n", view.getRobotSurfaces()[j].getP2().y);
            //}
        }
        fprintf(fgnup, "e\n");


    fflush(fgnup);
    fclose(fgnup);
}

void plotMapGNU(const char * filename, const Map & map) {
    FILE * fgnup = popen(GNUPLOT_PATH, "w");
    if (!fgnup) {
        cerr << "ERROR: " << GNUPLOT_PATH << " not found" << endl;
        return;
    }

    vector<View> views = map.getMap();
    cout<<"Num of Views: "<<map.getMap().size()<<endl;
    // Get the plotting range
    double minX = 0, minY = 0, maxX = 0, maxY = 0;
    for (unsigned int i = 0; i < views.size(); i++)
        for (unsigned int j = 0; j < views[i].getSurfaces().size(); j++) {
            minX = min(minX, (double) views[i].getSurfaces()[j].getP1().x);
            minX = min(minX, (double) views[i].getSurfaces()[j].getP2().x);

            maxX = max(maxX, (double) views[i].getSurfaces()[j].getP1().x);
            maxX = max(maxX, (double) views[i].getSurfaces()[j].getP2().x);

            minY = min(minY, (double) views[i].getSurfaces()[j].getP1().y);
            minY = min(minY, (double) views[i].getSurfaces()[j].getP2().y);

            maxY = max(maxY, (double) views[i].getSurfaces()[j].getP1().y);
            maxY = max(maxY, (double) views[i].getSurfaces()[j].getP2().y);
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

    fprintf(fgnup, "plot ");
    int lastI = -1;
    for (unsigned int i = 0; i < views.size()-1; i++) {
        fprintf(fgnup, "\"-\" ti \"Objects\" with lines %d, \\\n", i + 1);
        lastI = i;
    }
    fprintf(fgnup, "\"-\" ti \"Objects\" with lines %d\n", lastI + 5);



    // Plot Objects
    for (unsigned int i = 0; i < views.size(); i++) {
        //ploting surfaces
        for (unsigned int j = 0; j < views[i].getSurfaces().size(); j++) {
            fprintf(fgnup, "%g ", views[i].getSurfaces()[j].getP1().x);
            fprintf(fgnup, "%g\n", views[i].getSurfaces()[j].getP1().y);
            fprintf(fgnup, "%g ", views[i].getSurfaces()[j].getP2().x);
            fprintf(fgnup, "%g\n\n", views[i].getSurfaces()[j].getP2().y);

        }
        //ploting robot
        for (unsigned int j = 0; j < views[i].getRobotSurfaces().size(); j++) {
            //if(Objects[i].getASRNo() == 1) {
            fprintf(fgnup, "%g ", views[i].getRobotSurfaces()[j].getP1().x);
            fprintf(fgnup, "%g\n", views[i].getRobotSurfaces()[j].getP1().y);
            fprintf(fgnup, "%g ", views[i].getRobotSurfaces()[j].getP2().x);
            fprintf(fgnup, "%g\n\n", views[i].getRobotSurfaces()[j].getP2().y);
            //}
        }
        fprintf(fgnup, "e\n");
    }
    
//    //deleted surfaces
//    //ploting landmarks
//        for (unsigned int j = 0; j < views[views.size()-2].getLandmarks().size(); j++) {
//            //if(Objects[i].getASRNo() == 1) {
//            fprintf(fgnup, "%g ", views[views.size()-2].getLandmarks()[j].getP1().x);
//            fprintf(fgnup, "%g\n", views[views.size()-2].getLandmarks()[j].getP1().y);
//            fprintf(fgnup, "%g ", views[views.size()-2].getLandmarks()[j].getP2().x);
//            fprintf(fgnup, "%g\n\n", views[views.size()-2].getLandmarks()[j].getP2().y);
//            //}
//        }
//        fprintf(fgnup, "e\n");

    fflush(fgnup);
    fclose(fgnup);
}