#include "Printer.h"
#include "Color.h"


#include "View.h"

Printer::Printer () {
        sizeX = 500;
        sizeY = 500;
        
        canvas = cv::Mat::zeros(cv::Size(sizeX, sizeY), CV_8UC3);
        
        canvas.setTo(cv::Scalar(255, 255, 255));
        
    }

Printer::~Printer() {
    
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
   
    cv::imwrite(filename,canvas);
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
    
    
    cv::imwrite(filename, canvas);
}