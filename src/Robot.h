/* 
 * File:   Connector.h
 * Author: Guillaume Diallo-Mulliez
 *
 * Created on May 28, 2013, 12:20 AM
 */

#ifndef ROBOT_H
#define	ROBOT_H

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
#include "View.h"




using namespace std;
//using namespace cv;

#define _USE_MATH_DEFINES

struct AngleAndDistance {
    double angle;
    double distance;
};

class Robot {
private:
    ArRobot robot;
    int step; // Number of step taken so far
    cv::Point3f Pos; // Robot Position (X, Y, Angle)
    double odoDistanceOld;
    double globalAngleOld;
    double angleToMove;
    double distanceToMove;
    
    AngleAndDistance lastLocomotion;//save from move method.



public:

    /* Constructor */
    Robot();
    /* Destructor */
    ~Robot();

    /* Connection and disconnection to the robot */
    void connect(int argc, char **argv, ArSimpleConnector* connector);
    void disconnect();


    /* Movement */
    void incStep();
    int getStep();
    void move();
    void moveToTheGoal(AngleAndDistance & goal);
    inline void setHeading(double heading);
    inline void moveDistance(double distance, double velocity);
    void updatePos(float r, float teta);
    cv::Point3f getPos();
    
    AngleAndDistance getLastLocomotion();

    //save angle and distance travelled in a txt file.
    void saveTravelInfo(double dist, double angle, char * filename);

};


#endif	/* ROBOT_H */

