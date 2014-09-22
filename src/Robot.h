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


/* -------------------- Global variables -------------------- */

const double DIR_TOLERANCE = 5;
const double MOVE_TOLERANCE = 100;
const int TIMEOUT = 8000;
const int SHORT_PAUSE = 100;

using namespace std;
using namespace cv;

#define _USE_MATH_DEFINES




class Robot
{
private:
    ArRobot robot;                           
    int step;                                // Number of step taken so far
    Point3f Pos;                             // Robot Position (X, Y, Angle)
    double odoDistanceOld;
    double globalAngleOld;
    double angleToMove;
    double distanceToMove;
    
    
    
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
    inline void setHeading(double heading);
    inline void moveDistance(double distance, double velocity);
    void updatePos(float r, float teta);
    Point3f getPos();
    
    void saveTravelInfo(double dist, double angle, char * filename);
    
};


#endif	/* ROBOT_H */

