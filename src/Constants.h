#ifndef CONSTANTS_H
#define CONSTANTS_H


const int COLOR_IMAGE_WIDTH = 320;
const int COLOR_IMAGE_HEIGHT = 240;

#define DISPARITY_HEIGHT 240.
#define DISPARITY_WIDTH 320.
#define RECTIFIED_HEIGHT 768.
#define RECTIFIED_WIDTH 1024.


//some parameter for Robot class.
const double DIR_TOLERANCE = 5;
const double MOVE_TOLERANCE = 100;
const int TIMEOUT = 8000;
const int SHORT_PAUSE = 100;

const double CONVERT_TO_MM = 10;

//simple math
const double CONVERT_TO_RADIAN = (M_PI/180.0);
const double CONVERT_TO_DEGREE = (180.0/M_PI);

//view construction
const unsigned MINIMUM_SURFACE_POINTS = 2;
const double DEPTH_DIFF_TRESHOLD = 0.15;
const int COLOR_DIFF_TRESHOLD = 100;
//const int step = DISPARITY_WIDTH / 100;

//values to decide whether a surface is a landmark
#define LANDMARK_DIRECTION 10. //10 degree w.r.t robot facing
#define LANDMARK_DISTANCE 300. //3 Meter from robot

#endif /* CONSTANTS_H */
