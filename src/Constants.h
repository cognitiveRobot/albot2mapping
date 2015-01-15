#ifndef CONSTANTS_H
#define CONSTANTS_H


#ifdef WAITHERE
   char temp;
   cout<<"Waiting...";
    cin>>temp;
#endif

//localspaces or global map type(egoCentric or allCentric)
//true means egoCentric reference frame will be used. otherwise allCentric.
const bool EGOCENTRIC_REFERENCE_FRAME = true;
const bool REMEMBER_COMMON_VIEW = false;


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
const double CONVERT_M_TO_MM = 1000;

//simple math
const double CONVERT_TO_RADIAN = (M_PI/180.0);
const double CONVERT_TO_DEGREE = (180.0/M_PI);

//view construction
const unsigned MINIMUM_SURFACE_POINTS = 2;
const double DEPTH_DIFF_TRESHOLD = 0.15;
const int COLOR_DIFF_TRESHOLD = 100;
//const int step = DISPARITY_WIDTH / 100;

//values to decide whether a surface is a landmark
#define LANDMARK_DIRECTION 40. //10 degree w.r.t robot facing
#define LANDMARK_DISTANCE 300. //3 Meter from robot



//switch for global mapping.
const bool COMPUTE_GLOBAL_MAP = true;
const bool PRINT_GLOBAL_MAP = true;


#define _HANDLE_TRICLOPS_ERROR( function, error ) \
{ \
   if( error != TriclopsErrorOk ) \
   { \
      printf( \
	 "ERROR: %s reported %s.\n", \
	 function, \
	 triclopsErrorToString( error ) ); \
      exit( 1 ); \
   } \
} \


//color text on terminal
#define RESET   "\033[0m"
#define BLACK   "\033[30m"      /* Black */
#define RED     "\033[31m"      /* Red */
#define GREEN   "\033[32m"      /* Green */
#define YELLOW  "\033[33m"      /* Yellow */
#define BLUE    "\033[34m"      /* Blue */
#define MAGENTA "\033[35m"      /* Magenta */
#define CYAN    "\033[36m"      /* Cyan */
#define WHITE   "\033[37m"      /* White */
#define BOLDBLACK   "\033[1m\033[30m"      /* Bold Black */
#define BOLDRED     "\033[1m\033[31m"      /* Bold Red */
#define BOLDGREEN   "\033[1m\033[32m"      /* Bold Green */
#define BOLDYELLOW  "\033[1m\033[33m"      /* Bold Yellow */
#define BOLDBLUE    "\033[1m\033[34m"      /* Bold Blue */
#define BOLDMAGENTA "\033[1m\033[35m"      /* Bold Magenta */
#define BOLDCYAN    "\033[1m\033[36m"      /* Bold Cyan */
#define BOLDWHITE   "\033[1m\033[37m"      /* Bold White */

#endif /* CONSTANTS_H */
