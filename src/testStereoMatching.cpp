#include <stdio.h>
#include <string.h>
#include <iostream>
#include <string>
#include <utility>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include<opencv2/calib3d/calib3d.hpp>
#include "opencv2/core/core.hpp"

using namespace cv;

int main()
{cv::Mat leftimg =cv::imread("leftimage.jpg");
cv::Mat rightimg = cv::imread("rightimage.jpg");
cv::Size imagesize = leftimg.size();
cv::Mat disparity_left=cv::Mat(imagesize.height,imagesize.width,CV_16S);
cv::Mat disparity_right=cv::Mat(imagesize.height,imagesize.width,CV_16S);
cv::Mat g1,g2,disp,disp8;
cv::cvtColor(leftimg,g1,cv::COLOR_BGR2GRAY);
cv::cvtColor(rightimg,g2,cv::COLOR_BGR2GRAY);

//cv::Ptr<cv::StereoBM> sbm = cv::createStereoBM(16,21);

Ptr<StereoBM> sbm = createStereoBM(16,21);

sbm.setDisp12MaxDiff(1);
sbm->setSpeckleRange(8);
sbm->setSpeckleWindowSize(9);
sbm->setUniquenessRatio(0);
sbm->setTextureThreshold(507);
sbm->setMinDisparity(-39);
sbm->setPreFilterCap(61);
sbm->setPreFilterSize(5);
sbm->compute(g1,g2,disparity_left);
normalize(disparity_left, disp8, 0, 255, CV_MINMAX, CV_8U);

cv::imshow("left", leftimg);
cv::imshow("right", rightimg);
cv::imshow("disp", disp8);

cv::waitKey(0);
}

//
//
//#include <stdio.h>
//    #include <iostream>
//    #include "opencv2/calib3d/calib3d.hpp"
//    #include "opencv2/core/core.hpp"
//    #include "opencv2/highgui/highgui.hpp"
//    
//    using namespace cv;
//    
//    const char *windowDisparity = "Disparity";
//    
//    void readme();
//    
//    int main( int argc, char** argv )
//    {
//      if( argc != 3 )
//      { readme(); return -1; }
//    
//      //-- 1. Read the images
//      Mat imgLeft = imread( argv[1], CV_LOAD_IMAGE_GRAYSCALE );
//      Mat imgRight = imread( argv[2], CV_LOAD_IMAGE_GRAYSCALE );
//      //-- And create the image in which we will save our disparities
//      Mat imgDisparity16S = Mat( imgLeft.rows, imgLeft.cols, CV_16S );
//      Mat imgDisparity8U = Mat( imgLeft.rows, imgLeft.cols, CV_8UC1 );
//    
//      if( !imgLeft.data || !imgRight.data )
//      { std::cout<< " --(!) Error reading images " << std::endl; return -1; }
//    
//      //-- 2. Call the constructor for StereoBM
//      int ndisparities = 16*5;   
//      int SADWindowSize = 21; 
//      StereoBM sbm( StereoBM::BASIC_PRESET,
//                                    ndisparities,
//                    SADWindowSize );
//    
//      //-- 3. Calculate the disparity image
//      sbm( imgLeft, imgRight, imgDisparity16S, CV_16S );
//    
//      //-- Check its extreme values
//      double minVal; double maxVal;
//    
//      minMaxLoc( imgDisparity16S, &minVal, &maxVal );
//    
//      printf("Min disp: %f Max value: %f \n", minVal, maxVal);
//    
//      //-- 4. Display it as a CV_8UC1 image
//      imgDisparity16S.convertTo( imgDisparity8U, CV_8UC1, 255/(maxVal - minVal));
//    
//      namedWindow( windowDisparity, WINDOW_NORMAL );
//      imshow( windowDisparity, imgDisparity8U );
//    
//      //-- 5. Save the image
//      imwrite("SBM_sample.png", imgDisparity16S);
//    
//      waitKey(0);
//    
//      return 0;
//    }
//    
//    void readme()
//    { std::cout << " Usage: ./SBMSample <imgLeft> <imgRight>" << std::endl; }
 