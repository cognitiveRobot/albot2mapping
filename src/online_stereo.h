#ifndef ONLINE_STEREO_H
#define ONLINE_STEREO_H


struct node {
    //3 states
    int ML; //monocular left
    int MR; //monocular right
    int B; //both == seen by both cameras
};

void getRGB(IplImage* image, unsigned char r[], unsigned char g[], unsigned char b[], int height);

void BP_Cu2(node* leftToRight, node* rightToLeft);

void runStereo(IplImage* image1, IplImage* image2);

void computeDisparity(char *leftImg, char *rightImg, int dMin, int dMax, char *outputChar) ;

void computeDepthAndPointCloud( char* outputDepth, char *pointCloudfile, char *point2Dfile);

#endif