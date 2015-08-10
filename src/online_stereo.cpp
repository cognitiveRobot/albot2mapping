// Created by Minh Nguyen for his phD project
// BP code is from RUI GONG  
// The Camera calibration part is copied from openCV website under free of use license, in December 2010
//#include <io.h>
#include <stdio.h>	// For printf()
#include <iostream>
#include <opencv/cv.h>		// Main OpenCV library.
#include <opencv/highgui.h>	// OpenCV functions for files and graphical windows.

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include "Printer.h"

#include "online_stereo.h"

using namespace std;
int height;
int width;
int dMin = 0;
int dMax = 60;
int dispDiff = 0;
int dCount = dMax + 1;
int smooth = 1;
int position = 0;

CvScalar point;

char *leftImg = "H:/Data/raw/left-9.pgm";
char *rightImg = "H:/Data/raw/right-9.pgm";
char *outputChar = "output.png";

int occlusion = 40;
int BigOcclusion = 60;
int counter = 0;

//declare arrays of left and right pixels with each different band RGB
static unsigned char *leftR;
static unsigned char *leftG;
static unsigned char *leftB;
static unsigned char *rightR;
static unsigned char *rightG;
static unsigned char *rightB;


int big = 6000000;
double minVal = 0.0, maxVal = 0.0;
IplImage* img;
node *lR;
node *rL;
short *labels;
IplImage* image1;
IplImage* image2;
IplImage* out;
CvPoint2D32f centre;
//initialise all rgb image pixel in to their arrays

void getRGB(IplImage* image, unsigned char r[], unsigned char g[], unsigned char b[], int height) {
    int y = height;
    {
        for (int x = 0; x < width; x++) {
            CvScalar p = cvGet2D(image, y, x);
            r[x] = (unsigned char) cvRound(p.val[0]);
            g[x] = (unsigned char) cvRound(p.val[1]);
            b[x] = (unsigned char) cvRound(p.val[2]);
        }
    }
}

void BP_Cu2(node* leftToRight, node* rightToLeft) {
    int w = width;
    int dispDiff = 0;

    //declare all ML, MR, B to be big values
    for (int i = 0; i < (width * dCount); i++) {
        leftToRight[i].ML = big;
        leftToRight[i].B = big;
        leftToRight[i].MR = big;
        rightToLeft[i].ML = big;
        rightToLeft[i].B = big;
        rightToLeft[i].MR = big;
    }

    int y = 0;
    {
        //calculate the forest and nodes from left to right

        for (int x = 1; x < w - 1; x++) {
            //going from bottom up of disparity of dEven count
            for (int dEven = 0; dEven < dMax; dEven += 2) {
                int posL = x + ceil((float) dEven / 2);
                int posR = x - floor((float) dEven / 2);

                if (posL < w && posR >= 0) {
                    float diffR = leftR[posL] - rightR[posR];
                    float diffG = leftG[posL] - rightG[posR];
                    float diffB = leftB[posL] - rightB[posR];

                    short msg = abs(diffR) + abs(diffG) + abs(diffB);

                    int currentD = dEven + dispDiff;

                    if (x <= 1) {
                        leftToRight[(posL * dCount) + currentD].MR = occlusion;
                        leftToRight[(posL * dCount) + currentD].B = msg;
                        leftToRight[(posL * dCount) + currentD].ML = occlusion;
                    } else {
                        if (currentD - 1 >= 0) {
                            leftToRight[(posL * dCount) + currentD].MR =
                                    min(leftToRight[((posL - 1) * dCount) + ((currentD - 1) * 1)].MR,
                                    leftToRight[((posL - 1) * dCount) + ((currentD - 1) * 1)].B) + occlusion;
                        } else {
                            leftToRight[(posL * dCount) + currentD].MR = big;
                        }

                        if (posL - 1 >= 1) {
                            int minimum = min(leftToRight[((posL - 1) * dCount) + currentD].MR,
                                    leftToRight[((posL - 1) * dCount) + currentD].B);

                            if (currentD + 1 < dCount) {
                                minimum = min(minimum, leftToRight[(posL * dCount) + (currentD + 1)].ML);
                            }

                            leftToRight[(posL * dCount) + currentD].B = minimum + msg;
                            leftToRight[(posL * dCount) + currentD].ML = minimum + occlusion;
                        }
                    }
                }
            }

            //going from bottom up of disparity of dOdd count
            for (int dOdd = 1; dOdd < dMax; dOdd += 2) {
                int posL = x + ceil((float) dOdd / 2);
                int posR = x - floor((float) dOdd / 2);

                if (posL < w && posR >= 0) {
                    float diffR = leftR[posL] - rightR[posR];
                    float diffG = leftG[posL] - rightG[posR];
                    float diffB = leftB[posL] - rightB[posR];

                    short msg = abs(diffR) + abs(diffG) + abs(diffB);

                    int currentD = dOdd + dispDiff;

                    if (x <= 1) {
                        leftToRight[(posL * dCount) + currentD].MR = occlusion;
                        leftToRight[(posL * dCount) + currentD].B = msg;
                        leftToRight[(posL * dCount) + currentD].ML = occlusion;
                    } else {
                        if (currentD - 1 >= 0) {
                            leftToRight[(posL * dCount) + currentD].MR =
                                    min(leftToRight[((posL - 1) * dCount) + ((currentD - 1) * 1)].MR,
                                    leftToRight[((posL - 1) * dCount) + ((currentD - 1) * 1)].B) + occlusion;
                        } else {
                            leftToRight[(posL * dCount) + currentD].MR = big;
                        }

                        if (posL - 1 >= 0) {
                            int minimum = min(leftToRight[((posL - 1) * dCount) + currentD].MR,
                                    leftToRight[((posL - 1) * dCount) + currentD].B);

                            if (currentD + 1 < dCount) {
                                minimum = min(minimum, leftToRight[(posL * dCount) + ((currentD + 1) * 1)].ML);
                            }

                            leftToRight[(posL * dCount) + currentD].B = minimum + msg;
                            leftToRight[(posL * dCount) + currentD].ML = minimum + occlusion;
                        }
                    }
                }
            }
        }


        //calculate the forest and nodes from right to left
        for (int x = w - 1; x >= 0; x--) {
            for (int dEven = 0; dEven <= dMax; dEven += 2) {
                int posL = x + (int) ceil((float) dEven / 2);
                int posR = x - (int) floor((float) dEven / 2);

                if (posL < w - 1 && posR >= 0) {
                    float diffR = leftR[posL] - rightR[posR];
                    float diffG = leftG[posL] - rightG[posR];
                    float diffB = leftB[posL] - rightB[posR];

                    short msg = abs(diffR) + abs(diffG) + abs(diffB);

                    int currentD = dEven + dispDiff;

                    if (x == w - 2) {
                        rightToLeft[(posL * dCount) + currentD].MR = occlusion;
                        rightToLeft[(posL * dCount) + currentD].B = msg;
                        rightToLeft[(posL * dCount) + currentD].ML = occlusion;
                    } else {
                        if (currentD - 1 >= 0) {
                            int minimum = min(rightToLeft[((posL + 1) * dCount) + currentD].B,
                                    rightToLeft[((posL + 1) * dCount) + currentD].ML);

                            if (currentD + 1 < dCount) {
                                minimum = min(minimum, rightToLeft[((posL + 1) * dCount) + ((currentD + 1) * 1)].MR);
                            }

                            rightToLeft[(posL * dCount) + currentD].MR = minimum + occlusion;
                            rightToLeft[(posL * dCount) + currentD].B = minimum + msg;

                            rightToLeft[(posL * dCount) + currentD].ML = min(
                                    rightToLeft[((posL) * dCount) + ((currentD - 1) * 1)].B,
                                    rightToLeft[((posL) * dCount) + ((currentD - 1) * 1)].ML) + occlusion;
                        } else {
                            rightToLeft[(posL * dCount) + currentD].ML = big;
                        }
                    }
                }
            }

            for (int dOdd = 1; dOdd <= dMax; dOdd += 2) {
                int posL = x + (int) ceil((float) dOdd / 2) - 1;
                int posR = x - (int) floor((float) dOdd / 2) - 1;

                if (posL < w - 1 && posR >= 0) {
                    float diffR = leftR[posL] - rightR[posR];
                    float diffG = leftG[posL] - rightG[posR];
                    float diffB = leftB[posL] - rightB[posR];

                    short msg = abs(diffR) + abs(diffG) + abs(diffB);

                    int currentD = dOdd + dispDiff;

                    if (x == w - 2) {
                        rightToLeft[(posL * dCount) + currentD].MR = occlusion;
                        rightToLeft[(posL * dCount) + currentD].B = msg;
                        rightToLeft[(posL * dCount) + currentD].ML = occlusion;
                    } else {
                        if (currentD - 1 >= 0) {
                            int minimum = min(rightToLeft[((posL + 1) * dCount) + currentD].B,
                                    rightToLeft[((posL + 1) * dCount) + currentD].ML);

                            if (currentD + 1 < dCount) {
                                minimum = min(minimum, rightToLeft[((posL + 1) * dCount) + ((currentD + 1) * 1)].MR);
                            }

                            rightToLeft[(posL * dCount) + currentD].MR = minimum + occlusion;
                            rightToLeft[(posL * dCount) + currentD].B = minimum + msg;

                            rightToLeft[(posL * dCount) + currentD].ML = min(
                                    rightToLeft[((posL) * dCount) + ((currentD - 1) * 1)].B,
                                    rightToLeft[((posL) * dCount) + ((currentD - 1) * 1)].ML) + occlusion;
                        } else {
                            rightToLeft[(posL * dCount) + currentD].ML = big;
                        }
                    }
                }
            }
        }

    }
}

void runStereo(IplImage* image1, IplImage* image2) {
    {

        width = image1->width;
        height = image1->height;
        CvSize size = cvSize(width, height);
        out = cvCreateImage(size, IPL_DEPTH_8U, 0);
        point.val[0] = 0;
        point.val[1] = 0;
        point.val[2] = 0;
        point.val[3] = 0;

        //array of each colour band
        leftR = new unsigned char[width];
        leftG = new unsigned char[width];
        leftB = new unsigned char[width];
        rightR = new unsigned char[width];
        rightG = new unsigned char[width];
        rightB = new unsigned char[width];

        lR = new node[width * dCount];
        rL = new node[width * dCount];

        labels = new short[width * dCount];

        for (int y = 0; y < height; y++)
            //int y = 200;
        {

            getRGB(image1, leftR, leftG, leftB, y); // y = height

            getRGB(image2, rightR, rightG, rightB, y);

            //setup the forest of Symmetric Dynamic programming
            BP_Cu2(lR, rL);

            //add up the forest from left to right and right to left to get symmetric result
            for (int x = 0; x < width; x++) {
                int tempX = x * dCount;
                for (int i = 0; i < dCount; i++) {
                    lR[tempX + i].MR += rL[tempX + i].MR;
                    lR[tempX + i].B += rL[tempX + i].B;
                    lR[tempX + i].ML += rL[tempX + i].ML;
                }
            }

            for (int x = 0; x < width; x++) {
                int top = 0;
                int topCount = 0;
                for (int i = 0; i < dCount; i++) {
                    int posL = x;
                    int posR = x;

                    if (posL < width && posR >= 0) {
                        if (i == 0) {
                            top = min
                                    (
                                    min
                                    (
                                    lR[(posL * dCount)].MR,
                                    lR[(posL * dCount)].B
                                    )
                                    ,
                                    lR[(posL * dCount)].ML
                                    );
                        } else {
                            int current = min(
                                    lR[(posL * dCount) + i].MR
                                    ,
                                    min(
                                    lR[(posL * dCount) + i].B
                                    ,
                                    lR[(posL * dCount) + i].ML
                                    )
                                    );

                            if (current < top) {
                                topCount = i;
                                top = current;
                            }
                        }
                    }
                }
                point.val[0] = floor((float) topCount * (255.0 / (float) dCount));
                cvSet2D(out, y, x, point);

            }
        }

        cvReleaseImage(&image1);
        cvReleaseImage(&image2);
        free(leftR);
        free(leftG);
        free(leftB);
        free(rightR);
        free(rightG);
        free(rightB);
        free(lR);
        free(rL);
        free(labels);
        //printf("done...\n");

        //cvReleaseImage(&out);		
    }
}

void computeDisparity(char *leftImg, char *rightImg, int dMin, int dMax, char *outputChar) {

    IplImage* image1 = cvLoadImage(leftImg);
    IplImage* image2 = cvLoadImage(rightImg);
     
    IplImage *image1S = cvCreateImage(cvSize((int) (image1->width * 0.5), (int) (image1->height * 0.5)), image1->depth, image1->nChannels);
    cvResize(image1, image1S);

    IplImage *image2S = cvCreateImage(cvSize((int) (image2->width * 0.5), (int) (image2->height * 0.5)), image2->depth, image2->nChannels);
    cvResize(image2, image2S);

    dCount = dMax + 1;

    runStereo(image1, image2);
    
    cvSaveImage(outputChar, out);
}

void computeDepthAndPointCloud( char* outputDepth, char *pointCloudfile, char *point2Dfile) {

    uchar *p;

    for (unsigned int y = 0; y < out->height; y++) {
        for (unsigned int x = 0; x < out->width; x++) {
            p = cvPtr2D(out, y, x, NULL);
            if (*p == 0) {
                *p = 100;
            } else {
                *p = 100 / (*p);
            }

        }
    }
    
    cvSaveImage(outputDepth, out);
    

    pcl::PointCloud<pcl::PointXYZ> cloud;

    // Fill in the cloud data
    cloud.width = (out->width - 100) * (int) (out->height / 3);
    cloud.height = 1;
    cloud.is_dense = false;
    cloud.points.resize(cloud.width * cloud.height);

    vector<vector<PointXY> > pointsAllRow;
    
    int cloudIndex = 0;

    for (unsigned int y = (int) (out->height / 3); y < (int) (out->height * 2 / 3); y++) {
        vector<PointXY> pointsOneRow;
        for (unsigned int x = 50; x < (out->width - 50); x++) {
            p = cvPtr2D(out, y, x, NULL);
            cloud[cloudIndex].x = (int) (x - out->width / 2) * (*p);
            cloud[cloudIndex].y = (*p)*200;
            cloud[cloudIndex].z = y  * (*p);

            pointsOneRow.push_back(PointXY(cloud[cloudIndex].x, cloud[cloudIndex].y));

            cloudIndex++;
        }
        pointsAllRow.push_back(pointsOneRow);
    }

    vector<PointXY> avgPoints;
    double sumX, sumY, avgX, avgY;
    int totalPointsInAColumn = 0;
    for (unsigned int j = 0; j < out->width - 101; j++) {
        sumX = 0;
        sumY = 0;
        totalPointsInAColumn = 0;
        for (unsigned int i = 0; i < pointsAllRow.size(); i++) {
            if (pointsAllRow[i][j].getX() != -1 && pointsAllRow[i][j].getY() != -1) {
                sumX += pointsAllRow[i][j].getX();
                sumY += pointsAllRow[i][j].getY();
                totalPointsInAColumn++;
                //      cout << "i " << i << " sX " << sumX << " sY " << sumY << " ";

            }
        }

        if (totalPointsInAColumn > 5) {
            avgX = sumX / totalPointsInAColumn;
            avgY = sumY / totalPointsInAColumn;
            /*    cout << "sumX " << sumX << " sumY " << sumY << endl;
                cout << "avgX " << avgX << " avgY " << avgY << endl;
                cout << "total points " << totalPointsInAColumn << endl;*/
            avgPoints.push_back(PointXY(avgX, avgY));
        }
    }

    pcl::io::savePCDFileASCII(pointCloudfile, cloud);
    
    writeASCIIPoints2D(point2Dfile, avgPoints);
    cout << "Points are plotted using GNU." << endl;

    plotPointsAndSurfacesGNU("../outputs/points.png", avgPoints, vector<Surface>());
}

