/* 
 * File:   Output.h
 * Author: Guillaume Diallo-Mulliez
 *
 * Created on May 30, 2013, 12:13 AM
 */

#ifndef OUTPUT_H
#define	OUTPUT_H



#include "header.h"
#include "blobObject.h"
#include "View.h"


/* ------------------------- Namespaces ------------------------- */
using namespace std;
//using namespace cv;

class Output {
private:

    cv::Mat drawing;
    cv::Mat Image500;
    cv::Mat mapDrawing;

    char filenameRight[50], filenameLeft[50], filenameColor[50], filenameRectified[50];





public:

    /* Constructor */
    Output();

    /* Destructor */
    ~Output();



    /* Writing in a .ppm file */
    int writePpm(char* szFilename, unsigned char* pucBuffer, int width, int height);



    /* Drawing */
    void rotate(cv::Point2f* target, cv::Point2f Center, float angle);

    cv::Mat createView(View curView);
    void updateMap(vector <View> Map);



    /* Display outputs in a file */
    void drawView(int v);
    void drawMap();






};

#endif	/* OUTPUT_H */

