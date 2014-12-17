/* 
 * File:   Mapper.h
 * Author: Guillaume Diallo-Mulliez
 *
 * Created on May 30, 2013, 1:22 AM
 */

#ifndef MAPPER_H
#define	MAPPER_H

#include "View.h"


/* ------------------------- Namespaces ------------------------- */
using namespace std;
//using namespace cv;


class Map {
private:

    int sizeX;
    int sizeY;
    int M;

    View currentView;
    View previousView;
    vector<Surface> surfaces;
    vector<cv::Point3f> rbtPos;
    cv::Mat drawing;
    
    vector<View> map;
    
    vector<AngleAndDistance> pathSegments;




public:

    Map(int _sizeX, int _sizeY);
    ~Map();

    void initializeMap(const View & firstView);
    
    void setPreviousView(const View & pView);
    View getPreviousView();
    
    void addPathSegment(const AngleAndDistance & lastPathSegment);
    vector<AngleAndDistance> getPathSegments() const;
    
    vector<Surface> transformToGlobalMap(const vector<Surface>& rpSurfaces, 
    const vector<AngleAndDistance>& allPathSegments);
    
    //for aloCentric
    void addCVUsingOdo(const View & curView, const AngleAndDistance & homeInfo);
    void cleanMapUsingOdo(const View & curView, const AngleAndDistance & homeInfo);
    
    //for egoCentric
    void addPVUsingOdo(const View & curView, const AngleAndDistance & homeInfo);
    
    
    void update(View newView); // Update the map according to the newView
    bool isBehind(Surface Old, Surface New, cv::Point3f rbtPos); /* NOT SURE IF DONE CORRECTLY */ // Check if Old and New surfaces are concealing each other
    void coordTransf(cv::Point3f *target, cv::Point3f newCenter, double hX,
            double hY); // Transform the coordinates of *target with newCenter and homothetic transformation in X & Y directions
    void rotate(cv::Point2f* target, cv::Point2f Center, float angle); // Rotate *target point of angle around Center
    void display(); // Display map in output file
    View getView();
    
    vector<View> getMap() const;
    
    void saveInTxtFile(const char * filename, const vector<Surface> & rpSurfaces);
    
    //mapping
    void expandMap(const View & curView);
};

//
vector<double> findBoundariesOfCV(const vector<Surface> & cvSurfaces, double extension);

//mapping 
vector<Surface> trangulateSurfaces(const Surface & refInMap, const Surface & refInCV, const vector<Surface>& cvSurfaces);


#endif	/* MAPPER_H */

