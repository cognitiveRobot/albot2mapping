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

const int MIN_DISTANCE_VISION=800; 
const int MAX_DISTANCE_VISION=5000; 

class Map {
private:

    int sizeX;
    int sizeY;
    int M;
    int ID;
    
    vector<int> lostSteps; //it contains view number of lost situations.

    View currentView;
    View previousView;
    vector<Surface> surfaces;
    vector<cv::Point3f> rbtPos;
    cv::Mat drawing;
    list<Surface> boundaries;
   // double leftBoundaryAngle;
  //  double rightBoundaryAngle;
    vector< pair<double,double> > anglesWithoutBoundaries;
    
    vector<View> map;
    
    vector<AngleAndDistance> pathSegments;
    
    vector<Surface> tempSurfaces;
    vector<Surface> landmarkSurfaces;

    Surface refForPreviousLS;
    Surface refForNextLS;


public:

    Map(int _sizeX, int _sizeY);
    ~Map();
    
    void initializeMap(const View & firstView);
    
    void setMapID(const int & a);
    int getMapID();
    
    void setPreviousView(const View & pView);
    View getPreviousView();
    
    list<Surface> getBoundaries();
    void addBoundary(Surface newBoundary);
    
    vector< pair<double,double> >  getAnglesWithoutBoundaries();
    bool setAnglesWithoutBoundaries(unsigned int index, pair<double,double> value);
    void addAnglesWithoutBoundaries(pair<double,double> value);
    
    void setTempSurfaces(const vector<Surface> & surfs);
    vector<Surface> getTempSurfaces() const;
    
    void setLandmarkSurfaces(const vector<Surface> & surfs);
    vector<Surface> getLandmarkSurfaces() const;
    
    void setRefForPreviousLS(const Surface & surf);
    Surface getRefForPreviousLS();
    
    void setRefForNextLS(const Surface & surf);
    Surface getRefForNextLS();
    
    vector<int> getLostStepsNumber() const;
   
    void addPathSegment(const AngleAndDistance & lastPathSegment);
    vector<AngleAndDistance> getPathSegments() const;
    
    vector<Surface> transformToGlobalMap(const vector<Surface>& rpSurfaces, 
    const vector<AngleAndDistance>& allPathSegments);
    
    void cleanMap(const vector<Surface>& cvSurfacesOnMap, const vector<Surface>& cRobotSurfaces);
    
    //for aloCentric
    void addCVUsingOdo(const View & curView, const AngleAndDistance & homeInfo);
    void cleanMapUsingOdo(const View & curView, const AngleAndDistance & homeInfo);
    
    //for egoCentric
    void addPVUsingOdo(const View & curView, const AngleAndDistance & homeInfo);
    
    //updating
    void addCVUsingMultipleRef(const View & curView); //Deprecated
    
    //addCVUsingMultipleRef split in 2 parts
    View computeCVUsingMultipleRef(const View & curView);
    void addCv(const View & view);
    
    
    //void update(View newView); // Update the map according to the newView
    //bool isBehind(Surface Old, Surface New, cv::Point3f rbtPos); /* NOT SURE IF DONE CORRECTLY */ // Check if Old and New surfaces are concealing each other
    void coordTransf(cv::Point3f *target, cv::Point3f newCenter, double hX,
            double hY); // Transform the coordinates of *target with newCenter and homothetic transformation in X & Y directions
    void rotate(cv::Point2f* target, cv::Point2f Center, float angle); // Rotate *target point of angle around Center
    void display(); // Display map in output file
    View getView();
    
    vector<View> getMap() const;
    
    Map getItself() const;
    
    void saveInTxtFile(const char * filename, const vector<Surface> & rpSurfaces);
    
    //mapping
    void findReferenceSurface(const View & curView, Surface & tempSurf);
    
    void expandMap(const View & curView);
    
    //delete surfaces less than MIN_DISTANCE_VISION away
    //returns the surfaces of the last view with the robot position in place of the deleted surfaces (for PointInPolygon)
   vector<Surface> ClearCloseSurfaces(const vector<Surface>& robotSurfaces);    
};

//
vector<double> findBoundariesOfCV(const vector<Surface> & cvSurfaces, double extension);

//construct polygon from a set of surfaces
vector<SurfaceT> constructRectangle(const vector<Surface> & transformedSurfaces);
vector<SurfaceT> constructPolygon(const vector<Surface> & transformedSurfaces, const vector<Surface> & robotSurfaces);

//mapping 
ReferenceSurfaces findTheClosestReference(Surface & cvSurface, vector<ReferenceSurfaces> allRefSurfaces);
Surface makeSurfaceWith(const Surface & refInMap, double angle, double distance, double length);
Surface trangulateSurface(const Surface & refInMap, const Surface & refInCV, const Surface & cvSurface, const int & refPoint);
vector<Surface> trangulateSurfaces(const Surface & refInMap, const Surface & refInCV, const vector<Surface>& cvSurfaces);


//point in polygon.
bool PointInPolygon(const double & pointX, const double & pointY, const vector<Surface>& surfaces, 
        const vector<Surface>& robotSurfaces);

/**
 * Return true if the point to check can't be seen from the robot position because it's hidden by a surface in allSurfaces
 */
bool PointHiddenBySurfaces(const cv::Point2f pointToCheck, const vector<Surface>& allSurfaces, const vector<Surface>& robotSurfaces);

bool SurfaceHidingPoint(const cv::Point2f pointToCheck, const Surface& surface, const vector<Surface>& robotSurfaces);

#endif	/* MAPPER_H */

