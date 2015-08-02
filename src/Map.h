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

const int MIN_DISTANCE_VISION = 800;
const int MAX_DISTANCE_VISION = 3000;

class Map {
private:

    int sizeX;
    int sizeY;
    int M;
    int ID;

    vector<View> map;
    vector<pair<Surface, bool> > mapBoundaries; //bool=true if it's a surface, false if it's an exit       
    Surface exit;
    Surface entrance;
    vector<AngleAndDistance> pathSegments;
    vector<int> lostSteps; //it contains view number of lost situations.  

    //Attributes not used in current version of the program
    View currentView;
    View previousView;
    vector<Surface> surfaces;
    vector<cv::Point3f> rbtPos;
    cv::Mat drawing;
    list<Surface> boundaries;
    //Intervals of angle from the origin of the map in which we don't have any boundary yet
    vector< pair<double, double> > anglesWithoutBoundaries;
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

    vector<View> getMap() const;

    void setMapBoundaries(vector<pair<Surface, bool> > boundaries);
    vector<pair<Surface, bool> > getMapBoundaries();
    Surface getEntrance() const;

    Surface getExit() const;

    vector<int> getLostStepsNumber() const;

    void addPathSegment(const AngleAndDistance & lastPathSegment);
    vector<AngleAndDistance> getPathSegments() const;

    // Getters & setters not used in current version of the program (not sure) ----------------------------------------------------
    void setPreviousView(const View & pView);
    View getPreviousView();

    void setTempSurfaces(const vector<Surface> & surfs);
    vector<Surface> getTempSurfaces() const;

    void setLandmarkSurfaces(const vector<Surface> & surfs);
    vector<Surface> getLandmarkSurfaces() const;

    void setRefForPreviousLS(const Surface & surf);
    Surface getRefForPreviousLS();

    void setRefForNextLS(const Surface & surf);
    Surface getRefForNextLS();

    list<Surface> getBoundaries();
    void addBoundary(Surface newBoundary);
    vector< pair<double, double> > getAnglesWithoutBoundaries();
    bool setAnglesWithoutBoundaries(unsigned int index, pair<double, double> value);
    void addAnglesWithoutBoundaries(pair<double, double> value);
    //--End getters & setters --------------------------------------------------------------------------------------------------
    
    
    void computeEntrance(const Map& prevMap);
    void computeExit();
    
    View computeCVUsingOdometer(const View & curView);
    
    /**
     * Transform curView in map's coordinate using gaps as reference
     * @param curView
     * @param angleLastLocomotion
     * @return  view in map's coordinates
     */
    View computeCVUsingGap(View & curView, double angleLastLocomotion);
    
        /**
     * Adds curView to the map by finding in CV the (orthogonal) distance between longSurfCV and the other side of the gap
     * and building a surface in PV at the same (orthogonal) distance from the corresponding gap endpoint 
     * that passes by the other gap endpoint (throws false if impossible)
     * @param curView
     * @param longSurfCV
     * @param gapPV
     * @param prevRbtPos
     */
    void addViewUsingCorridorWidth(View& curView, Surface longSurfCV, Surface gapPV, cv::Point2f prevRbtPos);
    
        /**
     * Adds view to the map, clean it and plot it.
     * Doesn't change the view coordinates.
     * @param view
     */
    void addCvAndClean(const View & view);
    
        /**
     * Add curView to the map using gaps and long surfaces (uses addViewUsingCorridorWidth)
     * @param curView
     */
    void addCvUsingGap(View & curView);
    
        /**
     * Update the map by cleaning incoherent surfaces
     * @param cvSurfacesOnMap
     * @param cRobotSurfaces
     */
    void cleanMap(const vector<Surface>& cvSurfacesOnMap, const vector<Surface>& cRobotSurfaces);
    
        /**
     * Deletes surfaces less than MIN_DISTANCE_VISION away
     * @param robotSurfaces
     * @return the surfaces of the last view with the robot position in place of the deleted surfaces (for PointInPolygon)
     */
    vector<Surface> ClearCloseSurfaces(const vector<Surface>& robotSurfaces);

    /**
     * Returns the main direction (PCA) of the longest surface in CV.
     * We consider small surfaces close to the longest surface to compute the main direction.
     */
    Surface findLongSurfaceInCV(View& curView, View& prevView);

    /**
     * Find a gap in previous view in the angle and distance range we expect to 
     * find it according to the current view gap (using findExit)
     * @param robotOrientation expected position of the robot for the current step in map coordinates (odometer)
     * @param curView
     * @param prevView
     */
    Surface FindGapWithDistance(const Surface& robotOrientation, const View& curView, const View& prevView);
    
    /**
     * Build a map from the input data and compute its entrance and exit
     * @param dataset name of the folder containing the data
     * @param firstView number of the first view to compute
     * @param numSteps number of steps to compute
     * @param lastMap previous map computed
     */
    void BuildMap(char* dataset, int firstView, int numSteps, Map *lastMap = 0);

    View BuildMapWithBoundaries(char* dataset, View *firstView = 0, Map *lastMap = 0);

    vector<AngleAndDistance> FindWayHome();

    void addSurfacesAfterEntrance(Map& lastMap);
    

    //Methods not used in current version of the program (not sure) ---------------------------------------------------------
    vector<Surface> transformToGlobalMap(const vector<Surface>& rpSurfaces,
            const vector<AngleAndDistance>& allPathSegments);

    //void update(View newView); // Update the map according to the newView
    //bool isBehind(Surface Old, Surface New, cv::Point3f rbtPos); /* NOT SURE IF DONE CORRECTLY */ // Check if Old and New surfaces are concealing each other
    void coordTransf(cv::Point3f *target, cv::Point3f newCenter, double hX,
            double hY); // Transform the coordinates of *target with newCenter and homothetic transformation in X & Y directions
    void rotate(cv::Point2f* target, cv::Point2f Center, float angle); // Rotate *target point of angle around Center
    void display(); // Display map in output file
    View getView();

    Map getItself() const;

    void saveInTxtFile(const char * filename, const vector<Surface> & rpSurfaces);

    //mapping
    void findReferenceSurface(const View & curView, Surface & tempSurf);

    void expandMap(const View & curView);

    /**
     * Just adds the view to the map, no modification
     */
    void addView(View& view);

    //for aloCentric
    void addCVUsingOdo(const View & curView, const AngleAndDistance & homeInfo);
    void cleanMapUsingOdo(const View & curView, const AngleAndDistance & homeInfo);

    //for egoCentric
    void addPVUsingOdo(const View & curView, const AngleAndDistance & homeInfo);

    //updating
    void addCVUsingMultipleRef(const View & curView); //Deprecated

    /**
     * Computes the PCA direction of each side of the gap and links the views by using the best match
     * @param view
     * @param pcaExitBordersPV PCA direction of each side of the gap in previous view
     * @param angleLastLocomotion angle of the last locomotion (odometer)
     * @param rbtOrientationPV First surface of robot surfaces in previous view
     * @return view in map's coordinates
     */
    View computeViewPositionWithExitBorders(View& view, pair<Surface, Surface> pcaExitBordersPV, double angleLastLocomotion, Surface rbtOrientationPV);

    /**
     * Adds curView to the map by matching the long surfaces after extending them with the projection of the closest gap endpoint
     * on them. This projected point is used as reference point for the matching.
     * (throws false if impossible)
     * @param curView
     * @param gapInCV gap in current view
     * @param gapInPV equivalent gap in previous view
     * @param longSurfInCV longest surface in current view
     * @param longSurfInPV equivalent surface in previous view
     */
    void addViewProjectingGapOnLongSurf(View& curView, Surface gapInCV, Surface gapInPV, Surface longSurfInCV, Surface longSurfInPV);
    
};

Surface makeSurfaceWith(const Surface & refInMap, double angle, double distance, double length);
Surface trangulateSurface(const Surface & refInMap, const Surface & refInCV, const Surface & cvSurface, const int & refPoint);
vector<Surface> trangulateSurfaces(const Surface & refInMap, const Surface & refInCV, const vector<Surface>& cvSurfaces);
//point in polygon.
bool PointInPolygon(const double & pointX, const double & pointY, const vector<Surface>& surfaces,
        const vector<Surface>& robotSurfaces);

/**
 * Find the closest accessible exit 
 * @param surfaces
 * @param robotSurfaces
 * @param takeBorderPoints if true, the first and last points will be considered
 * @param otherExit if not null, make sure the orientation of the exit found is not too different from the otherExit's orientation
 * @param refSurfacePair odometer ReferenceSurfaces used to compare the orientation of the exit found with otherExit's orientation
 * @return 
 */
Surface findExit(const vector<Surface>& surfaces, const cv::Point2f rbtPos, const bool takeBorderPoints = false, const Surface *otherExit = 0, ReferenceSurfaces *refSurfacePair = 0);

/**
 * Returns true if exit matches all the criteria to be an exit (accessible, long enough, not crossing any surface)
 */
bool isExit(const Surface& exit, const PointXY& rbtPos, const vector<Surface>& surfaces);

/**
 * Find the long surface in PV corresponding to longSurfCV 
 * by finding the longest surface in the right distance range and on the same side of the gap
 * @param prevView
 * @param curView
 * @param rbtPosNextStep expected robot position in current step in map's coordinates (odometer)
 * @param longSurfCV main direction of the longest surface in CV
 * @return main direction of the longest surface in PV (PCA)
 */
Surface findLongSurfaceInPV(View& prevView, View& curView, cv::Point2f rbtPosNextStep, Surface& longSurfCV);

/**
 * Return true if the point to check can't be seen from the robot position because it's hidden by a surface in allSurfaces
 */
bool PointHiddenBySurfaces(const cv::Point2f pointToCheck, const vector<Surface>& allSurfaces, const vector<Surface>& robotSurfaces);

/**
 * Returns true if the surface is hiding pointToCheck
 */
bool SurfaceHidingPoint(const cv::Point2f pointToCheck, const Surface& surface, const vector<Surface>& robotSurfaces);



//Methods not used in current version of the program (not sure) ----------------------------------------------------------------
vector<double> findBoundariesOfCV(const vector<Surface> & cvSurfaces, double extension);

//construct polygon from a set of surfaces
vector<SurfaceT> constructRectangle(const vector<Surface> & transformedSurfaces);
vector<SurfaceT> constructPolygon(const vector<Surface> & transformedSurfaces, const vector<Surface> & robotSurfaces);

//mapping 
ReferenceSurfaces findTheClosestReference(Surface & cvSurface, vector<ReferenceSurfaces> allRefSurfaces);

/**
 * @Deprecated
 * Find the closest gap from the robot
 * @param surfaces
 * @param robotSurfaces
 * @param takeBorderPoints if true, the first and last point will be considered
 * @return 
 */
Surface FindGap(const vector<Surface>& surfaces, const vector<Surface>& robotSurfaces, const bool takeBorderPoints = false);

#endif	/* MAPPER_H */

