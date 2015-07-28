#include "Map.h"
#include "PointAndSurface.h"
#include "GeometryFuncs.h"
#include "ImageProcessing.h"
#include "Printer.h"
#include "SameSurfaceFinderOdo.h"
#include "GlobalMap.h"
#include <list>

Map::Map(int _sizeX, int _sizeY) :
sizeX(_sizeX), sizeY(_sizeY) {
    sizeX = 2000;
    sizeY = 2000;
    M = 0;
    ID = 0;
    anglesWithoutBoundaries.push_back(pair<double, double>(0, 360));
    //=vector<cv::Point3f>(1,cv::Point3f(0,0,0));
}

Map::~Map() {
}

void Map::setMapID(const int & a) {
    ID = a;
}

int Map::getMapID() {
    return ID;
}

void Map::initializeMap(const View & firstView) {
    map.clear(); //make sure the map is empty.
    map.push_back(firstView); //add the first view.
    pathSegments.clear(); //make sure the pathsegments is empty.
    /* vector<Surface> surfaces = firstView.getSurfaces(); // findTrustedSurfaces(firstView);
    boundaries = list<Surface>(surfaces.begin(), surfaces.end());
    Surface robotOrientation = firstView.getRobotSurfaces()[0];
    double leftBoundaryAngle = robotOrientation.getAngleFromP1ToPoint(boundaries.front().getP1().x, boundaries.front().getP1().y);
    double rightBoundaryAngle = robotOrientation.getAngleFromP1ToPoint(boundaries.back().getP2().x, boundaries.back().getP2().y);
    anglesWithoutBoundaries[0] = pair<double, double>(leftBoundaryAngle, rightBoundaryAngle);
     */
    entrance = firstView.getRobotSurfaces()[0];
    entrance.rotateAroundP1(-90);
    entrance.setP1(entrance.getP1().x - (entrance.getP2().x - entrance.getP1().x), entrance.getP1().y - (entrance.getP2().y - entrance.getP1().y));
    char mapName[50];
    sprintf(mapName, "%s%d%s%d%s", "../outputs/Maps/Map-", getMapID(), "-View-", firstView.getId(), "a-before.png");
    plotViewsGNU(mapName, this->getMap());
}

void Map::setPreviousView(const View & pView) {
    previousView = pView;
}

View Map::getPreviousView() {
    return previousView;
}

list<Surface> Map::getBoundaries() {
    return boundaries;
}

void Map::addBoundary(Surface newBoundary) {
    Surface rbtOrientation = this->map[0].getRobotSurfaces()[0];
    double angleNewP1 = rbtOrientation.getAngleFromP1ToPoint(newBoundary.getP1().x, newBoundary.getP1().y);
    double angleNewP2 = rbtOrientation.getAngleFromP1ToPoint(newBoundary.getP2().x, newBoundary.getP2().y);
    list<Surface>::iterator it = boundaries.begin();
    while (it != boundaries.end()) {
        double angleItP1 = rbtOrientation.getAngleFromP1ToPoint(it->getP1().x, it->getP1().y);
        double angleItP2 = rbtOrientation.getAngleFromP1ToPoint(it->getP2().x, it->getP2().y);
        if (angleNewP1 < angleItP1 && angleNewP1 < angleItP2 && angleNewP2 < angleItP1 && angleNewP2 < angleItP2) {
            boundaries.insert(it, newBoundary);
            break;
        } else if (angleNewP1 > angleItP1 && angleNewP1 > angleItP2 && angleNewP2 > angleItP1 && angleNewP2 > angleItP2) {
            it++;
            continue;
        } else if (newBoundary.length() > it->length()) {
            boundaries.insert(it, newBoundary);
            break;
        } else {
            it++;
        }
    }
    if (it == boundaries.end()) {
        boundaries.push_back(newBoundary);
    }
}

vector< pair<double, double> > Map::getAnglesWithoutBoundaries() {
    return anglesWithoutBoundaries;
}

bool Map::setAnglesWithoutBoundaries(unsigned int index, pair<double, double> value) {
    if (index < anglesWithoutBoundaries.size()) {
        anglesWithoutBoundaries[index] = value;
        return true;
    }
    return false;
}

void Map::addAnglesWithoutBoundaries(pair<double, double> value) {
    anglesWithoutBoundaries.push_back(value);
}

void Map::addPathSegment(const AngleAndDistance & lastPathSegment) {
    pathSegments.push_back(lastPathSegment);
}

vector<AngleAndDistance> Map::getPathSegments() const {
    return pathSegments;
}

void Map::setTempSurfaces(const vector<Surface> & surfs) {
    tempSurfaces = surfs;
}

vector<Surface> Map::getTempSurfaces() const {
    return tempSurfaces;
}

void Map::setLandmarkSurfaces(const vector<Surface> & surfs) {
    landmarkSurfaces = surfs;
}

vector<Surface> Map::getLandmarkSurfaces() const {
    return landmarkSurfaces;
}

void Map::setRefForPreviousLS(const Surface & surf) {
    refForPreviousLS = surf;
}

Surface Map::getRefForPreviousLS() {
    return refForPreviousLS;
}

void Map::setRefForNextLS(const Surface & surf) {
    refForNextLS = surf;
}

Surface Map::getRefForNextLS() {
    return refForNextLS;
}

vector<int> Map::getLostStepsNumber() const {
    return lostSteps;
}

Map Map::getItself() const {
    return *this;
}

Surface Map::getEntrance() const {
    return entrance;
}

Surface Map::getExit() const {
    return exit;
}

void Map::coordTransf(cv::Point3f *target, cv::Point3f newCenter, double hX,
        double hY) {
    // Homothetic transformation to adapt to axes
    target->x *= hX;
    target->y *= hY;
    // Change the center of the frame reference
    target->x += newCenter.x;
    target->y += newCenter.y;
}

void Map::rotate(cv::Point2f* target, cv::Point2f Center, float angle) {
    float r;
    float teta;
    target->x = target->x - Center.x;
    target->y = target->y - Center.y;
    target->y = -target->y;
    r = sqrt(target->x * target->x + target->y * target->y); // Getting distance from center
    if (target->x < 0) {
        if (target->y < 0) {
            teta = atan(target->y / target->x) - M_PI;
        } else {
            teta = atan(target->y / target->x) + M_PI;
        }
    } else
        teta = atan(target->y / target->x);
    // Set angle for rotated point
    angle *= M_PI;
    angle /= (float) 180;
    teta += angle;
    // Set new position
    target->x = r * cos(teta);
    target->y = r * sin(teta);
    target->x = target->x + Center.x;
    target->y = Center.y - target->y;
}

vector<Surface> Map::transformToGlobalMap(const vector<Surface>& rpSurfaces,
        const vector<AngleAndDistance>& allPathSegments) {
    vector<Surface> transformed = rpSurfaces;
    double newX, newY, angle;
    for (unsigned int i = allPathSegments.size(); i-- > 0;) {
        cout << i << " angle: " << allPathSegments[i].angle << " dist: " << allPathSegments[i].distance << endl;
        angle = allPathSegments[i].angle * CONVERT_TO_RADIAN; // degree to randian.
        //find cv center in the pv coordinate frame.
        //need to convert robot position from mm to cm.
        newX = (allPathSegments[i].distance / 10.0) * sin(-angle); //x= d*cos(th) = d*cos(90-angle) = d*sin(angle) //as aris give - value for right turn
        newY = (allPathSegments[i].distance / 10.0) * cos(-angle); //y=d*sin(th)=d*sin(90-angle)=d*cos(angle)
        //transforming robot
        for (unsigned int i = 0; i < transformed.size(); i++) {
            transformed[i] = transformed[i].transformB(newX, newY, angle);
        }
    }
    return transformed;
}
//ego-centric mapping.
//adding pv on to cv to compute egoCentric map.

void Map::addPVUsingOdo(const View & curView, const AngleAndDistance & homeInfo) {
    //check the point in polygon cleaning old.
    vector<Surface> tempSurf;
    View tempView;
    double newX, newY, angle;
    angle = this->pathSegments.back().angle * CONVERT_TO_RADIAN; // degree to randian.
    //find cv center in the pv coordinate frame.
    //need to convert robot position from mm to cm.
    newX = (this->pathSegments.back().distance / 1.0) * sin(-angle); //x= d*cos(th) = d*cos(90-angle) = d*sin(angle) //as aris give - value for right turn
    newY = (this->pathSegments.back().distance / 1.0) * cos(-angle); //y=d*sin(th)=d*sin(90-angle)=d*cos(angle)
    Surface refForPLS = this->getRefForPreviousLS();
    refForPLS = refForPLS.transFrom(newX, newY, angle);
    this->setRefForPreviousLS(refForPLS);
    for (unsigned int i = 0; i < this->map.size(); i++) {
        tempView = this->map[i];
        //transform surfaces
        tempSurf = tempView.getSurfaces();
        for (unsigned int j = 0; j < tempSurf.size(); j++) {
            tempSurf[j] = tempSurf[j].transFrom(newX, newY, angle);
        }
        tempView.setSurfaces(tempSurf);
        //transform robotSurfaces
        tempSurf = tempView.getRobotSurfaces();
        for (unsigned int j = 0; j < tempSurf.size(); j++) {
            tempSurf[j] = tempSurf[j].transFrom(newX, newY, angle);
        }
        tempView.setRobotSurfaces(tempSurf);
        //reset View.
        this->map[i] = tempView;
    }
    //add current View.
    this->map.push_back(curView); //in the map.
    this->setLandmarkSurfaces(curView.getSurfaces()); //
}
//all-centric mapping.
//transform cView to first view co-ordinate and add to expand the current local space.

void Map::addCVUsingOdo(const View & curView, const AngleAndDistance & homeInfo) {
    cout << "Add curView to the map." << endl;
    vector<Surface> transformedSurfaces;
    cout << "@Home: " << homeInfo.angle << " " << homeInfo.distance << endl;
    vector<AngleAndDistance> allPathSegments = this->getPathSegments();
    cout << "Num of pathSegments: " << allPathSegments.size() << endl;
    //transforming surfaces
    vector<Surface> cvSurfaces = curView.getSurfaces();
    vector<Surface> rpSurfaces = curView.getRobotSurfaces();
    cout << "CV size: " << cvSurfaces.size() << endl;
    double newX, newY, angle;
    for (unsigned int i = allPathSegments.size(); i-- > 0;) {
        cout << i << " angle: " << allPathSegments[i].angle << " dist: " << allPathSegments[i].distance << endl;
        angle = allPathSegments[i].angle * CONVERT_TO_RADIAN; // degree to randian.
        //find cv center in the pv coordinate frame.
        //need to convert robot position from mm to cm.
        newX = (allPathSegments[i].distance / 1.0) * sin(-angle); //x= d*cos(th) = d*cos(90-angle) = d*sin(angle) //as aris give - value for right turn
        newY = (allPathSegments[i].distance / 1.0) * cos(-angle); //y=d*sin(th)=d*sin(90-angle)=d*cos(angle)
        //transform cv on to map.
        for (unsigned int i = 0; i < cvSurfaces.size(); i++) {
            cvSurfaces[i] = cvSurfaces[i].transformB(newX, newY, angle);
        }
        //transforming robot
        for (unsigned int i = 0; i < rpSurfaces.size(); i++) {
            rpSurfaces[i] = rpSurfaces[i].transformB(newX, newY, angle);
        }
    }
    //cout << "RobotPos: " << newX << " " << newY << endl;
    cout << BOLDRED << "Transfomed surfs: " << cvSurfaces.size() << RESET << endl;
    View tranView;
    tranView.setSurfaces(cvSurfaces);
    tranView.setRobotSurfaces(rpSurfaces);
    map.push_back(tranView);
    this->setLandmarkSurfaces(curView.getSurfaces()); //
}

void Map::cleanMap(const vector<Surface>& cvSurfacesOnMap, const vector<Surface>& cRobotSurfaces) {
    vector<Surface> surfacesOutsideCV, finalSurfaces;
    vector<Surface> tempSurf;
    View tempView;

    //Deleting the last view surfaces closer than MIN_DISTANCE_VISION
    vector<Surface> surfacesForPointInPolygon = ClearCloseSurfaces(cRobotSurfaces);
    vector<Surface> lastViewSurfaces = this->map[this->map.size() - 1].getSurfaces();
    vector<Surface> robotPath;
    for (unsigned int i = 1; i<this->map.size(); i++) {
        cv::Point2f rbtPos1 = this->map[i - 1].getRobotSurfaces()[0].getP1();
        cv::Point2f rbtPos2 = this->map[i].getRobotSurfaces()[0].getP1();
        robotPath.push_back(Surface(rbtPos1.x, rbtPos1.y, rbtPos2.x, rbtPos2.y));
    }
    if (lastViewSurfaces.size() > 0) {
        for (unsigned int i = 0; i<this->map.size() - 1; i++) {
            tempView = this->map[i];
            tempSurf = tempView.getSurfaces();

            //Check for surfaces we're seeing through
            for (unsigned int j = 0; j < tempSurf.size(); j++) {
                bool intersectingRbtPath = false;
                for (unsigned int k = 0; k < robotPath.size(); k++) {
                    if (tempSurf[j].intersects(robotPath[k])) {
                        intersectingRbtPath = true;
                        break;
                    }
                }
                if (intersectingRbtPath) {
                    //We don't keep the surface because the robot has passed through it
                    continue;
                }

                //Point in polygon
                bool P1inPolygon = PointInPolygon(tempSurf[j].getP1().x, tempSurf[j].getP1().y, surfacesForPointInPolygon, cRobotSurfaces);
                bool P2inPolygon = PointInPolygon(tempSurf[j].getP2().x, tempSurf[j].getP2().y, surfacesForPointInPolygon, cRobotSurfaces);
                bool MiddleInPolygon = PointInPolygon(tempSurf[j].midPoint().x, tempSurf[j].midPoint().y, surfacesForPointInPolygon, cRobotSurfaces);
                if (P1inPolygon && P2inPolygon && MiddleInPolygon) {
                    // Don't keep the surface
                    continue;
                } else if (P1inPolygon || P2inPolygon || MiddleInPolygon) {
                    // Keeps only the parts of the surface outside polygon
                    vector<cv::Point2f> tempPoints = tempSurf[j].getAllPoints();
                    cv::Point2f lastPointOut;
                    bool inPolygon = P1inPolygon;
                    if (!P1inPolygon) {
                        lastPointOut = tempPoints[0];
                    }
                    for (unsigned int k = 1; k < tempPoints.size(); k++) {
                        bool test = PointInPolygon(tempPoints[k].x, tempPoints[k].y, surfacesForPointInPolygon, cRobotSurfaces);
                        if (inPolygon && !test) {
                            lastPointOut = tempPoints[k];
                            inPolygon = false;
                        } else if (!inPolygon && test) {
                            surfacesOutsideCV.push_back(Surface(lastPointOut.x, lastPointOut.y, tempPoints[k - 1].x, tempPoints[k - 1].y));
                            inPolygon = true;
                        }
                    }
                    if (!inPolygon) {
                        //Close the last surface
                        surfacesOutsideCV.push_back(Surface(lastPointOut.x, lastPointOut.y, tempPoints[tempPoints.size() - 1].x, tempPoints[tempPoints.size() - 1].y));
                    }
                } else if (!P1inPolygon && !P2inPolygon && !MiddleInPolygon) {
                    // Keeps the whole surface
                    surfacesOutsideCV.push_back(tempSurf[j]);
                }
            }
            //Check for points hidden by a surface
            for (unsigned int j = 0; j < surfacesOutsideCV.size(); j++) {
                bool P1hidden = PointHiddenBySurfaces(surfacesOutsideCV[j].getP1(), lastViewSurfaces, tempView.getRobotSurfaces());
                bool P2hidden = PointHiddenBySurfaces(surfacesOutsideCV[j].getP2(), lastViewSurfaces, tempView.getRobotSurfaces());
                if (P1hidden && P2hidden) {
                    // Don't keep the surface
                    continue;
                } else if (P1hidden || P2hidden) {
                    // Keeps only the parts of the surface not hidden
                    vector<cv::Point2f> points = surfacesOutsideCV[j].getAllPoints();
                    cv::Point2f lastVisiblePoint;
                    bool hidden = P1hidden;
                    if (!P1hidden) {
                        lastVisiblePoint = points[0];
                    }
                    for (unsigned int k = 1; k < points.size(); k++) {
                        bool test = PointHiddenBySurfaces(points[k], lastViewSurfaces, cRobotSurfaces);
                        if (hidden && !test) {
                            lastVisiblePoint = points[k];
                            hidden = false;
                        } else if (!hidden && test) {
                            finalSurfaces.push_back(Surface(lastVisiblePoint.x, lastVisiblePoint.y, points[k - 1].x, points[k - 1].y));
                            hidden = true;
                        }
                    }
                    if (!hidden) {
                        //Close the last surface
                        finalSurfaces.push_back(Surface(lastVisiblePoint.x, lastVisiblePoint.y, points[points.size() - 1].x, points[points.size() - 1].y));
                    }
                } else if (!P1hidden && !P2hidden) {
                    // Keeps the whole surface
                    finalSurfaces.push_back(surfacesOutsideCV[j]);
                }
            }
            tempView.setSurfaces(finalSurfaces);
            this->map[i] = tempView;
            //clear variables to reuse.
            tempSurf.clear();
            surfacesOutsideCV.clear();
            finalSurfaces.clear();
        }
    }
    cout << "Has been cleaned" << endl;
}

void Map::cleanMapUsingOdo(const View & curView, const AngleAndDistance & homeInfo) {
    cout << "Cleaning the map." << endl;
    vector<Surface> transformedSurfaces;
    double angle = homeInfo.angle * CONVERT_TO_RADIAN; // degree to randian.
    //find cv center in the pv coordinate frame.
    //need to convert robot position from mm to cm.
    double newX = (homeInfo.distance / 10.0) * sin(-angle); //x= d*cos(th) = d*cos(90-angle) = d*sin(angle) //as aris give - value for right turn
    double newY = (homeInfo.distance / 10.0) * cos(-angle); //y=d*sin(th)=d*sin(90-angle)=d*cos(angle)
    //transforming surfaces
    vector<Surface> cvSurfaces = curView.getSurfaces();
    cout << "cvSurfaces: " << cvSurfaces.size() << endl;
    //find cv boundary
    vector<double> boundariesOfCV = findBoundariesOfCV(cvSurfaces, 20.0);
    //for printing only
    vector<Surface> boundaryLines;
    boundaryLines.push_back(Surface(0, 0, boundariesOfCV[0], boundariesOfCV[1]));
    boundaryLines.push_back(Surface(boundariesOfCV[0], boundariesOfCV[1], boundariesOfCV[0], boundariesOfCV[4]));
    boundaryLines.push_back(Surface(boundariesOfCV[0], boundariesOfCV[4], boundariesOfCV[2], boundariesOfCV[4]));
    boundaryLines.push_back(Surface(boundariesOfCV[2], boundariesOfCV[4], boundariesOfCV[2], boundariesOfCV[3]));
    boundaryLines.push_back(Surface(boundariesOfCV[2], boundariesOfCV[3], 0, 0));
    if (EGOCENTRIC_REFERENCE_FRAME == true)
        transformedSurfaces = boundaryLines;
    else {
        for (unsigned int i = 0; i < boundaryLines.size(); i++) {
            transformedSurfaces.push_back(boundaryLines[i].transformB(newX, newY, angle));
        }
    }
    //for debugging only.
    // char mapName[50];
    // sprintf(mapName, "%s%d%s%d%s", "../outputs/Maps/LS-", this->getMapID(), "-v-", curView.getId(), "a-before-withB.png");
    // this->setTempSurfaces(boundaryLines);
    // plotMapGNU(mapName, this->getItself());
    // this->getTempSurfaces().clear();
    //making polygon from cv.
    vector<SurfaceT> polygon = constructRectangle(transformedSurfaces);
    //check the point in polygon cleaning old.
    //cleanMap(polygon);
}
//all-centric mapping.
//add cv to the map using multiple reference.

void Map::addCVUsingMultipleRef(const View & curView) {
    // cout<<endl<<"Adding cv using multiple reference surfaces!!!"<<endl;
    //find reference surfaces
    ReferenceSurfaces refSurfacePair;
    /* vector<ReferenceSurfaces> sameSurfaces;
    SameSurfaceFinderOdo sSurfaceInfo;
    sSurfaceInfo.recognizeAllSameSurface(sameSurfaces, this->getMap(), this->getLandmarkSurfaces(), curView.getSurfaces(), this->getPathSegments().back());
    if ( sameSurfaces.size() > 1000) { //using landmark as reference.
    // if ( sameSurfaces.size() > 0 &&
    // (curView.getId() == 7 or curView.getId() == 19 or curView.getId() == 33 or curView.getId() == 67)) {
    cout<<"The following reference surfaces are found .."<<endl;
    for(unsigned int i=0;i<sameSurfaces.size(); i++) {
    sameSurfaces[i].display();
    //sameSurfaces[i].getMapSurface().display();
    }
    } else {*/
    // cout<<endl<<endl<<"Recognition failed. Need to use odometery. "<<endl;
    //using odometer.
    refSurfacePair.setMapSurface(makeSurfaceWith(this->getMap().back().getRobotSurfaces()[0],
            this->getPathSegments().back().angle, this->getPathSegments().back().distance, 400.0));
    refSurfacePair.setViewSurface(curView.getRobotSurfaces()[0]);
    refSurfacePair.setRefPoint(1);
    //save this view number.
    this->lostSteps.push_back(curView.getId());
    //waitHere();
    //}
    Surface cvSurfaceOnMap;
    vector<Surface> allCVSurfacesOnMap;
    for (unsigned int i = 0; i < curView.getSurfaces().size(); i++) {
        //find the closest ref pair using current view
        // if(sameSurfaces.size() > 0)
        // refSurfacePair = findTheClosestReference(curView.getSurfaces()[i],sameSurfaces);
        //trangulate this surface.
        // cout << curView.getSurfaces()[i].getId() << " refID: " << refSurfacePair.getViewSurface().getId() << endl;
        cvSurfaceOnMap = trangulateSurface(refSurfacePair.getMapSurface(), refSurfacePair.getViewSurface(),
                curView.getSurfaces()[i], refSurfacePair.getRefPoint());
        allCVSurfacesOnMap.push_back(cvSurfaceOnMap);
    }
    //compute robot surfaces.
    Surface cRobotSurfaceOnMap;
    vector<Surface> cRobotSurfacesOnMap;
    // if(sameSurfaces.size() > 0)
    // refSurfacePair = findTheClosestReference(curView.getRobotSurfaces()[0],sameSurfaces);
    for (unsigned int i = 0; i < curView.getRobotSurfaces().size(); i++) {
        cRobotSurfaceOnMap = trangulateSurface(refSurfacePair.getMapSurface(), refSurfacePair.getViewSurface(),
                curView.getRobotSurfaces()[i], refSurfacePair.getRefPoint());
        cRobotSurfacesOnMap.push_back(cRobotSurfaceOnMap);
    }
    View cViewOnMap;
    cViewOnMap.setSurfaces(allCVSurfacesOnMap);
    cViewOnMap.setRobotSurfaces(cRobotSurfacesOnMap);
    this->map.push_back(cViewOnMap);
    char mapName[50];
    sprintf(mapName, "%s%d%s%d%s", "../outputs/Maps/Map-", getMapID(), "-View-", curView.getId(), "a-before.png");
    plotViewsGNU(mapName, this->getMap());
    //cleanMap.
    cleanMap(allCVSurfacesOnMap, cRobotSurfacesOnMap);
    sprintf(mapName, "%s%d%s%d%s", "../outputs/Maps/Map-", getMapID(), "-View-", curView.getId(), "b-after.png");
    //demo1
    // vector<View> views = this->getMap();
    // views.push_back(makeViewFromSurfaces(convertSurfaceT2Surface(polygon)));
    // plotViewsGNU(mapName,views);
    //end demo1
    plotViewsGNU(mapName, this->getMap());
    //
    // waitHere();
}

View Map::computeCVUsingOdometer(const View& curView) {
    ReferenceSurfaces refSurfacePair;
    //using odometer.
    refSurfacePair.setMapSurface(makeSurfaceWith(this->getMap().back().getRobotSurfaces()[0],
            this->getPathSegments().back().angle, this->getPathSegments().back().distance, 400.0));
    refSurfacePair.setViewSurface(curView.getRobotSurfaces()[0]);
    refSurfacePair.setRefPoint(1);
    //save this view number.
    this->lostSteps.push_back(curView.getId());
    //waitHere();
    Surface cvSurfaceOnMap;
    vector<Surface> allCVSurfacesOnMap;
    for (unsigned int i = 0; i < curView.getSurfaces().size(); i++) {
        //trangulate this surface.
        cvSurfaceOnMap = trangulateSurface(refSurfacePair.getMapSurface(), refSurfacePair.getViewSurface(),
                curView.getSurfaces()[i], refSurfacePair.getRefPoint());
        allCVSurfacesOnMap.push_back(cvSurfaceOnMap);
    }
    //compute robot surfaces.
    Surface cRobotSurfaceOnMap;
    vector<Surface> cRobotSurfacesOnMap;
    for (unsigned int i = 0; i < curView.getRobotSurfaces().size(); i++) {
        cRobotSurfaceOnMap = trangulateSurface(refSurfacePair.getMapSurface(), refSurfacePair.getViewSurface(),
                curView.getRobotSurfaces()[i], refSurfacePair.getRefPoint());
        cRobotSurfacesOnMap.push_back(cRobotSurfaceOnMap);
    }
    View cViewOnMap;
    cViewOnMap.setSurfaces(allCVSurfacesOnMap);
    cViewOnMap.setRobotSurfaces(cRobotSurfacesOnMap);
    cViewOnMap.setId(curView.getId());
    cViewOnMap.setHasGap(curView.getHasGap());
    return cViewOnMap;
}

void Map::computeEntrance(const Map& prevMap) {

    ReferenceSurfaces refSurfacePair;
    refSurfacePair.setMapSurface(this->map.front().getRobotSurfaces()[0]);
    refSurfacePair.setViewSurface(makeSurfaceWith(prevMap.getMap().back().getRobotSurfaces()[0],
            prevMap.getPathSegments().back().angle, prevMap.getPathSegments().back().distance, 400.0));
    refSurfacePair.setRefPoint(1);

    this->entrance = trangulateSurface(refSurfacePair.getMapSurface(), refSurfacePair.getViewSurface(),
            prevMap.getExit(), refSurfacePair.getRefPoint());
}

void Map::computeExit() {

    double distance = this->getPathSegments().back().distance;
    if (distance > 100) {
        //We position the exit a little before the next robot position if the robot has moved enough
        distance -= 100;
    }
    Surface beforeFirstPosition = makeSurfaceWith(this->map.back().getRobotSurfaces()[0], this->getPathSegments().back().angle, distance, 400.0);
    beforeFirstPosition.rotateAroundP1(-90);

    //Double the surface's length by extending it on P1 side
    beforeFirstPosition.setP1(beforeFirstPosition.getP1().x - (beforeFirstPosition.getP2().x - beforeFirstPosition.getP1().x), beforeFirstPosition.getP1().y - (beforeFirstPosition.getP2().y - beforeFirstPosition.getP1().y));
    this->exit = beforeFirstPosition;
}

void Map::addView(View& view) {
    this->map.push_back(view);
}

void Map::addCvAndClean(const View& view) {
    this->map.push_back(view);
    char mapName[50];
    sprintf(mapName, "%s%d%s%d%s", "../outputs/Maps/Map-", getMapID(), "-View-", view.getId(), "a-before.png");
    plotViewsGNU(mapName, this->getMap());
    //cleanMap.
    cleanMap(view.getSurfaces(), view.getRobotSurfaces());
    sprintf(mapName, "%s%d%s%d%s", "../outputs/Maps/Map-", getMapID(), "-View-", view.getId(), "b-after.png");
    plotViewsGNU(mapName, this->getMap());
    // waitHere();
}

View Map::computeCVUsingGap(View & curView, double angleLastLocomotion) {
    Surface mapGap = this->map[this->map.size() - 1].getGap();
    Surface curGap = curView.getGap();
    double angleGaps = mapGap.getAngleWithSurface(curGap);
    double diff = abs(angleGaps - angleLastLocomotion);
    if (diff > 90 && diff < 270) {
        curGap = Surface(curGap.getP2().x, curGap.getP2().y, curGap.getP1().x, curGap.getP1().y);
        curView.setGap(curGap);
    }
    ReferenceSurfaces refSurfacePair;
    refSurfacePair.setViewSurface(curGap);
    refSurfacePair.setMapSurface(mapGap);
    refSurfacePair.setRefPoint(1);
    Surface cvSurfaceOnMap;
    vector<Surface> allCVSurfacesOnMap;
    for (unsigned int i = 0; i < curView.getSurfaces().size(); i++) {
        //trangulate this surface
        cvSurfaceOnMap = trangulateSurface(refSurfacePair.getMapSurface(), refSurfacePair.getViewSurface(),
                curView.getSurfaces()[i], refSurfacePair.getRefPoint());
        allCVSurfacesOnMap.push_back(cvSurfaceOnMap);
    }
    //compute robot surfaces.
    Surface cRobotSurfaceOnMap;
    vector<Surface> cRobotSurfacesOnMap;
    for (unsigned int i = 0; i < curView.getRobotSurfaces().size(); i++) {
        cRobotSurfaceOnMap = trangulateSurface(refSurfacePair.getMapSurface(), refSurfacePair.getViewSurface(),
                curView.getRobotSurfaces()[i], refSurfacePair.getRefPoint());
        cRobotSurfacesOnMap.push_back(cRobotSurfaceOnMap);
    }
    View cViewOnMap;
    cViewOnMap.setSurfaces(allCVSurfacesOnMap);
    cViewOnMap.setRobotSurfaces(cRobotSurfacesOnMap);
    cViewOnMap.setId(curView.getId());
    return cViewOnMap;
}

void Map::addCvUsingGap(View & curView) {

    //Compute approximate robot position at the current step in last view coordinates (odometer)
    Surface newRbtSurface;
    ReferenceSurfaces refSurfacePair;
    refSurfacePair.setMapSurface(makeSurfaceWith(this->getMap().back().getRobotSurfaces()[0],
            this->getPathSegments().back().angle, this->getPathSegments().back().distance, 400.0));
    refSurfacePair.setViewSurface(curView.getRobotSurfaces()[0]);
    refSurfacePair.setRefPoint(1);
    newRbtSurface = trangulateSurface(refSurfacePair.getMapSurface(), refSurfacePair.getViewSurface(),
            curView.getRobotSurfaces()[0], refSurfacePair.getRefPoint());

    View prevView = this->map[map.size() - 1];

    //Find longest surface in current view
    Surface longSurfCV = this->findLongSurfaceInCV(curView, prevView);

    //Update previous view gap by searching the current gap in the previous view (using odometer distance and angle range)    
    try {
        cout << "Searching gap in previous view" << endl;
        prevView.setGap(this->FindGapWithDistance(newRbtSurface, curView, prevView));
        this->map[map.size() - 1] = prevView;
    } catch (bool e) {
        cout << "Cannot use gaps, using long surface" << endl;
        throw false;
    }

    //Find same long surface in previous view
    Surface longSurfPV;
    try {
        longSurfPV = findLongSurfaceInPV(prevView, curView, newRbtSurface.getP1(), longSurfCV);
    } catch (bool e) {
        cout << "Cannot find long surface in previous view" << endl;
        throw false;
    }

    //Plot long surfaces
    char longSurfPlot [50];
    sprintf(longSurfPlot, "%s%d%s", "../outputs/Maps/longSurfCV-", curView.getId(), ".png");
    vector<Surface> tmp;
    tmp.push_back(longSurfCV);
    tmp.push_back(curView.getGap());
    plotSurfacesGNU(longSurfPlot, tmp);

    sprintf(longSurfPlot, "%s%d%s", "../outputs/Maps/longSurfPV-", curView.getId(), ".png");
    vector<Surface> tmp2;
    tmp2.push_back(longSurfPV);
    tmp2.push_back(prevView.getGap());
    plotSurfacesGNU(longSurfPlot, tmp2);

    //Plot gaps
    char gapsplot[50];
    sprintf(gapsplot, "%s%d%s", "../outputs/Maps/gapsPlot-", curView.getId(), ".png");
    vector<Surface> tmpGapsPlot = curView.getSurfaces();
    tmpGapsPlot.push_back(curView.getGap());
    plotSurfacesGNU(gapsplot, tmpGapsPlot);
    sprintf(gapsplot, "%s%d%s", "../outputs/Maps/gapsPlot-", curView.getId(), "-prev.png");
    vector<Surface> tmpGapsPlot2 = prevView.getSurfaces();
    tmpGapsPlot2.push_back(prevView.getGap());
    plotSurfacesGNU(gapsplot, tmpGapsPlot2);

    //Add view to map
    this->addViewUsingCorridorWidth(curView, longSurfCV, prevView.getGap(), prevView.getRobotSurfaces()[0].getP1());
}

void Map::saveInTxtFile(const char * filename, const vector<Surface> & rpSurfaces) {
    cout << "Saving map in a txt file" << endl;
    ofstream outFile(filename, ios::out);
    vector<Surface> surfaces;
    // 8 digits should be more than enough
    // outFile << fixed;
    //outFile.precision(10);
    vector<View> allViews = this->getMap();
    outFile << "@TotalViews: " << allViews.size() << endl;
    outFile << endl << "@SPosition: " << 0 << " " << 0 << " " << 0 << " " << 40 << endl;
    outFile << "@CPosition: " << rpSurfaces[0].getP1().x << " ";
    outFile << rpSurfaces[0].getP1().y << " ";
    outFile << rpSurfaces[0].getP2().x << " ";
    outFile << rpSurfaces[0].getP2().y << endl;
    outFile << endl << "@RefSurfaces: " << endl;
    outFile << this->refForPreviousLS.getP1().x << " ";
    outFile << this->refForPreviousLS.getP1().y << " ";
    outFile << this->refForPreviousLS.getP2().x << " ";
    outFile << this->refForPreviousLS.getP2().y << endl;
    outFile << this->refForNextLS.getP1().x << " ";
    outFile << this->refForNextLS.getP1().y << " ";
    outFile << this->refForNextLS.getP2().x << " ";
    outFile << this->refForNextLS.getP2().y << endl;
    outFile << endl << "@AllSurfaces: " << endl;
    for (unsigned int j = 0; j < allViews.size(); j++) {
        surfaces = allViews[j].getSurfaces();
        for (int i = 0; i < int(surfaces.size()); i++) {
            outFile << surfaces[i].getP1().x << " ";
            outFile << surfaces[i].getP1().y << " ";
            outFile << surfaces[i].getP2().x << " ";
            outFile << surfaces[i].getP2().y << endl;
        }
        // waitHere();
    }
    outFile.close();
    cout << "Map file saved" << endl;
}

void Map::display() {
    drawing = cv::Mat::zeros(cv::Size(sizeX, sizeY), CV_8UC3);
    drawing.setTo(cv::Scalar(255, 255, 255));
    cv::Point2f P11, P21, P1, P2;
    cv::Point2f O(0, 0);
    Surface curObst1;
    // Draw robot positions
    for (unsigned int i = 0; i < rbtPos.size(); i++) {
        cv::Point2f rbt0(rbtPos[i].x, rbtPos[i].y);
        cv::Point2f rbt1(rbt0.x - 15, rbt0.y - 15);
        cv::Point2f rbt2(rbt0.x + 15, rbt0.y + 15);
        cv::Point2f rbt3(rbt0.x, rbt0.y - 30);
        rotate(&rbt3, rbt0, rbtPos[i].z); // Set the robot into the right direction
        cv::rectangle(drawing, rbt1, rbt2, cv::Scalar(0, 0, 255), 3, 8, 0);
        cv::line(drawing, rbt0, rbt3, cv::Scalar(0, 0, 0), 3, 8, 0);
    }
    // Draw Surfaces Surfaces
    //cout << surfaces.size() << " Surfaces drawn in new map" << endl;
    for (unsigned int i = 0; i < surfaces.size(); i++) {
        // Adapt the point to the openCV Mat drawing
        P1 = cv::Point2f(rbtPos[0].x + surfaces[i].getP1().x,
                rbtPos[0].y - surfaces[i].getP1().y);
        P2 = cv::Point2f(rbtPos[0].x + surfaces[i].getP2().x,
                rbtPos[0].y - surfaces[i].getP2().y);
        cv::line(drawing, P1, P2, cv::Scalar(0, 0, 255), 3, 8, 0); // Draw the actual line
    }
    // Display the map in a file
    char filename[50];
    sprintf(filename, "%s%d%s", "../outputs/Maps/Map", M, ".jpg");
    cv::imwrite(filename, drawing);
    M++;
}

View Map::getView() {
    return currentView;
}

vector<View> Map::getMap() const {
    return this->map;
}
//mapping.

void Map::findReferenceSurface(const View & curView, Surface & tempSurf) {
    cout << "Looking for reference surface :)" << endl;
    vector<ReferenceSurfaces> sameSurfaces;
    SameSurfaceFinderOdo sSurfaceInfo;
    sSurfaceInfo.recognizeSameSurface(sameSurfaces, this->getLandmarkSurfaces(), curView.getSurfaces(), this->getPathSegments().back());
    if (sameSurfaces.size() > 0) {
        //this->setRefForNextLS(sameSurfaces[0]);
        //tempSurf = sameSurfaces[1];
        for (unsigned int i = 0; i < sameSurfaces.size(); i++) {
            sameSurfaces[i].display();
        }
        waitHere();
    }
}

void Map::expandMap(const View & curView) {
    //Surface refInMap = this->getPreviousView().getRPositionInPV();
    Surface refInMap = this->getPreviousView().getLandmarks().back();
    cout << "ref @previous LS " << endl;
    refInMap.display();
    //Surface refInCV(0, 0, 0, 40);
    Surface refInCV = curView.getLandmarks()[0];
    cout << "ref @current LS " << endl;
    refInCV.display();
    vector<Surface> cvSurfaces = curView.getSurfaces();
    cout << "cvSurf: " << cvSurfaces.size() << endl;
    vector<Surface> cvSurfacesOnMap = trangulateSurfaces(refInMap, refInCV, cvSurfaces);
    cout << "cvSurfOnMap: " << cvSurfacesOnMap.size() << endl;
    // vector<Surface> rpSurfaces = curView.getRobotSurfaces();
    // vector<Surface> rpInMap = trangulateSurfaces(refInMap, refInCV, rpSurfaces);
    // cout << "rpSurfOnMap: " << rpInMap.size() << endl;
    // displaySurfaces(rpInMap);
    View newView;
    newView.setSurfaces(cvSurfacesOnMap);
    // newView.setRobotSurfaces(rpInMap);
    this->map.push_back(newView);
}

vector<Surface> Map::ClearCloseSurfaces(const vector<Surface>& robotSurfaces) {
    vector<Surface> surfacesCleaned;
    vector<Surface> surfacesForPointInPolygon; // Contains the robot position in place of deleted surfaces
    cv::Point2f robotPos = robotSurfaces[0].getP1();
    View lastView = this->map[map.size() - 1];
    vector<Surface> lastSurfaces = lastView.getSurfaces();
    for (unsigned int i = 0; i < lastSurfaces.size(); i++) {
        double distFromRobot = lastSurfaces[i].distFromP1ToPoint(robotPos.x, robotPos.y);
        double distFromRobot2 = lastSurfaces[i].distFromP2ToPoint(robotPos.x, robotPos.y);
        if (distFromRobot > MIN_DISTANCE_VISION && distFromRobot2 > MIN_DISTANCE_VISION) {
            // Keep new surfaces more than MIN_DISTANCE_VISION away
            surfacesCleaned.push_back(lastSurfaces[i]);
            surfacesForPointInPolygon.push_back(lastSurfaces[i]);
        } else {
            // Ignore new surfaces less than MIN_DISTANCE_VISION away and replace them by robotPos so the PointInPolygon still work
            if (i > 0 && i < lastSurfaces.size() - 1) {
                surfacesForPointInPolygon.push_back(Surface(robotPos.x, robotPos.y, robotPos.x, robotPos.y));
            }
        }
    }
    lastView.setSurfaces(surfacesCleaned);
    this->map[this->map.size() - 1] = lastView;
    return surfacesForPointInPolygon;
}

void Map::addViewProjectingGapOnLongSurf(View& curView, Surface gapInCV, Surface gapInPV, Surface longSurfInCV, Surface longSurfInPV) {

    //Project the nearest point of the gap on the long surface (CV)
    double distGapP1CV = longSurfInCV.distFromPoint(gapInCV.getP1().x, gapInCV.getP1().y);
    double distGapP2CV = longSurfInCV.distFromPoint(gapInCV.getP2().x, gapInCV.getP2().y);
    PointXY projectedPtCV;

    if (distGapP1CV < distGapP2CV) {
        projectedPtCV = longSurfInCV.projectPointOnSurface(gapInCV.getP1().x, gapInCV.getP1().y);
    } else {
        projectedPtCV = longSurfInCV.projectPointOnSurface(gapInCV.getP2().x, gapInCV.getP2().y);
    }

    //Update the surface to set the projected point as an endpoint
    double distSurfP1ToProjected = longSurfInCV.distFromP1ToPoint(projectedPtCV.getX(), projectedPtCV.getY());
    double distSurfP2ToProjected = longSurfInCV.distFromP2ToPoint(projectedPtCV.getX(), projectedPtCV.getY());
    int refPointCV;

    if (distSurfP1ToProjected < distSurfP2ToProjected) {
        longSurfInCV = Surface(projectedPtCV.getX(), projectedPtCV.getY(), longSurfInCV.getP2().x, longSurfInCV.getP2().y);
        refPointCV = 1;
    } else {
        longSurfInCV = Surface(longSurfInCV.getP1().x, longSurfInCV.getP1().y, projectedPtCV.getX(), projectedPtCV.getY());
        refPointCV = 2;
    }

    //Plot projected point
    /*char CVplot[50];
    sprintf(CVplot, "%s%d%s", "../outputs/Maps/projectedCV-", curView.getId(), ".png");
    vector<PointXY> cvPts;
    cvPts.push_back(projectedPtCV);
    plotPointsAndSurfacesGNU(CVplot, cvPts, curView.getSurfaces());*/

    //Project the nearest point of the gap on the long surface (PV)
    double distGapP1PV = longSurfInPV.distFromPoint(gapInPV.getP1().x, gapInPV.getP1().y);
    double distGapP2PV = longSurfInPV.distFromPoint(gapInPV.getP2().x, gapInPV.getP2().y);

    PointXY projectedPtPV;

    if (distGapP1PV < distGapP2PV) {
        projectedPtPV = longSurfInPV.projectPointOnSurface(gapInPV.getP1().x, gapInPV.getP1().y);
    } else {
        projectedPtPV = longSurfInPV.projectPointOnSurface(gapInPV.getP2().x, gapInPV.getP2().y);
    }

    /*
    char PVplot[50];
    sprintf(PVplot, "%s%d%s", "../outputs/Maps/projectedPV-", curView.getId(), ".png");
    vector<PointXY> cvPtsPV;
    cvPtsPV.push_back(projectedPtPV);
    vector<Surface> PVSurf;
    PVSurf.push_back(longSurfInPV);
    plotPointsAndSurfacesGNU(PVplot, cvPtsPV, PVSurf);*/

    //Update the surface to set the projected point as an endpoint
    double distSurfP1ToProjectedPV = longSurfInPV.distFromP1ToPoint(projectedPtPV.getX(), projectedPtPV.getY());
    double distSurfP2ToProjectedPV = longSurfInPV.distFromP2ToPoint(projectedPtPV.getX(), projectedPtPV.getY());
    int refPointPV;

    if (distSurfP1ToProjectedPV < distSurfP2ToProjectedPV) {
        longSurfInPV = Surface(projectedPtPV.getX(), projectedPtPV.getY(), longSurfInPV.getP2().x, longSurfInPV.getP2().y);
        refPointPV = 1;
    } else {
        longSurfInPV = Surface(longSurfInPV.getP1().x, longSurfInPV.getP1().y, projectedPtPV.getX(), projectedPtPV.getY());
        refPointPV = 2;
    }

    //Adjust endpoint order
    int refPoint = refPointCV;
    if (refPointCV != refPointPV) {
        longSurfInPV = Surface(longSurfInPV.getP2().x, longSurfInPV.getP2().y, longSurfInPV.getP1().x, longSurfInPV.getP1().y);
    }

    //Take long surfaces as reference to match the views
    ReferenceSurfaces refPair;
    refPair.setMapSurface(longSurfInPV);
    refPair.setViewSurface(longSurfInCV);
    refPair.setRefPoint(refPoint);

    //Calculate the view surfaces in map's coordinates
    Surface cvSurfaceOnMap;
    vector<Surface> allCVSurfacesOnMap;
    for (unsigned int i = 0; i < curView.getSurfaces().size(); i++) {
        cvSurfaceOnMap = trangulateSurface(refPair.getMapSurface(), refPair.getViewSurface(),
                curView.getSurfaces()[i], refPair.getRefPoint());
        allCVSurfacesOnMap.push_back(cvSurfaceOnMap);
    }
    //Calculate the robot surfaces in map's coordinates
    Surface cRobotSurfaceOnMap;
    vector<Surface> cRobotSurfacesOnMap;
    for (unsigned int i = 0; i < curView.getRobotSurfaces().size(); i++) {
        cRobotSurfaceOnMap = trangulateSurface(refPair.getMapSurface(), refPair.getViewSurface(),
                curView.getRobotSurfaces()[i], refPair.getRefPoint());
        cRobotSurfacesOnMap.push_back(cRobotSurfaceOnMap);
    }
    View cViewOnMap;
    cViewOnMap.setSurfaces(allCVSurfacesOnMap);
    cViewOnMap.setRobotSurfaces(cRobotSurfacesOnMap);
    cViewOnMap.setId(curView.getId());
    cViewOnMap.setHasGap(curView.getHasGap());

    this->addCvAndClean(cViewOnMap);
}

Surface Map::findLongSurfaceInCV(View& curView, View& prevView) {
    //Find the distance after which surfaces of current view can't be seen in previous view
    View odoView = this->computeCVUsingOdometer(curView);
    cv::Point2f expectedRbtPos = odoView.getRobotSurfaces()[0].getP1();
    double maxDistance = 0;
    for (unsigned int i = 0; i < prevView.getSurfaces().size(); i++) {
        double dist = max(prevView.getSurfaces()[i].distFromP1ToPoint(expectedRbtPos.x, expectedRbtPos.y), prevView.getSurfaces()[i].distFromP2ToPoint(expectedRbtPos.x, expectedRbtPos.y));
        if (dist > maxDistance) {
            maxDistance = dist;
        }
    }

    //Compute the longest surface in current view that is closer than maxDistance 
    cv::Point2f robotPos = curView.getRobotSurfaces()[0].getP1();
    vector<PointXY> surfacePts;
    surfacePts.push_back(PointXY(curView.getSurfaces()[0].getP1().x, curView.getSurfaces()[0].getP1().y));
    surfacePts.push_back(PointXY(curView.getSurfaces()[0].getP2().x, curView.getSurfaces()[0].getP2().y));
    double maxLength = curView.getSurfaces()[0].length();
    for (unsigned int i = 1; i < curView.getSurfaces().size(); i++) {
        if (min(curView.getSurfaces()[i].distFromP1ToPoint(robotPos.x, robotPos.y), curView.getSurfaces()[i].distFromP2ToPoint(robotPos.x, robotPos.y)) < maxDistance) {
            Surface lastSurface = Surface(surfacePts[surfacePts.size() - 2].getX(), surfacePts[surfacePts.size() - 2].getY(), surfacePts[surfacePts.size() - 1].getX(), surfacePts[surfacePts.size() - 1].getY());
            double angle = curView.getSurfaces()[i].getAngleWithSurface(lastSurface);
            if (lastSurface.distFromP2ToPoint(curView.getSurfaces()[i].getP1().x, curView.getSurfaces()[i].getP1().y) < 600
                    && ((int) abs(angle) % 180 < 45 || (int) abs(angle) % 180 > 135)) {
                surfacePts.push_back(PointXY(curView.getSurfaces()[i].getP1().x, curView.getSurfaces()[i].getP1().y));
                surfacePts.push_back(PointXY(curView.getSurfaces()[i].getP2().x, curView.getSurfaces()[i].getP2().y));
                if (curView.getSurfaces()[i].length() > maxLength) {
                    maxLength = curView.getSurfaces()[i].length();
                }
            } else if (curView.getSurfaces()[i].length() > maxLength) {
                maxLength = curView.getSurfaces()[i].length();
                surfacePts.clear();
                surfacePts.push_back(PointXY(curView.getSurfaces()[i].getP1().x, curView.getSurfaces()[i].getP1().y));
                surfacePts.push_back(PointXY(curView.getSurfaces()[i].getP2().x, curView.getSurfaces()[i].getP2().y));
            }
        }

    }

    return principalComponentAnalysis(surfacePts);
}

void Map::BuildMap(char* dataset, int firstView, int numSteps, Map *lastMap) {

    Robot Albot;
    View curView;
    curView.setRobotSurfaces(Albot.getRectRobot());

    char viewName[50], pointFile[50];

    for (unsigned int i = 0; i < numSteps; i++) {

        //construct view from points.
        curView.setId(firstView + i);
        sprintf(pointFile, "%s%s%s%d", "../inputs/", dataset, "/pointCloud/points2D-", curView.getId());
        curView.constructView(pointFile);

        cout << "View is formed :)" << endl;
        cout << endl << "==================================================" << endl << endl;
        cout << "View no. " << curView.getId() << ":" << endl;
        sprintf(viewName, "%s%d", "../outputs/Maps/view-", curView.getId());
        plotViewGNU(viewName, curView);

        vector<Surface> boundaries = curView.findBoundaries();
        char plotBoundaries[50];
        sprintf(plotBoundaries, "%s%d%s", "../outputs/Maps/boundaries-", curView.getId(), ".png");
        plotSurfacesGNU(plotBoundaries, boundaries);

        //Using odometer
        if (i == 0) {
            this->initializeMap(curView);
        } else {

            //Add view to map
            View viewOnMap = this->computeCVUsingOdometer(curView);
            this->addCvAndClean(viewOnMap);
        }

        //read odometer info
        sprintf(viewName, "%s%s%s%d", "../inputs/", dataset, "/surfaces/coordTrans-", curView.getId());
        readOdometry(Albot, viewName);
        this->addPathSegment(Albot.getLastLocomotion());
        this->setLandmarkSurfaces(curView.getSurfaces());

        if (i == 0 && lastMap != 0) {
            this->computeEntrance(*lastMap);
            this->addSurfacesAfterEntrance(*lastMap);
        }
    }
    cout << endl;

    this->computeExit();
}

vector<double> findBoundariesOfCV(const vector<Surface> & cvSurfaces, double extension) {
    double leftBoundaryX = 0;
    double leftBoundaryY = 0;
    double rightBoundaryX = 0;
    double rightBoundaryY = 0;
    double frontBoundary = 0;
    for (int i = 0; i<int(cvSurfaces.size()); i++) {
        if (cvSurfaces[i].getP1().x < leftBoundaryX) {
            leftBoundaryX = cvSurfaces[i].getP1().x;
            leftBoundaryY = cvSurfaces[i].getP1().y;
        }
        if (cvSurfaces[i].getP2().x < leftBoundaryX) {
            leftBoundaryX = cvSurfaces[i].getP2().x;
            leftBoundaryY = cvSurfaces[i].getP2().y;
        }
        if (cvSurfaces[i].getP1().x > rightBoundaryX) {
            rightBoundaryX = cvSurfaces[i].getP1().x;
            rightBoundaryY = cvSurfaces[i].getP1().y;
        }
        if (cvSurfaces[i].getP2().x > rightBoundaryX) {
            rightBoundaryX = cvSurfaces[i].getP2().x;
            rightBoundaryY = cvSurfaces[i].getP2().y;
        }
        if (cvSurfaces[i].getP1().y > frontBoundary)
            frontBoundary = cvSurfaces[i].getP1().y;
        if (cvSurfaces[i].getP2().y > frontBoundary)
            frontBoundary = cvSurfaces[i].getP2().y;
    }
    vector<double> result;
    result.push_back(leftBoundaryX - extension);
    result.push_back(leftBoundaryY - extension);
    result.push_back(rightBoundaryX + extension);
    result.push_back(rightBoundaryY - extension);
    result.push_back(frontBoundary + extension);
    return result;
}
//construct polygon from a set of surfaces

vector<SurfaceT> constructRectangle(const vector<Surface> & transformedSurfaces) {
    vector<SurfaceT> polygon;
    for (unsigned int i = 0; i < transformedSurfaces.size() - 1; i++) {
        //surface i.
        polygon.push_back(transformedSurfaces[i].ToSurfaceT());
    }
    return polygon;
}
//construct polygon from a set of surfaces

vector<SurfaceT> constructPolygon(const vector<Surface> & transformedSurfaces, const vector<Surface> & robotSurfaces) {
    vector<SurfaceT> polygon;
    //from robotPosition to first surface.
    polygon.push_back(SurfaceT(PointXY((double) robotSurfaces[0].getP1().x, (double) robotSurfaces[0].getP1().y),
            PointXY((double) transformedSurfaces[0].getP1().x, (double) transformedSurfaces[0].getP1().y)));
    for (unsigned int i = 0; i < transformedSurfaces.size() - 1; i++) {
        //surface i.
        polygon.push_back(transformedSurfaces[i].ToSurfaceT());
        //gap or surface i p2 to surface i+1 p1
        polygon.push_back(SurfaceT(PointXY((double) transformedSurfaces[i].getP2().x, (double) transformedSurfaces[i].getP2().y),
                PointXY((double) transformedSurfaces[i + 1].getP1().x, (double) transformedSurfaces[i + 1].getP1().y)));
    }
    //last surface.
    polygon.push_back(transformedSurfaces.back().ToSurfaceT());
    //last gap or last surface p2 to robotp1
    //gap or surface i p2 to surface i+1 p1
    polygon.push_back(SurfaceT(PointXY((double) transformedSurfaces.back().getP2().x, (double) transformedSurfaces.back().getP2().y),
            PointXY((double) robotSurfaces[0].getP1().x, (double) robotSurfaces[0].getP1().y)));
    return polygon;
}
//it finds the closest ref pair where cvSurfaces and viewSurface in RefSurfaces are in same co-ordinate.

ReferenceSurfaces findTheClosestReference(Surface & cvSurface, vector<ReferenceSurfaces> allRefSurfaces) {
    double closestDist, tempDist;
    SurfaceT cvSurfaceT = cvSurface.ToSurfaceT();
    closestDist = distBetweenSurfs(cvSurfaceT, allRefSurfaces[0].getViewSurface().ToSurfaceT());
    ReferenceSurfaces refPair = allRefSurfaces[0];
    for (unsigned int i = 1; i < allRefSurfaces.size(); i++) {
        tempDist = distBetweenSurfs(cvSurfaceT, allRefSurfaces[i].getViewSurface().ToSurfaceT());
        if (tempDist < closestDist) {
            refPair = allRefSurfaces[i];
            closestDist = tempDist;
        }
    }
    return refPair;
}
//test
//vector<Surface> temp;
// temp.push_back(curView.getRobotSurfaces()[0]);
// double angle, dist;
// for(int i=0; i<10; i++) {
// cout<<"angle ";
// cin >>angle;
// cout<<"dist ";
// cin >> dist;
// temp.push_back(makeSurfaceWith(curView.getRobotSurfaces()[0], angle, dist, 400));
// plotSurfacesGNU("../outputs/Maps/test.png",temp);
// waitHere();
// }


//it computes a surface of lenght(given) at distance (given) from p1 of refInMap in the direction of angle (given)

Surface makeSurfaceWith(const Surface & refInMap, double angle, double distance, double length) {
    //function has been corrected by rotating ref surf at the beginning.
    Surface refAfterRotation = refInMap;
    refAfterRotation.rotateAroundP1(angle);
    double x1, y1, x2, y2;
    double ang = 0;
    x1 = ((refAfterRotation.getP2().x - refAfterRotation.getP1().x) / refAfterRotation.length()) * cos(ang)-((refAfterRotation.getP2().y - refAfterRotation.getP1().y) / refAfterRotation.length()) * sin(ang);
    y1 = ((refAfterRotation.getP2().x - refAfterRotation.getP1().x) / refAfterRotation.length()) * sin(ang)+((refAfterRotation.getP2().y - refAfterRotation.getP1().y) / refAfterRotation.length()) * cos(ang);
    x1 = x1 * distance + refAfterRotation.getP1().x;
    y1 = y1 * distance + refAfterRotation.getP1().y;
    distance += length;
    x2 = ((refAfterRotation.getP2().x - refAfterRotation.getP1().x) / refAfterRotation.length()) * cos(ang)-((refAfterRotation.getP2().y - refAfterRotation.getP1().y) / refAfterRotation.length()) * sin(ang);
    y2 = ((refAfterRotation.getP2().x - refAfterRotation.getP1().x) / refAfterRotation.length()) * sin(ang)+((refAfterRotation.getP2().y - refAfterRotation.getP1().y) / refAfterRotation.length()) * cos(ang);
    x2 = x2 * distance + refAfterRotation.getP1().x;
    y2 = y2 * distance + refAfterRotation.getP1().y;
    Surface result(x1, y1, x2, y2);
    //result.rotateAroundP1(angle);
    return result;
}

Surface trangulateSurface(const Surface & refInMap, const Surface & refInCV, const Surface & cvSurface, const int & refPoint) {
    double angle, distance;
    float x1, y1, x2, y2;
    if (refPoint == 1) {
        angle = refInCV.getAngleFromP1ToPoint(cvSurface.getP1().x, cvSurface.getP1().y);
        angle *= CONVERT_TO_RADIAN;
        distance = refInCV.distFromP1ToPoint(cvSurface.getP1().x, cvSurface.getP1().y);
        x1 = ((refInMap.getP2().x - refInMap.getP1().x) / refInMap.length()) * cos(angle)-((refInMap.getP2().y - refInMap.getP1().y) / refInMap.length()) * sin(angle);
        y1 = ((refInMap.getP2().x - refInMap.getP1().x) / refInMap.length()) * sin(angle)+((refInMap.getP2().y - refInMap.getP1().y) / refInMap.length()) * cos(angle);
        x1 = x1 * distance + refInMap.getP1().x;
        y1 = y1 * distance + refInMap.getP1().y;
        angle = refInCV.getAngleFromP1ToPoint(cvSurface.getP2().x, cvSurface.getP2().y);
        angle *= CONVERT_TO_RADIAN;
        distance = refInCV.distFromP1ToPoint(cvSurface.getP2().x, cvSurface.getP2().y);
        x2 = ((refInMap.getP2().x - refInMap.getP1().x) / refInMap.length()) * cos(angle)-((refInMap.getP2().y - refInMap.getP1().y) / refInMap.length()) * sin(angle);
        y2 = ((refInMap.getP2().x - refInMap.getP1().x) / refInMap.length()) * sin(angle)+((refInMap.getP2().y - refInMap.getP1().y) / refInMap.length()) * cos(angle);
        x2 = x2 * distance + refInMap.getP1().x;
        y2 = y2 * distance + refInMap.getP1().y;
    } else {
        angle = refInCV.getAngleFromP2ToPoint(cvSurface.getP1().x, cvSurface.getP1().y);
        angle *= CONVERT_TO_RADIAN;
        distance = refInCV.distFromP2ToPoint(cvSurface.getP1().x, cvSurface.getP1().y);
        x1 = ((refInMap.getP1().x - refInMap.getP2().x) / refInMap.length()) * cos(angle)-((refInMap.getP1().y - refInMap.getP2().y) / refInMap.length()) * sin(angle);
        y1 = ((refInMap.getP1().x - refInMap.getP2().x) / refInMap.length()) * sin(angle)+((refInMap.getP1().y - refInMap.getP2().y) / refInMap.length()) * cos(angle);
        x1 = x1 * distance + refInMap.getP2().x;
        y1 = y1 * distance + refInMap.getP2().y;
        angle = refInCV.getAngleFromP2ToPoint(cvSurface.getP2().x, cvSurface.getP2().y);
        angle *= CONVERT_TO_RADIAN;
        distance = refInCV.distFromP2ToPoint(cvSurface.getP2().x, cvSurface.getP2().y);
        x2 = ((refInMap.getP1().x - refInMap.getP2().x) / refInMap.length()) * cos(angle)-((refInMap.getP1().y - refInMap.getP2().y) / refInMap.length()) * sin(angle);
        y2 = ((refInMap.getP1().x - refInMap.getP2().x) / refInMap.length()) * sin(angle)+((refInMap.getP1().y - refInMap.getP2().y) / refInMap.length()) * cos(angle);
        x2 = x2 * distance + refInMap.getP2().x;
        y2 = y2 * distance + refInMap.getP2().y;
    }
    return Surface(x1, y1, x2, y2);
}

vector<Surface> trangulateSurfaces(const Surface & refInMap, const Surface & refInCV, const vector<Surface>& cvSurfaces) {
    vector<Surface> cvSurfacesOnMap;
    for (unsigned int i = 0; i < cvSurfaces.size(); i++) {
        cvSurfacesOnMap.push_back(trangulateSurface(refInMap, refInCV, cvSurfaces[i], 1));
    }
    return cvSurfacesOnMap;
}
//it finds anypoint (pointX, pointY) is inside or outside of the polygon (surfaces, and robotPosition).
//if any point lies on the vertex of the polygon or on the edge of the polygon then it returns false.
// http://stackoverflow.com/questions/11716268/point-in-polygon-algorithm
// http://www.ecse.rpi.edu/~wrf/Research/Short_Notes/pnpoly.html

bool PointInPolygon(const double & pointX, const double & pointY, const vector<Surface>& surfaces,
        const vector<Surface>& robotSurfaces) {
    vector<PointXY> points;
    points.push_back(PointXY(robotSurfaces[0].getP1().x, robotSurfaces[0].getP1().y));
    for (unsigned int i = 0; i < surfaces.size(); i++) {
        points.push_back(PointXY(surfaces[i].getP1().x, surfaces[i].getP1().y));
        points.push_back(PointXY(surfaces[i].getP2().x, surfaces[i].getP2().y));
    }
    // int i, j, nvert = points.size();
    // bool c = false;
    //
    // for(i = 0, j = nvert - 1; i < nvert; j = i++) {
    // if( ( (points[i].getY() >= pointY ) != (points[j].getY() >= pointY) ) &&
    // (pointX <= (points[j].getX() - points[i].getX()) * (pointY - points[i].getY()) / (points[j].getY() - points[i].getY()) + points[i].getX())
    // )
    // c = !c;
    // }
    //
    // return c;
    return pointInPolygon(pointX, pointY, points);
}

bool PointHiddenBySurfaces(const cv::Point2f pointToCheck, const vector<Surface>& allSurfaces, const vector<Surface>& robotSurfaces) {
    if (allSurfaces.size() > 0) {
        cv::Point2f robotPos = robotSurfaces[0].getP1();
        vector<Surface> adjacentSurfaces;
        adjacentSurfaces.push_back(allSurfaces[0]);
        for (unsigned int i = 1; i < allSurfaces.size(); i++) {
            if (adjacentSurfaces[adjacentSurfaces.size() - 1].getP2() == allSurfaces[i].getP1()) {
                adjacentSurfaces.push_back(allSurfaces[i]);
                continue;
            }
            if (!PointInPolygon(pointToCheck.x, pointToCheck.y, adjacentSurfaces, robotSurfaces)) {
                // Test whether the point is in the right angle range to be hidden by adjacentSurfaces
                Surface s1 = Surface(robotPos.x, robotPos.y, adjacentSurfaces[0].getP1().x, adjacentSurfaces[0].getP1().y);
                Surface s2 = Surface(robotPos.x, robotPos.y, adjacentSurfaces[adjacentSurfaces.size() - 1].getP2().x, adjacentSurfaces[adjacentSurfaces.size() - 1].getP2().y);
                Surface stest = Surface(robotPos.x, robotPos.y, pointToCheck.x, pointToCheck.y);
                double stestAngle = stest.getAngleWithXaxis();
                double diffAngle = s1.getAngleFromP1ToPoint(s2.getP2().x, s2.getP2().y);
                if ((diffAngle > 180 && stestAngle <= s1.getAngleWithXaxis() && stestAngle >= s2.getAngleWithXaxis())
                        || (diffAngle < 180 && stestAngle >= s1.getAngleWithXaxis() && stestAngle <= s2.getAngleWithXaxis())) {
                    return true;
                }
                adjacentSurfaces.clear();
                adjacentSurfaces.push_back(allSurfaces[i]);
            }
        }
    }
    return false;
}

bool SurfaceHidingPoint(const cv::Point2f pointToCheck, const Surface& surface, const vector<Surface>& robotSurfaces) {
    cv::Point2f robotPos = robotSurfaces[0].getP1();
    vector<Surface> vectSurfaces;
    vectSurfaces.push_back(surface);
    if (!PointInPolygon(pointToCheck.x, pointToCheck.y, vectSurfaces, robotSurfaces)) {
        // Test whether the point is in the right angle range to be hidden by adjacentSurfaces
        Surface s1 = Surface(robotPos.x, robotPos.y, surface.getP1().x, surface.getP1().y);
        Surface s2 = Surface(robotPos.x, robotPos.y, surface.getP2().x, surface.getP2().y);
        Surface stest = Surface(robotPos.x, robotPos.y, pointToCheck.x, pointToCheck.y);
        double stestAngle = stest.getAngleWithXaxis();
        double diffAngle = s1.getAngleFromP1ToPoint(s2.getP2().x, s2.getP2().y);
        if ((diffAngle > 180 && stestAngle <= s1.getAngleWithXaxis() && stestAngle >= s2.getAngleWithXaxis())
                || (diffAngle < 180 && stestAngle >= s1.getAngleWithXaxis() && stestAngle <= s2.getAngleWithXaxis())) {
            return true;
        }
    }
    return false;
}

bool compare_double(const pair<double, PointXY>& a, pair<double, PointXY>& b) {
    return a.first < b.first;
}

Surface findExit(const vector<Surface>& surfaces, const cv::Point2f rbtPos, const bool takeBorderPoints, const Surface *otherExit, ReferenceSurfaces *refSurfacePair) {

    list<pair<double, PointXY> > possibleLeftEndpts;
    list<pair<double, PointXY> > possibleRightEndpts;

    for (unsigned int i = 1; i < surfaces.size() - 1; i++) {
        // Left endpoint ? Yes if the next surface is further away
        double distP1next = surfaces[i + 1].distFromP1ToPoint(rbtPos.x, rbtPos.y);
        double distP2cur = surfaces[i].distFromP2ToPoint(rbtPos.x, rbtPos.y);
        bool nextSurfFurtherAway = distP1next > distP2cur || (surfaces[i + 1].distFromP1ToPoint(surfaces[i].getP2().x, surfaces[i].getP2().y) < 100 && surfaces[i + 1].distFromP2ToPoint(rbtPos.x, rbtPos.y) > distP2cur);

        if (nextSurfFurtherAway) {
            possibleLeftEndpts.push_back(make_pair(surfaces[i].distFromP2ToPoint(rbtPos.x, rbtPos.y), PointXY(surfaces[i].getP2().x, surfaces[i].getP2().y)));
        }

        //Right endpoint ? Yes if the previous surface is further away
        double distP2prev = surfaces[i - 1].distFromP2ToPoint(rbtPos.x, rbtPos.y);
        double distP1cur = surfaces[i].distFromP1ToPoint(rbtPos.x, rbtPos.y);
        bool prevSurfFurtherAway = distP2prev > distP1cur || (surfaces[i - 1].distFromP2ToPoint(surfaces[i].getP1().x, surfaces[i].getP1().y) < 100 && surfaces[i - 1].distFromP1ToPoint(rbtPos.x, rbtPos.y) > distP1cur);

        if (prevSurfFurtherAway) {
            possibleRightEndpts.push_back(make_pair(surfaces[i].distFromP1ToPoint(rbtPos.x, rbtPos.y), PointXY(surfaces[i].getP1().x, surfaces[i].getP1().y)));
        }
    }
    if (takeBorderPoints) {
        //First surface
        double distP11 = surfaces[1].distFromP1ToPoint(rbtPos.x, rbtPos.y);
        double distP20 = surfaces[0].distFromP2ToPoint(rbtPos.x, rbtPos.y);
        bool secondSurfFurtherAway = distP11 > distP20 || (surfaces[1].distFromP1ToPoint(surfaces[0].getP2().x, surfaces[0].getP2().y) < 100 && surfaces[1].distFromP2ToPoint(rbtPos.x, rbtPos.y) > distP20);
        if (secondSurfFurtherAway) {
            possibleLeftEndpts.push_back(make_pair(surfaces[0].distFromP2ToPoint(rbtPos.x, rbtPos.y), PointXY(surfaces[0].getP2().x, surfaces[0].getP2().y)));
        }

        //Last surface
        double distP2Before = surfaces[surfaces.size() - 2].distFromP2ToPoint(rbtPos.x, rbtPos.y);
        double distP1Last = surfaces[surfaces.size() - 1].distFromP1ToPoint(rbtPos.x, rbtPos.y);
        bool surfBeforeFurtherAway = distP2Before > distP1Last || (surfaces[surfaces.size() - 2].distFromP2ToPoint(surfaces[surfaces.size() - 1].getP1().x, surfaces[surfaces.size() - 1].getP1().y) < 100 && surfaces[surfaces.size() - 2].distFromP1ToPoint(rbtPos.x, rbtPos.y) > distP1Last);
        if (surfBeforeFurtherAway) {
            possibleRightEndpts.push_back(make_pair(surfaces[surfaces.size() - 1].distFromP1ToPoint(rbtPos.x, rbtPos.y), PointXY(surfaces[surfaces.size() - 1].getP1().x, surfaces[surfaces.size() - 2].getP1().y)));
        }
    }

    if (possibleLeftEndpts.size() == 0 || possibleRightEndpts.size() == 0) {
        //Not enough endpoints
        throw false;
    }

    possibleLeftEndpts.sort(compare_double);
    possibleRightEndpts.sort(compare_double);

    //Plotting
    /* char possibleEndptsPlot [80];
     cout << "file " << surfaces[0].getP1().y << endl;
     sprintf(possibleEndptsPlot, "%s%f%s", "../outputs/Maps/possibleEndpts-", surfaces[0].getP1().y, "-left.png");
     vector<PointXY> tmp;
     for (list<pair<double, PointXY> >::iterator it = possibleLeftEndpts.begin(); it != possibleLeftEndpts.end(); it++) {
         tmp.push_back(it->second);
     }
     plotPointsAndSurfacesGNU(possibleEndptsPlot, tmp, surfaces);
     sprintf(possibleEndptsPlot, "%s%f%s", "../outputs/Maps/possibleEndpts-", surfaces[0].getP1().y, "-right.png");
     vector<PointXY> tmp2;
     for (list<pair<double, PointXY> >::iterator it = possibleRightEndpts.begin(); it != possibleRightEndpts.end(); it++) {
         tmp2.push_back(it->second);
     }
     plotPointsAndSurfacesGNU(possibleEndptsPlot, tmp2, surfaces);*/

    //Test possible exits with the endpoints found
    list<pair<double, PointXY> > rightEndptsConsulted;
    list<pair<double, PointXY> > leftEndptsConsulted;

    while (possibleLeftEndpts.size() > 0) {
        pair<double, PointXY> minLeft = possibleLeftEndpts.front();
        possibleLeftEndpts.pop_front();
        pair<double, PointXY> nextLeft;
        bool isLastLeftEndpt = true;
        if (possibleLeftEndpts.size() > 0) {
            nextLeft = possibleLeftEndpts.front();
            isLastLeftEndpt = false;
        }

        //Test the closest left endpoints with every right endpoint already tested
        list<pair<double, PointXY> >::iterator rightEndptsConsultedIt = rightEndptsConsulted.begin();
        while (rightEndptsConsultedIt != rightEndptsConsulted.end()) {
            Surface exit(minLeft.second.getX(), minLeft.second.getY(), rightEndptsConsultedIt->second.getX(), rightEndptsConsultedIt->second.getY());
            if (isExit(exit, PointXY(rbtPos.x, rbtPos.y), surfaces)) {
                if (otherExit != 0 && refSurfacePair != 0) {
                    Surface exitOnMapOdo = trangulateSurface(refSurfacePair->getMapSurface(), refSurfacePair->getViewSurface(),
                            *otherExit, refSurfacePair->getRefPoint());
                    unsigned int angle = (int) abs(exitOnMapOdo.getAngleWithSurface(exit));
                    if (angle % 180 < 50 || angle % 180 > 130) {
                        return exit;
                    }
                } else {
                    return exit;
                }
            }
            rightEndptsConsultedIt++;
        }

        leftEndptsConsulted.push_back(minLeft);

        //Test the closest left endpoint with every right endpoints closer than the next left endpoint 
        list<pair<double, PointXY> >::iterator rightEndptsIt = possibleRightEndpts.begin();
        while ((isLastLeftEndpt || rightEndptsIt->first < nextLeft.first) && rightEndptsIt != possibleRightEndpts.end()) {
            list<pair<double, PointXY> >::iterator leftEndptsConsultedIt = leftEndptsConsulted.begin();
            while (leftEndptsConsultedIt != leftEndptsConsulted.end()) {
                Surface exit(leftEndptsConsultedIt->second.getX(), leftEndptsConsultedIt->second.getY(), rightEndptsIt->second.getX(), rightEndptsIt->second.getY());
                if (isExit(exit, PointXY(rbtPos.x, rbtPos.y), surfaces)) {
                    if (otherExit != 0 && refSurfacePair != 0) {
                        Surface exitOnMapOdo = trangulateSurface(refSurfacePair->getMapSurface(), refSurfacePair->getViewSurface(),
                                *otherExit, refSurfacePair->getRefPoint());
                        unsigned int angle = (int) abs(exitOnMapOdo.getAngleWithSurface(exit));
                        if (angle % 180 < 50 || angle % 180 > 130) {
                            return exit;
                        }
                    } else {
                        return exit;
                    }
                }
                leftEndptsConsultedIt++;
            }

            rightEndptsConsulted.push_back(*rightEndptsIt);
            rightEndptsIt++;
        }
    }
    //No exit found
    throw false;
}

bool isExit(const Surface& exit, const PointXY& rbtPos, const vector<Surface>& surfaces) {

    double length = exit.length();

    if (length > 500) {
        //Compute exit vectors
        PointXY exitVect(exit.getP2().x - exit.getP1().x, exit.getP2().y - exit.getP1().y);
        exitVect = exitVect / sqrt(exitVect.getX() * exitVect.getX() + exitVect.getY() * exitVect.getY());
        PointXY orthoVect(-exitVect.getY(), exitVect.getX());
        orthoVect = orthoVect / sqrt(orthoVect.getX() * orthoVect.getX() + orthoVect.getY() * orthoVect.getY());

        //Compute the rectangle that must not be intersected by surfaces or contain any
        vector<PointXY> rectangle;
        double rectHalfWidth = 250;
        double rectHalfLength = 500;
        rectangle.push_back(PointXY(exit.midPoint().x - rectHalfLength * orthoVect.getX() - rectHalfWidth * exitVect.getX(), exit.midPoint().y - rectHalfLength * orthoVect.getY() - rectHalfWidth * exitVect.getY()));
        rectangle.push_back(PointXY(exit.midPoint().x + rectHalfLength * orthoVect.getX() - rectHalfWidth * exitVect.getX(), exit.midPoint().y + rectHalfLength * orthoVect.getY() - rectHalfWidth * exitVect.getY()));
        rectangle.push_back(PointXY(exit.midPoint().x + rectHalfLength * orthoVect.getX() + rectHalfWidth * exitVect.getX(), exit.midPoint().y + rectHalfLength * orthoVect.getY() + rectHalfWidth * exitVect.getY()));
        rectangle.push_back(PointXY(exit.midPoint().x - rectHalfLength * orthoVect.getX() + rectHalfWidth * exitVect.getX(), exit.midPoint().y - rectHalfLength * orthoVect.getY() + rectHalfWidth * exitVect.getY()));

        PointXY gapUpMiddle(exit.midPoint().x + rectHalfLength * orthoVect.getX(), exit.midPoint().y + rectHalfLength * orthoVect.getY());
        PointXY gapDownMiddle(exit.midPoint().x - rectHalfLength * orthoVect.getX(), exit.midPoint().y - rectHalfLength * orthoVect.getY());

        //Compute the rectangle that must be empty to access the left side of the gap
        vector<PointXY> leftAccess;
        leftAccess.push_back(rbtPos);
        leftAccess.push_back(rectangle[0]);
        leftAccess.push_back(rectangle[1]);
        leftAccess.push_back(gapUpMiddle);
        leftAccess.push_back(gapDownMiddle);

        //Compute the rectangle that must be empty to access the right side of the gap
        vector<PointXY> rightAccess;
        rightAccess.push_back(rbtPos);
        rightAccess.push_back(gapDownMiddle);
        rightAccess.push_back(gapUpMiddle);
        rightAccess.push_back(rectangle[2]);
        rightAccess.push_back(rectangle[3]);

        //Check if the exit matches all the criterias
        bool surfInside = false;
        bool isIntersecting = false;
        bool isAccessible = true;
        for (unsigned int k = 0; k < surfaces.size(); k++) {
            if (surfaceIntersectsPolygon(surfaces[k], rectangle)
                    || pointInPolygon(surfaces[k].getP1().x, surfaces[k].getP1().y, rectangle)
                    || pointInPolygon(surfaces[k].getP2().x, surfaces[k].getP2().y, rectangle)) {
                surfInside = true;
                break;
            }
            if (exit.intersects(surfaces[k])
                    && surfaces[k].getP1() != exit.getP1() && surfaces[k].getP2() != exit.getP2()
                    && surfaces[k].getP1() != exit.getP2() && surfaces[k].getP2() != exit.getP1()) {
                isIntersecting = true;
                break;
            }
            bool pointInLeftAccess = surfaceIntersectsPolygon(surfaces[k], leftAccess)
                    || pointInPolygon(surfaces[k].getP1().x, surfaces[k].getP1().y, leftAccess)
                    || pointInPolygon(surfaces[k].getP2().x, surfaces[k].getP2().y, leftAccess);

            bool pointInRightAccess = surfaceIntersectsPolygon(surfaces[k], rightAccess)
                    || pointInPolygon(surfaces[k].getP1().x, surfaces[k].getP1().y, rightAccess)
                    || pointInPolygon(surfaces[k].getP2().x, surfaces[k].getP2().y, rightAccess);
            if (pointInLeftAccess && pointInRightAccess) {
                isAccessible = false;
                break;
            }

        }
        if (!surfInside && !isIntersecting && isAccessible) {
            return true;
        }
    }
    return false;
}

Surface FindGap(const vector<Surface>& surfaces, const vector<Surface>& robotSurfaces, const bool takeBorderPoints) {
    //Init
    Surface robotOrientation = robotSurfaces[0];
    list<pair<double, PointXY> > gapsPoints;
    //Find gaps between adjacent surfaces
    for (unsigned int i = 0; i < surfaces.size() - 1; i++) {
        if (surfaces[i].distFromP2ToPoint(surfaces[i + 1].getP1().x, surfaces[i + 1].getP1().y) > 800) {
            PointXY gapP1 = PointXY(surfaces[i].getP2().x, surfaces[i].getP2().y);
            PointXY gapP2 = PointXY(surfaces[i + 1].getP1().x, surfaces[i + 1].getP1().y);
            double distP1 = robotOrientation.distFromP1ToPoint(gapP1.getX(), gapP1.getY());
            double distP2 = robotOrientation.distFromP1ToPoint(gapP2.getX(), gapP2.getY());
            if (distP1 > MIN_DISTANCE_VISION && distP2 > MIN_DISTANCE_VISION) {
                gapsPoints.push_back(make_pair(distP1, gapP1));
            }
            if (distP2 > MIN_DISTANCE_VISION) {
                gapsPoints.push_back(make_pair(distP2, gapP2));
            }
        }
    }
    if (takeBorderPoints) {
        double distP1 = robotOrientation.distFromP1ToPoint(surfaces[0].getP1().x, surfaces[0].getP1().y);
        if (distP1 > MIN_DISTANCE_VISION) {
            gapsPoints.push_back(make_pair(distP1, PointXY(surfaces[0].getP1().x, surfaces[0].getP1().y)));
        }
        double distP2 = robotOrientation.distFromP1ToPoint(surfaces[surfaces.size() - 1].getP2().x, surfaces[surfaces.size() - 1].getP2().y);
        if (distP2 > MIN_DISTANCE_VISION) {
            gapsPoints.push_back(make_pair(distP2, PointXY(surfaces[surfaces.size() - 1].getP2().x, surfaces[surfaces.size() - 1].getP2().y)));
        }
    }
    if (gapsPoints.size() < 2) throw false;
    //Calculate the closest gap as a combination of the former gaps endpoints
    Surface closestGap;
    gapsPoints.sort(compare_double);
    pair<double, PointXY> min = gapsPoints.front();
    gapsPoints.pop_front();
    bool gapFound = false;
    while (gapsPoints.size() > 0) {
        double length = min.second.distFrom(gapsPoints.front().second);
        if (length > 800) {
            Surface gap = Surface(min.second.getX(), min.second.getY(), gapsPoints.front().second.getX(), gapsPoints.front().second.getY());
            bool isIntersecting = false;
            for (unsigned int i = 0; i < surfaces.size(); i++) {
                if (surfaces[i].getP1() == gap.getP1() || surfaces[i].getP2() == gap.getP2()
                        || surfaces[i].getP1() == gap.getP2() || surfaces[i].getP2() == gap.getP1()) {
                    //They have a common endpoint but don't really intersect
                    continue;
                }
                if (gap.intersects(surfaces[i])) {
                    isIntersecting = true;
                    break;
                }
            }
            if (!isIntersecting) {
                vector<Surface> tmpGapsPlot = surfaces;
                tmpGapsPlot.push_back(gap);
                plotSurfacesGNU("../outputs/Maps/gapsPlotTmp.png", tmpGapsPlot);
                char answer = 'y';
                cout << "We found the following gap. Do you want another one ? y/n" << endl;
                gap.display();
                cin >> answer;
                if (answer == 'n') {
                    closestGap = gap;
                    gapFound = true;
                    break;
                }
            }
        }
        min = gapsPoints.front();
        gapsPoints.pop_front();
    }
    if (!gapFound) {
        throw false;
    }
    closestGap.orderEndpoints(robotOrientation);
    return closestGap;
}

Surface Map::FindGapWithDistance(const Surface& robotOrientation, const View& curView, const View& prevView) {
    //Distance of current gap from current robot position
    double approximateDistance = max(curView.getRobotSurfaces()[0].distFromP1ToPoint(curView.getGap().getP1().x, curView.getGap().getP1().y),
            curView.getRobotSurfaces()[0].distFromP1ToPoint(curView.getGap().getP2().x, curView.getGap().getP2().y));
    //Angles of current gap endpoints from current robot orientation
    double angleP1 = curView.getRobotSurfaces()[0].getAngleFromP1ToPoint(curView.getGap().getP1().x, curView.getGap().getP1().y);
    double angleP2 = curView.getRobotSurfaces()[0].getAngleFromP1ToPoint(curView.getGap().getP2().x, curView.getGap().getP2().y);

    double angleMin = min(angleP1, angleP2);
    double angleMax = max(angleP1, angleP2);

    vector<Surface> surfaces = prevView.getSurfaces();
    vector<Surface> surfToKeep;
    double adjustedAngleMin = 0.7 * angleMin;
    double adjustedAngleMax = 1.3 * angleMax;
    bool zeroInBetween = abs(angleMin - angleMax) > 180;
    for (unsigned int i = 0; i < surfaces.size(); i++) {
        double distP1 = robotOrientation.distFromP1ToPoint(surfaces[i].getP1().x, surfaces[i].getP1().y);
        double distP2 = robotOrientation.distFromP1ToPoint(surfaces[i].getP2().x, surfaces[i].getP2().y);
        double angleP1 = robotOrientation.getAngleFromP1ToPoint(surfaces[i].getP1().x, surfaces[i].getP1().y);
        double angleP2 = robotOrientation.getAngleFromP1ToPoint(surfaces[i].getP2().x, surfaces[i].getP2().y);
        bool P1inAngle = (!zeroInBetween && angleP1 >= adjustedAngleMin && angleP1 <= adjustedAngleMax)
                || (zeroInBetween && (angleP1 >= adjustedAngleMin || angleP1 <= adjustedAngleMax));
        bool P2inAngle = (!zeroInBetween && angleP2 >= adjustedAngleMin && angleP2 <= adjustedAngleMax)
                || (zeroInBetween && (angleP2 >= adjustedAngleMin || angleP2 <= adjustedAngleMax));
        if ((abs(distP1 - approximateDistance) < 0.3 * approximateDistance && P1inAngle)
                || (abs(distP2 - approximateDistance) < 0.3 * approximateDistance && P2inAngle)) {
            surfToKeep.push_back(surfaces[i]);
        }
    }
    if (surfToKeep.size() < 2) {
        surfToKeep = surfaces;
        cout << "Can't find enough surfaces in the approximate distance" << endl;
        throw false;
    }

    /*    char name [50];
        vector<Surface> test = surfToKeep;
        test.insert(test.end(), robotSurfaces.begin(), robotSurfaces.end());
        sprintf(name, "%s%f%s", "../outputs/Maps/surfToKeep", robotOrientation.getP1().y, ".png");
        plotSurfacesGNU(name, test);*/

    ReferenceSurfaces refSurfacePair;
    refSurfacePair.setMapSurface(makeSurfaceWith(this->getMap().back().getRobotSurfaces()[0],
            this->getPathSegments().back().angle, this->getPathSegments().back().distance, 400.0));
    refSurfacePair.setViewSurface(curView.getRobotSurfaces()[0]);
    refSurfacePair.setRefPoint(1);

    return findExit(surfToKeep, robotOrientation.getP1(), true, &curView.getGap(), &refSurfacePair);
}

void Map::addViewUsingCorridorWidth(View& curView, Surface longSurfCV, Surface gapPV, cv::Point2f prevRbtPos) {
    Surface gapCV = curView.getGap();
    double corridorLengthP1 = longSurfCV.distFromPoint(gapCV.getP1().x, gapCV.getP1().y);
    double corridorLengthP2 = longSurfCV.distFromPoint(gapCV.getP2().x, gapCV.getP2().y);
    PointXY longSurfVect(longSurfCV.getP2().x - longSurfCV.getP1().x, longSurfCV.getP2().y - longSurfCV.getP1().y);
    vector<Surface> curSurfaces = curView.getSurfaces();
    Surface refPV;
    Surface refCV;

    if (corridorLengthP1 > corridorLengthP2) {
        //Exit between long surface and P1
        vector<Surface> tangents = findTangents(gapPV.getP1(), gapPV.getP2(), corridorLengthP1);

        if (tangents.size() == 0) throw false;

        for (unsigned int i = 0; i < tangents.size(); i++) {
            curSurfaces.push_back(tangents[i]);
            if (!pointsOnSameSideOfSurface(prevRbtPos, tangents[i].getP2(), gapPV)) {
                PointXY tangentVect(tangents[i].getP2().x - tangents[i].getP1().x, tangents[i].getP2().y - tangents[i].getP1().y);
                refPV = Surface(gapPV.getP1().x, gapPV.getP1().y, gapPV.getP1().x + tangentVect.getX(), gapPV.getP1().y + tangentVect.getY());
                break;
            }
        }

        refCV = Surface(gapCV.getP1().x, gapCV.getP1().y, gapCV.getP1().x + longSurfVect.getX(), gapCV.getP1().y + longSurfVect.getY());
        plotSurfacesGNU("../outputs/Maps/tangents.png", curSurfaces);
    } else {
        vector<Surface> tangents = findTangents(gapPV.getP2(), gapPV.getP1(), corridorLengthP2);

        if (tangents.size() == 0) throw false;

        for (unsigned int i = 0; i < tangents.size(); i++) {
            curSurfaces.push_back(tangents[i]);
            if (!pointsOnSameSideOfSurface(prevRbtPos, tangents[i].getP2(), gapPV)) {
                PointXY tangentVect(tangents[i].getP2().x - tangents[i].getP1().x, tangents[i].getP2().y - tangents[i].getP1().y);
                refPV = Surface(gapPV.getP2().x, gapPV.getP2().y, gapPV.getP2().x + tangentVect.getX(), gapPV.getP2().y + tangentVect.getY());
                break;
            }
        }
        refCV = Surface(gapCV.getP2().x, gapCV.getP2().y, gapCV.getP2().x + longSurfVect.getX(), gapCV.getP2().y + longSurfVect.getY());
        plotSurfacesGNU("../outputs/Maps/tangents.png", curSurfaces);
    }



    ReferenceSurfaces refPair;
    refPair.setMapSurface(refPV);
    refPair.setViewSurface(refCV);
    refPair.setRefPoint(1);

    //Calculate the view surfaces in map's coordinates
    Surface cvSurfaceOnMap;
    vector<Surface> allCVSurfacesOnMap;
    for (unsigned int i = 0; i < curView.getSurfaces().size(); i++) {
        cvSurfaceOnMap = trangulateSurface(refPair.getMapSurface(), refPair.getViewSurface(),
                curView.getSurfaces()[i], refPair.getRefPoint());
        allCVSurfacesOnMap.push_back(cvSurfaceOnMap);
    }
    //Calculate the robot surfaces in map's coordinates
    Surface cRobotSurfaceOnMap;
    vector<Surface> cRobotSurfacesOnMap;
    for (unsigned int i = 0; i < curView.getRobotSurfaces().size(); i++) {
        cRobotSurfaceOnMap = trangulateSurface(refPair.getMapSurface(), refPair.getViewSurface(),
                curView.getRobotSurfaces()[i], refPair.getRefPoint());
        cRobotSurfacesOnMap.push_back(cRobotSurfaceOnMap);
    }
    View cViewOnMap;
    cViewOnMap.setSurfaces(allCVSurfacesOnMap);
    cViewOnMap.setRobotSurfaces(cRobotSurfacesOnMap);
    cViewOnMap.setId(curView.getId());
    cViewOnMap.setHasGap(curView.getHasGap());

    this->addCvAndClean(cViewOnMap);
}

vector<AngleAndDistance> Map::FindWayHome() {
    vector<AngleAndDistance> wayHome;
    vector<Surface> allSurfaces;
    Surface rotEntrance = entrance;
    rotEntrance.rotateAroundP1(45); // rotEntrance middle point is 400mm from the entrance because the entrance width is 800mm
    PointXY beforeEntrance(rotEntrance.midPoint().x, rotEntrance.midPoint().y);

    //Look for a direct way from the exit to the entrance
    Surface directWay(exit.midPoint().x, exit.midPoint().y, beforeEntrance.getX(), beforeEntrance.getY());
    bool directWayPossible = true;
    for (unsigned int i = 0; i < map.size(); i++) {
        vector<Surface> surfaces = map[i].getSurfaces();
        for (unsigned int j = 0; j < surfaces.size(); j++) {
            allSurfaces.push_back(surfaces[j]);
            if (directWay.intersects(surfaces[j])) {
                directWayPossible = false;
                break;
            }
        }
        if (!directWayPossible) {
            break;
        }
    }
    if (directWayPossible) {
        AngleAndDistance path;
        path.distance = PointXY(directWay.getP1().x, directWay.getP1().y).distFrom(beforeEntrance);
        Surface tmp = exit;
        tmp.setP1(exit.midPoint().x, exit.midPoint().y);
        tmp.rotateAroundP1(-90);
        path.angle = tmp.getAngleFromP1ToPoint(beforeEntrance.getX(), beforeEntrance.getY());
        wayHome.push_back(path);
        return wayHome;
    }

    //No direct way : A* algorithm
    cout << "Finding a way to avoid obstacles" << endl;

    list<PointXY> path = findPathAStar(PointXY(exit.midPoint().x, exit.midPoint().y), beforeEntrance, allSurfaces);
    vector<PointXY> pathVect(path.begin(), path.end());
    Surface orientation = exit;
    orientation.setP1(orientation.midPoint().x, orientation.midPoint().y);
    orientation.rotateAroundP1(-90);

    for (unsigned int i = 1; i < pathVect.size()-1; i++) {
        AngleAndDistance ad;
        ad.distance = orientation.distFromP1ToPoint(pathVect[i].getX(), pathVect[i].getY());
        ad.angle = orientation.getAngleFromP1ToPoint(pathVect[i].getX(), pathVect[i].getY());
        orientation = makeSurfaceWith(orientation, ad.angle, ad.distance, 200);
        wayHome.push_back(ad);
    }

    return wayHome;
}

void Map::addSurfacesAfterEntrance(Map& lastMap) {

    vector<Surface> afterExit;
    cv::Point2f robotPosition = lastMap.getMap().back().getRobotSurfaces()[0].getP1();
    for (unsigned int i = 0; i < lastMap.getMap().size(); i++) {
        vector<Surface> surfs = lastMap.getMap()[i].getSurfaces();
        for (unsigned int j = 0; j < surfs.size(); j++) {
            if (!pointsOnSameSideOfSurface(robotPosition, surfs[j].getP1(), lastMap.getExit())
                    || !pointsOnSameSideOfSurface(robotPosition, surfs[j].getP2(), lastMap.getExit())) {
                afterExit.push_back(surfs[j]);
            }
        }
    }

    View v = this->map.front();
    vector<Surface> surfaceInViewCoordinates;
    for (unsigned int i = 0; i < afterExit.size(); i++) {
        surfaceInViewCoordinates.push_back(trangulateSurface(entrance, lastMap.getExit(), afterExit[i], 1));
    }

    v.addSurfaces(surfaceInViewCoordinates);
    this->map[0] = v;
}

View computeViewPositionWithExitBorders(View& view, pair<Surface, Surface> pcaExitBordersPV, double angleLastLocomotion, Surface rbtOrientationPV) {

    //Find current view exit borders directions
    pair<vector<Surface>, vector<Surface> > exitBordersCV;
    try {
        exitBordersCV = view.computeExitBordersDirections();
    } catch (bool e) {
        throw false;
    }
    cv::Point3f centre = cv::Point3f(view.getRobotSurfaces()[0].getP1().x, view.getRobotSurfaces()[0].getP1().y, view.getRobotSurfaces()[0].getAngleWithSurface(rbtOrientationPV) - angleLastLocomotion);
    vector<Surface> pcaSurfacesBorder1 = exitBordersCV.first;
    vector<Surface> pcaSurfacesBorder2 = exitBordersCV.second;

    //Plot CV exit
    char CVexit1[50];
    sprintf(CVexit1, "%s%d%s", "../outputs/Maps/CVexit-", view.getId(), "-before.png");
    vector<Surface> tmp1 = pcaSurfacesBorder1;
    tmp1.insert(tmp1.end(), pcaSurfacesBorder2.begin(), pcaSurfacesBorder2.end());
    plotSurfacesGNU(CVexit1, tmp1);
    vector<Surface> test;

    //Find the PCA surface which orientation matches best the previous view border PCA surface orientation (for each side)
    pcaSurfacesBorder1[0].rotate(centre);
    double minDiffAngle = abs(pcaExitBordersPV.first.getAngleWithSurface(pcaSurfacesBorder1[0]));
    int indexMinAngle = 0;
    for (unsigned int i = 1; i < pcaSurfacesBorder1.size(); i++) {
        pcaSurfacesBorder1[i].rotate(centre);
        test.push_back(pcaSurfacesBorder1[i]);
        double angleTmp = abs(pcaExitBordersPV.first.getAngleWithSurface(pcaSurfacesBorder1[i]));
        if (angleTmp < minDiffAngle) {
            minDiffAngle = angleTmp;
            indexMinAngle = i;
        }
    }
    vector<Surface> test2;
    pcaSurfacesBorder2[0].rotate(centre);
    double minDiffAngle2 = abs(pcaExitBordersPV.second.getAngleWithSurface(pcaSurfacesBorder2[0]));
    int indexMinAngle2 = 0;
    for (unsigned int i = 1; i < pcaSurfacesBorder2.size(); i++) {
        pcaSurfacesBorder2[i].rotate(centre);
        test2.push_back(pcaSurfacesBorder2[i]);
        double angleTmp = abs(pcaExitBordersPV.second.getAngleWithSurface(pcaSurfacesBorder2[i]));
        if (angleTmp < minDiffAngle2) {
            minDiffAngle2 = angleTmp;
            indexMinAngle2 = i;
        }
    }
    //Plot current view gap borders
    char CVexit[50];
    sprintf(CVexit, "%s%d%s", "../outputs/Maps/CVexit-", view.getId(), ".png");
    vector<Surface> tmp = pcaSurfacesBorder1;
    tmp.insert(tmp.end(), pcaSurfacesBorder2.begin(), pcaSurfacesBorder2.end());
    plotSurfacesGNU(CVexit, tmp);

    //Take as reference the side where the orientation matching is better
    Surface viewRef = pcaSurfacesBorder1[indexMinAngle];
    Surface mapRef = pcaExitBordersPV.first;
    int refPoint = 2;
    if (minDiffAngle2 < minDiffAngle) {
        viewRef = pcaSurfacesBorder2[indexMinAngle2];
        mapRef = pcaExitBordersPV.second;
        refPoint = 1;
    }
    centre.z = -centre.z;
    viewRef.rotate(centre);

    ReferenceSurfaces refSurfacePair;
    refSurfacePair.setViewSurface(viewRef);
    refSurfacePair.setMapSurface(mapRef);
    refSurfacePair.setRefPoint(refPoint);

    //Calculate the view surfaces in map's coordinates
    Surface cvSurfaceOnMap;
    vector<Surface> allCVSurfacesOnMap;
    for (unsigned int i = 0; i < view.getSurfaces().size(); i++) {
        cvSurfaceOnMap = trangulateSurface(refSurfacePair.getMapSurface(), refSurfacePair.getViewSurface(),
                view.getSurfaces()[i], refSurfacePair.getRefPoint());
        allCVSurfacesOnMap.push_back(cvSurfaceOnMap);
    }
    //Calculate the robot surfaces in map's coordinates
    Surface cRobotSurfaceOnMap;
    vector<Surface> cRobotSurfacesOnMap;
    for (unsigned int i = 0; i < view.getRobotSurfaces().size(); i++) {
        cRobotSurfaceOnMap = trangulateSurface(refSurfacePair.getMapSurface(), refSurfacePair.getViewSurface(),
                view.getRobotSurfaces()[i], refSurfacePair.getRefPoint());
        cRobotSurfacesOnMap.push_back(cRobotSurfaceOnMap);
    }
    View cViewOnMap;
    cViewOnMap.setSurfaces(allCVSurfacesOnMap);
    cViewOnMap.setRobotSurfaces(cRobotSurfacesOnMap);
    cViewOnMap.setId(view.getId());
    cViewOnMap.setHasGap(view.getHasGap());
    return cViewOnMap;
}

Surface findLongSurfaceInPV(View& prevView, View& curView, cv::Point2f rbtPosNextStep, Surface& longSurfCV) {

    //Find long surface orientation from the gap in current view (CV))
    Surface gapCV = curView.getGap();
    PointXY exitVect(gapCV.getP2().x - gapCV.getP1().x, gapCV.getP2().y - gapCV.getP1().y);
    exitVect = exitVect / sqrt(exitVect.getX() * exitVect.getX() + exitVect.getY() * exitVect.getY());
    PointXY orthoVect(-exitVect.getY(), exitVect.getX());
    orthoVect = orthoVect / sqrt(orthoVect.getX() * orthoVect.getX() + orthoVect.getY() * orthoVect.getY());
    Surface orthoSurf(gapCV.midPoint().x, gapCV.midPoint().y, gapCV.midPoint().x + orthoVect.getX(), gapCV.midPoint().y + orthoVect.getY());

    // Make sure the orthogonal surface is oriented outwards
    if (pointsOnSameSideOfSurface(curView.getRobotSurfaces()[0].getP1(), orthoSurf.getP2(), gapCV)) {
        orthoSurf.setP2(gapCV.midPoint().x - orthoVect.getX(), gapCV.midPoint().y - orthoVect.getY());
    }
    int orientationLongSurfCV = orientation(longSurfCV.midPoint(), orthoSurf.getP1(), orthoSurf.getP2());


    //Compute distance range in which we expect to find the long surface (PV)
    double distToLongSurfP1inCV = longSurfCV.distFromP1ToPoint(curView.getRobotSurfaces()[0].getP1().x, curView.getRobotSurfaces()[0].getP1().y);
    double distToLongSurfP2inCV = longSurfCV.distFromP2ToPoint(curView.getRobotSurfaces()[0].getP1().x, curView.getRobotSurfaces()[0].getP1().y);
    double minDistance = min(distToLongSurfP1inCV, distToLongSurfP2inCV)*0.8;
    double maxDistance = max(distToLongSurfP1inCV, distToLongSurfP2inCV)*1.2;


    //Find the longest surface within the right distance range and with same orientation
    vector<Surface> surfaces = prevView.getSurfaces();
    while (surfaces.size() > 0) {

        vector<PointXY> surfacePts;
        vector<int> surfacesIndexes; // To keep track of the surfaces selected
        surfacePts.push_back(PointXY(surfaces[0].getP1().x, surfaces[0].getP1().y));
        surfacePts.push_back(PointXY(surfaces[0].getP2().x, surfaces[0].getP2().y));
        double maxLength = surfaces[0].length();

        for (unsigned int i = 1; i < surfaces.size(); i++) {
            double tmpMinDist = min(surfaces[i].distFromP1ToPoint(rbtPosNextStep.x, rbtPosNextStep.y), surfaces[i].distFromP2ToPoint(rbtPosNextStep.x, rbtPosNextStep.y));
            double tmpMaxDist = max(surfaces[i].distFromP1ToPoint(rbtPosNextStep.x, rbtPosNextStep.y), surfaces[i].distFromP2ToPoint(rbtPosNextStep.x, rbtPosNextStep.y));

            if (tmpMinDist > minDistance && tmpMaxDist < maxDistance) {
                Surface lastSurface = Surface(surfacePts[surfacePts.size() - 2].getX(), surfacePts[surfacePts.size() - 2].getY(), surfacePts[surfacePts.size() - 1].getX(), surfacePts[surfacePts.size() - 1].getY());
                double angle = surfaces[i].getAngleWithSurface(lastSurface);
                if (lastSurface.distFromP2ToPoint(surfaces[i].getP1().x, surfaces[i].getP1().y) < 400
                        && ((int) abs(angle) % 180 < 45 || (int) abs(angle) % 180 > 135)) {
                    //Add every small surface that might be part of a longer one
                    surfacePts.push_back(PointXY(surfaces[i].getP1().x, surfaces[i].getP1().y));
                    surfacePts.push_back(PointXY(surfaces[i].getP2().x, surfaces[i].getP2().y));
                    surfacesIndexes.push_back(i);
                    /* if (surfaces[i].length() > maxLength) {
                         maxLength = surfaces[i].length();
                     }*/
                    maxLength += surfaces[i].length();
                } else if (surfaces[i].length() > maxLength) {
                    //Longer surface found
                    maxLength = surfaces[i].length();
                    surfacePts.clear();
                    surfacesIndexes.clear();
                    surfacePts.push_back(PointXY(surfaces[i].getP1().x, surfaces[i].getP1().y));
                    surfacePts.push_back(PointXY(surfaces[i].getP2().x, surfaces[i].getP2().y));
                    surfacesIndexes.push_back(i);
                }
            }

        }

        if (surfacesIndexes.size() == 0) {
            //No long surface
            throw false;
        }

        Surface pcaSurf = principalComponentAnalysis(surfacePts);
        //Compare long surface orientation in PV with the one in CV
        Surface gap = prevView.getGap();
        PointXY exitVect(gap.getP2().x - gap.getP1().x, gap.getP2().y - gap.getP1().y);
        exitVect = exitVect / sqrt(exitVect.getX() * exitVect.getX() + exitVect.getY() * exitVect.getY());
        PointXY orthoVect(-exitVect.getY(), exitVect.getX());
        orthoVect = orthoVect / sqrt(orthoVect.getX() * orthoVect.getX() + orthoVect.getY() * orthoVect.getY());
        Surface orthoSurf(gap.midPoint().x, gap.midPoint().y, gap.midPoint().x + orthoVect.getX(), gap.midPoint().y + orthoVect.getY());

        // Make sure the orthogonal surface is oriented outwards
        if (pointsOnSameSideOfSurface(prevView.getRobotSurfaces()[0].getP1(), orthoSurf.getP2(), gap)) {
            orthoSurf.setP2(gap.midPoint().x - orthoVect.getX(), gap.midPoint().y - orthoVect.getY());
        }

        if (orientationLongSurfCV == orientation(pcaSurf.midPoint(), orthoSurf.getP1(), orthoSurf.getP2())) {
            return pcaSurf;
        }

        // Different orientation : we erase the surfaces found and try again
        for (unsigned int i = 0; i < surfacesIndexes.size(); i++) {
            surfaces.erase(surfaces.begin() + surfacesIndexes[i]);
        }
    }

    //No long surface found
    throw false;
    return Surface();
}