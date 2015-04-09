#include "Map.h"
#include "PointAndSurface.h"
#include "GeometryFuncs.h"
#include "ImageProcessing.h"
#include "Printer.h"
#include "SameSurfaceFinderOdo.h"

Map::Map(int _sizeX, int _sizeY) :
sizeX(_sizeX), sizeY(_sizeY) {
    sizeX = 2000;
    sizeY = 2000;
    M = 0;
    ID = 0;
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
}

void Map::setPreviousView(const View & pView) {
    previousView = pView;
}

View Map::getPreviousView() {
    return previousView;
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

Map Map::getItself() const {
    return *this;
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
    //check the point in polygon    cleaning old.
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

void Map::cleanMap(const vector<SurfaceT> & polygon) {
    //check the point in polygon    cleaning old.
    vector<Surface> surfacesInsideCV, surfacesOutsideCV;
    vector<Surface> tempSurf;
    View tempView;

    for (unsigned int i = 0; i<this->map.size() - 1; i++) {
        tempView = this->map[i];
        tempSurf = tempView.getSurfaces();
        for (unsigned int j = 0; j < tempSurf.size(); j++) {
            if (pointInPolygon(PointXY((double) tempSurf[j].getP1().x, (double) tempSurf[j].getP1().y), polygon) == true ||
                    pointInPolygon(PointXY((double) tempSurf[j].getP2().x, (double) tempSurf[j].getP2().y), polygon) == true)
                surfacesInsideCV.push_back(tempSurf[j]);
            else
                surfacesOutsideCV.push_back(tempSurf[j]);
        }
        tempView.setSurfaces(surfacesOutsideCV);
        //tempView.setLandmarks(surfacesInsideCV);
        this->map[i] = tempView;

        //clear variables to reuse.
        surfacesInsideCV.clear();
        surfacesOutsideCV.clear();
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
    //    char mapName[50];
    //    sprintf(mapName, "%s%d%s%d%s", "../outputs/Maps/LS-", this->getMapID(), "-v-", curView.getId(), "a-before-withB.png");
    //    this->setTempSurfaces(boundaryLines);
    //    plotMapGNU(mapName, this->getItself());
    //    this->getTempSurfaces().clear();

    //making polygon from cv.
    vector<SurfaceT> polygon = constructPolygon(transformedSurfaces);

    //check the point in polygon    cleaning old.
    cleanMap(polygon);


    
}

//all-centric mapping.
//add cv to the map using multiple reference. 
void Map::addCVUsingMultipleRef(const View & curView) {
    cout<<endl<<"Adding cv using multiple reference surfaces!!!"<<endl;
    //find reference surfaces
    vector<ReferenceSurfaces> sameSurfaces;
    SameSurfaceFinderOdo sSurfaceInfo;
    sSurfaceInfo.recognizeSameSurface(sameSurfaces, this->getLandmarkSurfaces(), curView.getSurfaces(), this->getPathSegments().back());
    if (sameSurfaces.size() > 0) {
        //this->setRefForNextLS(sameSurfaces[0]);
        //tempSurf = sameSurfaces[1];
        cout<<"The following reference surfaces are found .."<<endl;
        for(unsigned int i=0;i<sameSurfaces.size(); i++) {
            sameSurfaces[i].display();
        }
        //waitHere();
    }
    ReferenceSurfaces refSurfacePair;
    Surface cvSurfaceOnMap;
    vector<Surface> allCVSurfacesOnMap;
    for(unsigned int i=0; i<curView.getSurfaces().size(); i++) {
        //find the closest ref pair using current view 
        refSurfacePair = findTheClosestReference(curView.getSurfaces()[i],sameSurfaces);
        //trangulate this surface.
        cout<<curView.getSurfaces()[i].getId()<<" refID: "<<refSurfacePair.getViewSurface().getId()<<endl;
        cvSurfaceOnMap = trangulateSurface(refSurfacePair.getMapSurface(),refSurfacePair.getViewSurface(),
                curView.getSurfaces()[i],refSurfacePair.getRefPoint());
        allCVSurfacesOnMap.push_back(cvSurfaceOnMap);
    }

    View cViewOnMap;
    cViewOnMap.setSurfaces(allCVSurfacesOnMap);
    this->map.push_back(cViewOnMap);
    plotViewsGNU("../outputs/Maps/1afteraddingCV.png",this->getMap());
    //construct polygon from cvOnMap.
    vector<SurfaceT> polygon = constructPolygon(allCVSurfacesOnMap);
    //cleanMap.
    cleanMap(polygon);
    
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

    //new codes for consistency  
    //unsigned int numline1 = 0;
    //for (unsigned int i = 0; i<surfaces.size(); i++)
    //  {
    // curObst1 = surfaces[i];                  // Transform the coordinates to have the right frame of reference

    //    if(curObst1.getPoints().size() > 4 ) // If there is enough points in 1 surface...
    //   {
    //            curObst1.setSurface();                                                     // Construct a line with the points
    // Adapt the point to the openCV Mat drawing
    //            curObst1.setP1(rbtPos[0].x + curObst1.getP1().x,rbtPos[0].y - curObst1.getP1().y );
    //            curObst1.setP2(rbtPos[0].x + curObst1.getP2().x,rbtPos[0].y - curObst1.getP2().y );
    //
    //            cv::line(drawing, curObst1.getP1(), curObst1.getP2(), cv::Scalar(0,0,255), 3, 8, 0);    // Draw that line
    //   numline1++;
    // }
    //}
    //cout << numline1<< " Number of lines in this map" << endl;
    // new code completes

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
        for(unsigned int i=0;i<sameSurfaces.size(); i++) {
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

//    vector<Surface> rpSurfaces = curView.getRobotSurfaces();
//    vector<Surface> rpInMap = trangulateSurfaces(refInMap, refInCV, rpSurfaces);
//    cout << "rpSurfOnMap: " << rpInMap.size() << endl;
//    displaySurfaces(rpInMap);

    
    View newView;
    newView.setSurfaces(cvSurfacesOnMap);
 //   newView.setRobotSurfaces(rpInMap);

    this->map.push_back(newView);

}



// some function
//it takes current view surfaces

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
vector<SurfaceT> constructPolygon(const vector<Surface> & transformedSurfaces) {
    vector<SurfaceT> polygon;
    for (unsigned int i = 0; i < transformedSurfaces.size(); i++) {
        polygon.push_back(SurfaceT(PointXY((double) transformedSurfaces[i].getP1().x, (double) transformedSurfaces[i].getP1().y),
                PointXY((double) transformedSurfaces[i].getP2().x, (double) transformedSurfaces[i].getP2().y)));
    }
    
    return polygon;
}

//it finds the closest ref pair where cvSurfaces and viewSurface in RefSurfaces are in same co-ordinate.
ReferenceSurfaces findTheClosestReference(Surface & cvSurface, vector<ReferenceSurfaces> allRefSurfaces) {
    double closestDist, tempDist;
    SurfaceT cvSurfaceT = cvSurface.ToSurfaceT();
    closestDist = distBetweenSurfs(cvSurfaceT,allRefSurfaces[0].getViewSurface().ToSurfaceT());
    ReferenceSurfaces refPair = allRefSurfaces[0];
    
    for(unsigned int i = 1; i<allRefSurfaces.size(); i++) {
        tempDist = distBetweenSurfs(cvSurfaceT,allRefSurfaces[i].getViewSurface().ToSurfaceT());
        if( tempDist < closestDist) {
            refPair = allRefSurfaces[i];
            closestDist = tempDist;
        }
    }
    
    return refPair;
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

        cvSurfacesOnMap.push_back(trangulateSurface(refInMap,refInCV,cvSurfaces[i],1));
    }

    return cvSurfacesOnMap;
}