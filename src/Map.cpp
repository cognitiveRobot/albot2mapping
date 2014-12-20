#include "Map.h"
#include "PointAndSurface.h"
#include "GeometryFuncs.h"
#include "ImageProcessing.h"
#include "Printer.h"

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
    vector<Surface> Map::getTempSurfaces() const{
        return tempSurfaces;
    }
    
   Map Map::getItself() const{
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

//adding pv on to cv to compute egoCentric map.
void Map::addPVUsingOdo(const View & curView, const AngleAndDistance & homeInfo) {
    //check the point in polygon    cleaning old.
    vector<Surface> tempSurf;
    View tempView;
    
    double newX, newY, angle;
    
    angle = this->pathSegments.back().angle * CONVERT_TO_RADIAN; // degree to randian.
        //find cv center in the pv coordinate frame.
        //need to convert robot position from mm to cm.
        newX = (this->pathSegments.back().distance / 10.0) * sin(-angle); //x= d*cos(th) = d*cos(90-angle) = d*sin(angle) //as aris give - value for right turn
        newY = (this->pathSegments.back().distance / 10.0) * cos(-angle); //y=d*sin(th)=d*sin(90-angle)=d*cos(angle)

    for (unsigned int i = 0; i<this->map.size(); i++) {
        tempView = this->map[i];
        
        //transform surfaces
        tempSurf = tempView.getSurfaces();
        for (unsigned int j = 0; j < tempSurf.size(); j++) {
            tempSurf[j] = tempSurf[j].transFrom(newX,newY,angle);
        }
        tempView.setSurfaces(tempSurf);
        
        //transform robotSurfaces
        tempSurf = tempView.getRobotSurfaces();
        for (unsigned int j = 0; j < tempSurf.size(); j++) {
            tempSurf[j] = tempSurf[j].transFrom(newX,newY,angle);
        }
        tempView.setRobotSurfaces(tempSurf);
        
        //reset View.
        this->map[i] = tempView;        
    }
        
        //add current View.
        this->map.push_back(curView);
}


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
        newX = (allPathSegments[i].distance / 10.0) * sin(-angle); //x= d*cos(th) = d*cos(90-angle) = d*sin(angle) //as aris give - value for right turn
        newY = (allPathSegments[i].distance / 10.0) * cos(-angle); //y=d*sin(th)=d*sin(90-angle)=d*cos(angle)

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


    cout<<BOLDRED<<"Transfomed surfs: "<<cvSurfaces.size()<<RESET<<endl;

    View tranView;
    tranView.setSurfaces(cvSurfaces);
    tranView.setRobotSurfaces(rpSurfaces);

    map.push_back(tranView);
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
    char mapName[50];
    sprintf(mapName, "%s%d%s%d%s", "../outputs/Maps/LS-", this->getMapID(), "-v-", curView.getId(), "a-before-withB.png");
    this->setTempSurfaces(boundaryLines);
    plotMapGNU(mapName, this->getItself());
    this->getTempSurfaces().clear();

    //making polygon from cv.
    vector<SurfaceT> polygon;
    for (unsigned int i = 0; i < transformedSurfaces.size(); i++) {
        polygon.push_back(SurfaceT(PointXY((double) transformedSurfaces[i].getP1().x, (double) transformedSurfaces[i].getP1().y),
                PointXY((double) transformedSurfaces[i].getP2().x, (double) transformedSurfaces[i].getP2().y)));
    }

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

void Map::update(View newView) {
    currentView = newView; //it save cv for next step.

    // Robot
    cv::Point3f tmpPos;
    cv::Point3f O(sizeX / 2, sizeY / 2, 0);
    //cv::Point3f O(sizeX/2, sizeY-15, 0);

    // Comparing surface variables
    vector<Surface> tmpObst, tmpObst1; // Temporary surfaces vector to update Obst
    int counter;
    unsigned int Osize, Osize1; // Original size of Obst before adding new Surfaces to the map

    cout << surfaces.size() << " Initial Surfaces in the map" << endl;
    /* Adapt newView to compare Surfaces to previous ones */
    newView.setSurfaces();
    newView.rotate();
    newView.translate();

    if (surfaces.size() == 0) { // If the map is empty, place the robot in initial position and add surfaces
        // Set robot position on the map and add it to rbtPos
        cout << surfaces.size() << "No Surface in the map" << endl;
        tmpPos = newView.getRobotPos();
        coordTransf(&tmpPos, O, (double) 1 / 10, (double) - 1 / 10);
        newView.setRobotPos(tmpPos.x, tmpPos.y, tmpPos.z);
        rbtPos.push_back(tmpPos);

        // Add the Surfaces to the map
        surfaces = newView.getSurfaces();

        newView.clearView();
        cout << surfaces.size() << " start Surface in the map" << endl;
    } else // Else compare surfaces
    {

        // Move the robot
        tmpPos = newView.getRobotPos();

        coordTransf(&tmpPos, O, (double) 1 / 10, (double) - 1 / 10);
        newView.setRobotPos(tmpPos.x, tmpPos.y, tmpPos.z);
        rbtPos.push_back(tmpPos);

        Osize = surfaces.size();
        Osize1 = surfaces.size();
        //old and then new
        for (unsigned int j = 0; j < newView.getSurfaces().size(); j++) { // For each surface of the new View

            counter = 0;

            for (unsigned int i = 0; i < Osize; i++) { // For every surface of the map

                if (isBehind(surfaces[i], newView.getSurfaces()[j],
                        rbtPos[rbtPos.size()])
                        || isBehind(newView.getSurfaces()[j], surfaces[i],
                        rbtPos[rbtPos.size()])) // Check if any Surfaces are concealing others
                {
                    counter++;

                }
                if (j == 0) {

                    tmpObst.push_back(surfaces[i]); // Add it to the updated map

                }
            }

            if (counter == 0) { // If an old Surface isn't concealing nor concealed....
                tmpObst.push_back(newView.getSurfaces()[j]); // add all surfaces from new view     // Add the new Surfaces

            }
        }

        // new and then old
        //for(unsigned int i=0; i < Osize; i++)           // For every surface of the map

        //{
        //     counter=0;

        // for(unsigned int j=0; j<newView.getSurfaces().size(); j++)         // For each surface of the new View

        //      {

        //          if(isBehind(surfaces[i], newView.getSurfaces()[j], rbtPos[rbtPos.size()]) || isBehind(newView.getSurfaces()[j], surfaces[i], rbtPos[rbtPos.size()]))     // Check if any Surfaces are concealing others
        // {
        //          counter++;

        //          }
        //if(i == 0 ){ 
        //surfaces.push_back(newView.getSurfaces()[j]);

        //             }
        //             }
        //             cout << counter << "Counter" << endl;
        //               if(counter == 0 ){
        //tmpObst.push_back(surfaces[i]);  // Add it to the updated map
        // If an old Surface isn't concealing nor concealed....
        //add all surfaces from new view     // Add the new Surfaces

        //           }
        //          }

        // both new and old
        //for(unsigned int i=0; i < Osize1; i++)           // For every surface of the map

        //{tmpObst.push_back(surfaces[i]);
        //}
        //           for(unsigned int j=0; j<newView.getSurfaces().size(); j++)         // For each surface of the new View

        //          {

        //tmpObst.push_back(newView.getSurfaces()[j]);

        //                  }  

        surfaces.clear();
        surfaces = tmpObst;
        // replace the map by the updated one
        cout << tmpObst.size() << " Surfaces in the new map" << endl;
        tmpObst.clear();

    }

}

bool Map::isBehind(Surface Old, Surface New, cv::Point3f rbtPos) {
    bool ret = false;

    // Translate to compare as same view
    Old.setP1(Old.getP1().x - rbtPos.x, Old.getP1().y - rbtPos.y);
    Old.setP2(Old.getP2().x - rbtPos.x, Old.getP2().y - rbtPos.y);
    New.setP1(Old.getP1().x - rbtPos.x, New.getP1().y - rbtPos.y);
    New.setP2(Old.getP2().x - rbtPos.x, New.getP2().y - rbtPos.y);

    if (Old.getP1().y > 0 && Old.getP2().y > 0) {
        if (Old.getP1().x == 0) {
            if (New.getP1().x > 0
                    && New.getP1().y
                    > (float) Old.getP2().y / Old.getP2().x
                    * New.getP1().x && New.getP2().x > 0
                    && New.getP2().y
                    > (float) Old.getP2().y / Old.getP2().x
                    * New.getP2().x) {
                ret = true;
            }
        } else if (Old.getP1().x < 0 && Old.getP2().x == 0) {
            if ((New.getP1().y
                    > (float) Old.getP1().y / Old.getP1().x * New.getP1().x
                    && New.getP1().x < 0)
                    || (New.getP2().y
                    > (float) Old.getP1().y / Old.getP1().x
                    * New.getP2().x && New.getP2().x < 0)) {
                ret = true;
            }
        } else if (Old.getP1().x < 0 && Old.getP2().x < 0) {
            if ((New.getP1().y
                    > (float) Old.getP1().y / Old.getP1().x * New.getP1().x
                    && New.getP1().y
                    < (float) Old.getP2().y / Old.getP2().x
                    * New.getP1().x)
                    || (New.getP2().y
                    > (float) Old.getP1().y / Old.getP1().x
                    * New.getP2().x
                    && New.getP2().y
                    < (float) Old.getP2().y / Old.getP2().x
                    * New.getP2().x)) {
                ret = true;
            }
        } else if (Old.getP1().x < 0 && Old.getP2().x > 0) {
            if ((New.getP1().y
                    > (float) Old.getP1().y / Old.getP1().x * New.getP1().x
                    && New.getP1().y
                    > (float) Old.getP2().y / Old.getP2().x
                    * New.getP1().x)
                    || (New.getP2().y
                    > (float) Old.getP1().y / Old.getP1().x
                    * New.getP2().x
                    && New.getP2().y
                    > (float) Old.getP2().y / Old.getP2().x
                    * New.getP2().x)) {
                ret = true;
            }
        } else if (Old.getP1().x > 0 && Old.getP2().x > 0) {
            if ((New.getP1().y
                    < (float) Old.getP1().y / Old.getP1().x * New.getP1().x
                    && New.getP1().y
                    > (float) Old.getP2().y / Old.getP2().x
                    * New.getP1().x)
                    || (New.getP2().y
                    < (float) Old.getP1().y / Old.getP1().x
                    * New.getP2().x
                    && New.getP2().y
                    > (float) Old.getP2().y / Old.getP2().x
                    * New.getP2().x)) {
                ret = true;
            }
        }
    }

    // Set them back to original position
    Old.setP1(Old.getP1().x + rbtPos.x, Old.getP1().y + rbtPos.y);
    Old.setP2(Old.getP2().x + rbtPos.x, Old.getP2().y + rbtPos.y);
    New.setP1(Old.getP1().x + rbtPos.x, New.getP1().y + rbtPos.y);
    New.setP2(Old.getP2().x + rbtPos.x, New.getP2().y + rbtPos.y);

    return ret;
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

void Map::expandMap(const View & curView) {
    
    Surface refInMap = this->getPreviousView().getRPositionInPV();
    cout<<"rp at2 "<<endl;
    refInMap.display();
    Surface refInCV(0, 0, 0, 40);
    vector<Surface> cvSurfaces = curView.getSurfaces();
    cout<<"cvSurf: "<<cvSurfaces.size()<<endl;
    vector<Surface> cvSurfacesOnMap = trangulateSurfaces(refInMap, refInCV, cvSurfaces);
    
    vector<Surface> rpInMap = trangulateSurfaces(refInMap, refInCV, curView.getRobotSurfaces());
    cout<<"rpSurf: "<<rpInMap.size()<<endl;
    displaySurfaces(rpInMap);
    
    cout<<"cvSurfOnMap: "<<cvSurfacesOnMap.size()<<endl;
    View newView;
    newView.setSurfaces(cvSurfacesOnMap);
    newView.setRobotSurfaces(rpInMap);
    
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


vector<Surface> trangulateSurfaces(const Surface & refInMap, const Surface & refInCV, const vector<Surface>& cvSurfaces) {
    vector<Surface> cvSurfacesOnMap;
    double angle, distance;
    float x1, y1, x2, y2;
    int limit = cvSurfaces.size();
//    if(cvSurfaces.size() > 10)
//        limit = 10;
    for (unsigned int i = 0; i < limit; i++) {
        angle = refInCV.getAngleWithSurface(Surface(0, 0, cvSurfaces[i].getP1().x, cvSurfaces[i].getP1().y));
        distance = refInCV.distFromP1ToPoint(cvSurfaces[i].getP1().x, cvSurfaces[i].getP1().y);

        angle *= CONVERT_TO_RADIAN;

        //cout << "angle: " << angle << " dist: " << distance << endl;
        //waitHere();
        x1 = ((refInMap.getP2().x - refInMap.getP1().x) / refInMap.length()) * cos(angle)-((refInMap.getP2().y - refInMap.getP1().y) / refInMap.length()) * sin(angle);
        y1 = ((refInMap.getP2().x - refInMap.getP1().x) / refInMap.length()) * sin(angle)+((refInMap.getP2().y - refInMap.getP1().y) / refInMap.length()) * cos(angle);

        x1 = x1 * distance + refInMap.getP1().x;
        y1 = y1 * distance + refInMap.getP1().y;

        angle = refInCV.getAngleWithSurface(Surface(0, 0, cvSurfaces[i].getP2().x, cvSurfaces[i].getP2().y));
        distance = refInCV.distFromP1ToPoint(cvSurfaces[i].getP2().x, cvSurfaces[i].getP2().y);

        angle *= CONVERT_TO_RADIAN;
        
        x2 = ((refInMap.getP2().x - refInMap.getP1().x) / refInMap.length()) * cos(angle)-((refInMap.getP2().y - refInMap.getP1().y) / refInMap.length()) * sin(angle);
        y2 = ((refInMap.getP2().x - refInMap.getP1().x) / refInMap.length()) * sin(angle)+((refInMap.getP2().y - refInMap.getP1().y) / refInMap.length()) * cos(angle);

        x2 = x2 * distance + refInMap.getP1().x;
        y2 = y2 * distance + refInMap.getP1().y;
        
        cvSurfacesOnMap.push_back(Surface(x1,y1,x2,y2));
    }
    
    return cvSurfacesOnMap;
}