#include "SameSurfaceFinderOdo.h"
#include "Printer.h"
#include "ImageProcessing.h"

bool SameSurfaceFinderOdo::matchSameSurfaces(vector<SameSurfaceInfo> & matchingInfoForAll, const vector<Surface> pvLandmarksOnCV, const vector<Surface> & cvLandmarks, 
            const double & angleTh, const double & distTh) {
    SameSurfaceInfo matchingInfo;
    bool success = false;
    double angleDiff, distanceDiffP1, distanceDiffP2;
    for (unsigned int i = 0; i < pvLandmarksOnCV.size(); i++) {
        for (unsigned int j = 0; j < cvLandmarks.size(); j++) {
            angleDiff = pvLandmarksOnCV[i].getAngleWithSurface(cvLandmarks[j]);

            if (abs(angleDiff) < angleTh && cvLandmarks[j].length() > 500.0) {
                // cout << endl << "angle Diff " << angleDiff << endl;
                //   cout << " pv: " << pvLandmarksOnCV[i].getId() << " cv: " << cvLandmarks[j].getId() << endl;
                distanceDiffP1 = cvLandmarks[j].distFromP1ToPoint(pvLandmarksOnCV[i].getP1().x, pvLandmarksOnCV[i].getP1().y);
                //    cout << "dist diff (p1,p1) : " << distanceDiffP1 << endl;
                distanceDiffP2 = cvLandmarks[j].distFromP2ToPoint(pvLandmarksOnCV[i].getP2().x, pvLandmarksOnCV[i].getP2().y);
                //   cout << "dist diff (p2,p2) : " << distanceDiffP2 << endl;
                if (distanceDiffP1 < distTh or distanceDiffP2 < distTh) {
                    //                    distP1 = cvLandmarks[j].distFromP1ToPoint(0, 0);
                    //                    distP2 = cvLandmarks[j].distFromP2ToPoint(0, 0);
                    //                    cout<<"dist p1 and p2 to cv robotP "<<distP1<<" "<<distP2<<endl;
                    // if ( distanceDiffP1 < lastDist ||  distanceDiffP2 < lastDist) {
                    // if (abs(angleDiff) < lastAngleDiff) {
                    //   lastAngleDiff = abs(angleDiff);
                    cout << "Ref found" << endl;
                    cout << endl << "angle Diff " << angleDiff << endl;
                    cout << " pv: " << pvLandmarksOnCV[i].getId() << " cv: " << cvLandmarks[j].getId() << endl;

                    cout << "dist diff (p1,p1) : " << distanceDiffP1 << endl;

                    cout << "dist diff (p2,p2) : " << distanceDiffP2 << endl;
                    //                            if(distanceDiffP1 < distanceDiffP2)
                    //                                lastDist = distanceDiffP1;
                    //                            else
                    //                                lastDist = distanceDiffP2;
                    matchingInfo.mapSurfaceID = i;
                    matchingInfo.cvSurfaceID = j;

                    if (distanceDiffP1 < distanceDiffP2)
                        matchingInfo.refPoint = 1;
                    else
                        matchingInfo.refPoint = 2;
                    matchingInfoForAll.push_back(matchingInfo);
                    success = true;

                }

            }
        }
    }
    cout<<matchingInfoForAll.size()<<endl;
    return success;
}

//for ego-centric mapping.
bool SameSurfaceFinderOdo::recognizeSameSurface(vector<ReferenceSurfaces> & allRefSurfaces, std::vector<Surface> pvLandmarks,
        std::vector<Surface> cvLandmarks, AngleAndDistance lastLocomotion) {
    cout << "Looking for same surfaces in the current view " << endl;

    std::vector<Surface> pvLandmarksOnCV = transform(pvLandmarks,lastLocomotion.angle,lastLocomotion.distance);
    
    //(debugging1)
    View temp(cvLandmarks); //only for display.
    temp.setRobotSurfaces(pvLandmarksOnCV);
    temp.setId(12);
    plotViewGNU("../outputs/Maps/LS-1-2.png", temp, true);
    //end of (debugging1)

    
    ReferenceSurfaces aRefSurface;
    vector<SameSurfaceInfo> matchingInfoForAll;
    
    bool success = matchSameSurfaces(matchingInfoForAll,pvLandmarksOnCV,cvLandmarks,10.0,500.0);
    
    for (unsigned int i = 0; i < matchingInfoForAll.size(); i++) {
        aRefSurface.setMapSurface(pvLandmarks[matchingInfoForAll[i].mapSurfaceID]);
        aRefSurface.setViewSurface(cvLandmarks[matchingInfoForAll[i].cvSurfaceID]);
        
        aRefSurface.setRefPoint(matchingInfoForAll[i].refPoint);
       
        allRefSurfaces.push_back(aRefSurface);
    }

    cout << "above is the result from recognition module" << endl;
    if (success == true) {
        cout << "Number of References: " << allRefSurfaces.size() << endl;

        //waitHere();
    } else {
        cout << "couldn't found any ref. :(" << endl;
        // waitHere();
    }


    return success;
}

//for allocentric mapping.
bool SameSurfaceFinderOdo::recognizeAllSameSurface(vector<ReferenceSurfaces> & allRefSurfaces, const vector<View> views, std::vector<Surface> pvLandmarks,
        std::vector<Surface> cvLandmarks, AngleAndDistance lastLocomotion) {
    cout << "Looking for same surfaces in the current view " << endl;

    std::vector<Surface> pvLandmarksOnCV = transform(pvLandmarks,lastLocomotion.angle,lastLocomotion.distance);
    
    //(debugging1)
    View temp(cvLandmarks); //only for display.
    temp.setRobotSurfaces(pvLandmarksOnCV);
    temp.setId(12);
    plotViewGNU("../outputs/Maps/LS-1-2.png", temp, true);
    //end of (debugging1)
    
    ReferenceSurfaces aRefSurface;
    vector<SameSurfaceInfo> matchingInfoForAll;
    
    bool success = matchSameSurfaces(matchingInfoForAll,pvLandmarksOnCV,cvLandmarks,10.0,500.0);
    cout<<matchingInfoForAll.size()<<endl;
    vector<Surface> cvLandmarksOnPV = transformB(cvLandmarks,lastLocomotion.angle,lastLocomotion.distance);
    for (unsigned int i = 0; i < matchingInfoForAll.size(); i++) {
        //aRefSurface.setMapSurface(views.back().getSurfaces()[matchingInfoForAll[i].mapSurfaceID]);
        aRefSurface.setMapSurface(cvLandmarksOnPV[matchingInfoForAll[i].cvSurfaceID]);
        aRefSurface.setViewSurface(cvLandmarks[matchingInfoForAll[i].cvSurfaceID]);
        
        aRefSurface.setRefPoint(matchingInfoForAll[i].refPoint);
       
        allRefSurfaces.push_back(aRefSurface);
    }

    cout << "above is the result from recognition module" << endl;
    if (success == true) {
        cout << "Number of References: " << allRefSurfaces.size() << endl;

        //waitHere();
    } else {
        cout << "couldn't found any ref. :(" << endl;
        // waitHere();
    }


    return success;
}