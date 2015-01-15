#include "SameSurfaceFinderOdo.h"
#include "Printer.h"
#include "ImageProcessing.h"

bool SameSurfaceFinderOdo::recognizeSameSurface(vector<Surface> & refSurfaces, std::vector<Surface> pvLandmarks,
        std::vector<Surface> cvLandmarks, AngleAndDistance lastLocomotion) {
    cout << "Looking for same surfaces in the current view " << endl;
    cout << "No. of Lsurfaces in PV: " << pvLandmarks.size() << endl;
    cout << "No. of Lsurfaces in CV: " << cvLandmarks.size() << endl;

    cout << "Angle " << lastLocomotion.angle << " dist " << lastLocomotion.distance << endl;

    double angle = lastLocomotion.angle * CONVERT_TO_RADIAN; // degree to randian.
    //find cv center in the pv coordinate frame.
    //need to convert robot position from mm to cm.
    double newX = (lastLocomotion.distance / 1.0) * sin(-angle); //x= d*cos(th) = d*cos(90-angle) = d*sin(angle) //as aris give - value for right turn
    double newY = (lastLocomotion.distance / 1.0) * cos(-angle); //y=d*sin(th)=d*sin(90-angle)=d*cos(angle)

    cout << "x " << newX << " y " << newY << endl;

    //transform pvLandmarks to cv coordinate frame.
    std::vector<Surface> pvLandmarksOnCV;
    for (unsigned int i = 0; i < pvLandmarks.size(); i++) {
        pvLandmarksOnCV.push_back(pvLandmarks[i].transFrom(newX, newY, angle));
    }
    cout << "No. of pv surfaces on CV " << pvLandmarksOnCV.size() << endl;
    
    //(debugging1)
    View temp(cvLandmarks); //only for display.
    temp.setRobotSurfaces(pvLandmarksOnCV);
    temp.setId(12);
    plotViewGNU("../outputs/Maps/LS-1-2.png",temp);
    //end of (debugging1)
    
    bool success = false;
    double angleDiff, distanceDiffP1, distanceDiffP2, distP1, distP2;
    double lastAngleDiff = 5.0;
    double lastDist = 10000.0;
    for (unsigned int i = 0; i < pvLandmarksOnCV.size(); i++) {
        for (unsigned int j = 0; j < cvLandmarks.size(); j++) {
            angleDiff = pvLandmarksOnCV[i].getAngleWithSurface(cvLandmarks[j]);
            
            if (abs(angleDiff) < 5.0) {
                cout << "angle Diff " << angleDiff << endl;
                cout << pvLandmarksOnCV[i].getId() << " cv " << cvLandmarks[j].getId() << endl;
                distanceDiffP1 = cvLandmarks[j].distFromP1ToPoint(pvLandmarksOnCV[i].getP1().x, pvLandmarksOnCV[i].getP1().y);
                cout << "dist diff (p1,p1) : " << distanceDiffP1 << endl;
                distanceDiffP2 = cvLandmarks[j].distFromP2ToPoint(pvLandmarksOnCV[i].getP2().x, pvLandmarksOnCV[i].getP2().y);
                cout << "dist diff (p2,p2) : " << distanceDiffP2 << endl;
                if (distanceDiffP1 < 400.0 or distanceDiffP2 < 400.0) {                    
                    distP1 = cvLandmarks[j].distFromP1ToPoint(0, 0);
                    distP2 = cvLandmarks[j].distFromP2ToPoint(0, 0);
                    cout<<"dist p1 and p2 to cv robotP "<<distP1<<" "<<distP2<<endl;
                    if ( distanceDiffP1 < lastDist ||  distanceDiffP2 < lastDist) {
                        // if (abs(angleDiff) < lastAngleDiff) {
                            lastAngleDiff = abs(angleDiff);
                            if(distanceDiffP1 < distanceDiffP2)
                                lastDist = distanceDiffP1;
                            else
                                lastDist = distanceDiffP2;
                            cout << endl<<"Ref found" << endl;
                            refSurfaces.clear();
                            refSurfaces.push_back(pvLandmarks[i]);
                            refSurfaces.push_back(cvLandmarks[j]);
                            success = true;
                            //waitHere();
                      //  }
                    }
                }

            }
        }
    }
    
    cout << "above is the result from recognition module" << endl;
    if(success == true) {
        cout<<"Ref is found :)"<<endl;
        waitHere();
    }else {
        cout<<"couldn't found any ref. :("<<endl;
        waitHere();
    }
    

    return success;
}