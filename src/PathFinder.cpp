#include "PathFinder.h"

#include "View.h"
#include "Printer.h"

#include <algorithm>


PathFinder::PathFinder() {
    nextGoal.angle = 0.0;
    nextGoal.distance = 0.0;
}

PathFinder::~PathFinder() {}

bool PathFinder::findNextGoal(const View& cView, AngleAndDistance & nG) {
    cout<<endl<<endl<<"----Finding Next Destination----"<<endl;
    cout<<cView.getId()<<endl;
    if(nG.distance < 1 && cView.getId() == 0) {
        cout<<BOLDGREEN<<"Looging for first goal point"<<RESET<<endl;
    }
    std::vector<Surface> landmarks = cView.getLandmarks();
    
    cout<<"Number of Landmarks in this view: "<<landmarks.size()<<endl;
    if(landmarks.size() == 0) {
        cout<<BOLDRED<<"Couldn't find any landmark from here!!!"<<RESET<<endl;
        return false;
    }
   
    
    std::sort(landmarks.begin(), landmarks.end(), SortBasedOnLength);
    
    cout<<"Sorted landmarsk "<<endl;
    displaySurfaces(landmarks);
    
    Printer printer;
    printer.plotLines(landmarks,Color(0,0,255));
    printer.plotLine(landmarks.back().getP1(),landmarks.back().getP2(),Color(0,255,0));
    cv::imwrite("selectedLandmark.jpg",printer.getCanvas());
    
    //robot facing in cv
    Surface robotFacing(0,0,0,30.0);
    Surface nextGoalDir(0,0,landmarks.back().midPoint().x,landmarks.back().midPoint().y);
    
    cout<<"Distance to target landmark: "<<nextGoalDir.length()<<endl;
    //setting output.
    nG.angle = robotFacing.getAngleWithSurface(nextGoalDir);
    nG.distance = nextGoalDir.length() - 70;
    
//    cout<<"Angle to next goal: "<<nG.angle<<endl;
//    cout<<"Distance to next goal: "<<nG.distance<<endl;
      
    
    return true;
}