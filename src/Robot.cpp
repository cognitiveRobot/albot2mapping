#include <fstream>
#include "Robot.h"

Robot::Robot() {
    
    fromHome.angle = 0.0;
    fromHome.distance = 0.0;
}

Robot::~Robot() {

}

void Robot::connect(int argc, char **argv, ArSimpleConnector* connector) {
    Aria::init();
    step = 0;



    if (!(*connector).parseArgs() || argc > 1) {
        Aria::logOptions();
        Aria::shutdown();
        Aria::exit(1);
    }


    // Try to connect, if we fail exit
    if (!(*connector).connectRobot(&robot)) {
        printf("Could not connect to robot... exiting\n");
        Aria::shutdown();
        exit(EXIT_FAILURE);
    }

    // Turn on the motors, turn off amigobot sounds
    robot.runAsync(true);
    robot.comInt(ArCommands::ENABLE, 1);
    robot.comInt(ArCommands::SOUNDTOG, 0);
    robot.lock();
    robot.clearDirectMotion();
    robot.unlock();


    // Initialize Position
    Pos.x = 0;
    Pos.y = 0;
    Pos.z = 0;


}

void Robot::disconnect() {
    robot.disconnect();
    Aria::shutdown();
}

void Robot::incStep() {
    step++;
}

int Robot::getStep() {
    return step;
}

void Robot::moveToTheGoal(AngleAndDistance & goal) {
     // Get odometer and orientation angle before moving
    robot.lock();
    double odoDistanceOld = robot.getOdometerDistance();
    double globalAngleOld = robot.getTh();
    robot.unlock();
    
    double angle = goal.angle;
    double dist;
    if(goal.distance > 100) {
        dist = 1000;
        goal.distance = goal.distance - 100;
    }
    else {
        dist = (goal.distance) * CONVERT_TO_MM;
        goal.distance = 0;
    }
    
    
    
    
    cout<<"NextD - A & D: "<<angle<<" & "<<dist<<endl;
    
   char tkStep;
   cout << endl << endl << "Shall I go? (y/n) "; // Ask user if continue
    cin >> tkStep;
    
     // Turn and move
    if(tkStep != 'n' && tkStep != 'N') {
    setHeading(angle);
    moveDistance(dist, 255);
    }
    
    goal.angle = 0;

    // Stop the robot and wait a bit
    robot.lock();
    robot.stop();
    robot.clearDirectMotion();
    robot.unlock();
    ArUtil::sleep(1500);

    // Get the actual distance traveled in this step
    robot.lock();
    double angleDiff = robot.getTh() - globalAngleOld;
    double odoDistanceDiff = robot.getOdometerDistance() - odoDistanceOld;
    robot.unlock();

    cout << "In this step: traveled distance = " << odoDistanceDiff << "; angle = " << angleDiff << endl;


    //save last locomotion
    lastLocomotion.angle = angleDiff;
    lastLocomotion.distance = odoDistanceDiff;
    
  


    updatePos(odoDistanceDiff, angleDiff);
    incStep();



    //save angle and distance in a txt file.
    char sname[50];
    sprintf(sname, "%s%d", "../outputs/surfaces/coordTrans-", step);
    saveTravelInfo(odoDistanceDiff, angleDiff, sname);
    
}

void Robot::move() {

    // Get odometer and orientation angle before moving
    robot.lock();
    double odoDistanceOld = robot.getOdometerDistance();
    double globalAngleOld = robot.getTh();
    robot.unlock();

    double angleToMove;
    double distanceToMove;

    while (true) {
        cout << endl << "How much to turn? ";
        cin >> angleToMove;
        cout << "How much to move? ";
        cin >> distanceToMove;

        if (cin.fail()) {
            cout << "Invalid number, try again!" << endl << endl;
            cin.clear();
            cin.ignore(1000, '\n');
        } else {
            break;
        }
    }

    // Turn and move
    setHeading(angleToMove);
    moveDistance(distanceToMove, 255);

    // Stop the robot and wait a bit
    robot.lock();
    robot.stop();
    robot.clearDirectMotion();
    robot.unlock();
    ArUtil::sleep(1500);

    // Get the actual distance traveled in this step
    robot.lock();
    double angleDiff = robot.getTh() - globalAngleOld;
    double odoDistanceDiff = robot.getOdometerDistance() - odoDistanceOld;
    robot.unlock();

    cout << "In this step: traveled distance = " << odoDistanceDiff << "; angle = " << angleDiff << endl;


    //save last locomotion
    lastLocomotion.angle = angleDiff;
    lastLocomotion.distance = odoDistanceDiff;

    //save totoal angle n dist from home. path integration 
    fromHome.angle += angleDiff;
    fromHome.distance += odoDistanceDiff;
    
     //save totoal angle n dist from home. path integration 
    loaclSpaceHome.angle += angleDiff;
    loaclSpaceHome.distance += odoDistanceDiff;

    updatePos(odoDistanceDiff, angleDiff);
    incStep();



    //save angle and distance in a txt file.
    char sname[50];
    sprintf(sname, "%s%d", "../outputs/surfaces/coordTrans-", step);
    saveTravelInfo(odoDistanceDiff, angleDiff, sname);

}

inline void Robot::setHeading(double heading) {
    ArTime start;

    robot.lock();
    //cout << "\n" << "Here  " << heading << "\n";
    robot.setDeltaHeading(heading);
    robot.unlock();

    start.setToNow();
    while (1) {
        robot.lock();
        if (robot.isHeadingDone(DIR_TOLERANCE)) {
            //      printf("Finished turn\n");
            robot.unlock();
            break;
        }
        if (start.mSecSince() > TIMEOUT) {
            printf("turn timed out\n");
            robot.unlock();
            break;
        }
        robot.unlock();
        ArUtil::sleep(SHORT_PAUSE);
    }

}

inline void Robot::moveDistance(double distance, double velocity) {
    ArPose prevpos, curpos;
    double distance_Travelled = 0;

    robot.lock();
    prevpos = curpos = robot.getPose();
    robot.unlock();
    robot.setVel(velocity);

    while (distance - 50 > distance_Travelled) {
        robot.lock();
        curpos = robot.getPose();
        robot.unlock();

        distance_Travelled = curpos.findDistanceTo(prevpos);

    }

    robot.stop();
    ArUtil::sleep(SHORT_PAUSE);


}

void Robot::updatePos(float r, float teta) {
    int Xdiff, Ydiff;

    Pos.z += teta;
    teta = Pos.z;

    teta *= (float) M_PI / 180;

    Xdiff = -r * sin(teta);
    Ydiff = r * cos(teta);
    Pos.x += Xdiff;
    Pos.y += Ydiff;

}

vector<Surface> Robot::getRectRobot() {
    vector<Surface> robotSurfaces;
    Surface aSurface;
     aSurface.set(0,0,0,40);
     robotSurfaces.push_back(aSurface);//+y
     aSurface.set(-20,-20,-20,20);
     robotSurfaces.push_back(aSurface);//left side
     aSurface.set(-20,20,20,20);
     robotSurfaces.push_back(aSurface);//front side
     aSurface.set(20,20,20,-20);
     robotSurfaces.push_back(aSurface);//right side
     aSurface.set(20,-20,-20,-20);
     robotSurfaces.push_back(aSurface);//back side
     
     return robotSurfaces;
}

cv::Point3f Robot::getPos() {
    return Pos;
}

AngleAndDistance Robot::getLastLocomotion() {
    return lastLocomotion;
}

AngleAndDistance Robot::getFromHome() const{
    return fromHome;
}

AngleAndDistance Robot::getLocalSpaceHome() const {
    return loaclSpaceHome;
}
    AngleAndDistance Robot::setLocalSpaceHome(double a, double b) {
        loaclSpaceHome.angle = a;
    loaclSpaceHome.distance = b;
    }

void Robot::saveTravelInfo(double dist, double angle, char * filename) {
    ofstream outFile(filename, ios::out);

    // output ASCII header (row and column)
    outFile << 1 << " " << 2 << endl;
    outFile << dist << " ";
    outFile << angle << endl;
    outFile.close();
}