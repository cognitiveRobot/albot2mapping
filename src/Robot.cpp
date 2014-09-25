#include <fstream>
#include "Robot.h"


Robot::Robot()
{
    
}

Robot::~Robot()
{
    
}


void Robot::connect(int argc, char **argv, ArSimpleConnector* connector)
{
    Aria::init();
    step = 0;



    if (!(*connector).parseArgs() || argc > 1)
    {
        Aria::logOptions();
        Aria::shutdown();
        Aria::exit(1);
    }


    // Try to connect, if we fail exit
    if (!(*connector).connectRobot(&robot))
    {
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
    Pos.x =0;
    Pos.y =0;
    Pos.z =0;
    
    
}


void Robot::disconnect()
{
    robot.disconnect();
    Aria::shutdown();
}


void Robot::incStep()
{
    step++;
}
    
int Robot::getStep()
{
    return step;
}


void Robot::move()
{
        
        // Get odometer and orientation angle before moving
    robot.lock();
    double odoDistanceOld = robot.getOdometerDistance();
    double globalAngleOld = robot.getTh();
    robot.unlock();

    double angleToMove;
    double distanceToMove;

    while(true)
    {
		cout << endl << "How much to turn? ";
		cin >> angleToMove;
		cout << "How much to move? ";
		cin >> distanceToMove;

		if(cin.fail())
		{
			cout << "Invalid number, try again!" << endl << endl;
			cin.clear();
			cin.ignore(1000, '\n');
		}
		else
		{
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
    

  
    
    updatePos(odoDistanceDiff, angleDiff);
    incStep();
    
    
    
      //save surfaces like laser
    char sname[50];
    sprintf(sname, "%s%d", "../outputs/surfaces/coordTrans-",step);
    
    saveTravelInfo(odoDistanceDiff,angleDiff,sname);
        
}


inline void Robot::setHeading(double heading)
{
  ArTime start;

  robot.lock();
  //cout << "\n" << "Here  " << heading << "\n";
  robot.setDeltaHeading(heading);
  robot.unlock();

  start.setToNow();
  while (1)
  {
    robot.lock();
    if (robot.isHeadingDone(DIR_TOLERANCE))
    {
      //      printf("Finished turn\n");
      robot.unlock();
      break;
    }
    if (start.mSecSince() > TIMEOUT)
    {
      printf("turn timed out\n");
      robot.unlock();
      break;
    }
    robot.unlock();
    ArUtil::sleep(SHORT_PAUSE);
  }
  
}
    

inline void Robot::moveDistance(double distance, double velocity)
{
  ArPose prevpos,curpos;
  double distance_Travelled = 0;

  robot.lock();
  prevpos = curpos = robot.getPose();
  robot.unlock();
  robot.setVel(velocity);

  while (distance - 50 > distance_Travelled)
    {
      robot.lock();
      curpos = robot.getPose();
      robot.unlock();

      distance_Travelled = curpos.findDistanceTo(prevpos);
      
    }

  robot.stop();
    ArUtil::sleep(SHORT_PAUSE);
    

}


void Robot::updatePos(float r, float teta)
{
    int Xdiff, Ydiff;
    
    Pos.z += teta;
    teta = Pos.z;
    
    teta *= (float) M_PI/180;
    
    Xdiff = -r*sin(teta);
    Ydiff = r*cos(teta);
    Pos.x += Xdiff;
    Pos.y += Ydiff;
    
}

cv::Point3f Robot::getPos()
{
    return Pos;
}

void Robot::saveTravelInfo(double dist, double angle, char * filename) {
    ofstream outFile (filename, ios::out);

  // output ASCII header (row and column)
  outFile << 1 <<" "<< 2 <<endl;
  outFile << dist<<" ";
  outFile << angle<<endl;
  outFile.close();
}