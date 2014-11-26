#include "Aria.h"

/** @example demo.cpp General purpose testing and demo program, using ArMode
 *    classes to provide keyboard control of various robot functions.
 *
 *  demo uses ArMode subclasses from ARIA. These modes
 *  provide keyboard control of various aspects and accessories of 
 *  the robot, and can be re-used in your programs if you wish.  
 *  The ArMode classes are defined in %ArModes.cpp.
 *
 *  "demo" is a useful program for testing out the operation of the robot
 *  for diagnostic or demonstration purposes.  Other example programs
 *  focus on individual areas.
 */

int main(int argc, char** argv) {
    Aria::init();
    ArSimpleConnector connector(&argc, argv);
    ArRobot robot;
    // ArSick sick;
    if (!connector.parseArgs() || argc > 1) {
        Aria::logOptions();
        Aria::shutdown();
        Aria::exit(1);
    }

    ArKeyHandler keyHandler;
    Aria::setKeyHandler(&keyHandler);
    robot.attachKeyHandler(&keyHandler);


    // robot.addRangeDevice(&sick);
    // Try to connect, if we fail exit
    if (!connector.connectRobot(&robot)) {
        //  cout << "Could not connect to robot... exiting" << endl;
        Aria::shutdown();
        return 1;
    }
    
        ArSonarDevice sonar;
  robot.addRangeDevice(&sonar);
    // Turn on the motors, turn off amigobot sounds
    robot.runAsync(true);

    robot.lock();

    ArModeTeleop teleop(&robot, "teleop", 't', 'T');
    teleop.activate();

    robot.comInt(ArCommands::ENABLE, 1);
    robot.comInt(ArCommands::SOUNDTOG, 0);
    robot.unlock();



    //   ArSickLogger myLogger(&robot,&sick,100,1,"logFile.log");
    robot.waitForRunExit();

    Aria::exit(0);

}

