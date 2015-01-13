/* 
 * File:   renameFiles.cpp
 * Author: mhossain
 *
 * Created on 13 January 2015, 1:52 PM
 */

#include <cstdlib>

#include "Robot.h"
#include "PointAndSurface.h"
#include "View.h"
#include "Printer.h"

using namespace std;

int main() {
    int newStart, oldEnd, oldStart;
    cout << "Starting file number: " << endl;
    cin>>oldStart;
    cout << endl << "Last old File number: " << endl;
    cin >> oldEnd;
    cout << "new name starts from ? " << endl;
    cin>>newStart;
    
    Robot Albot;
    char coordFile[50], pointFile[50];
    vector<PointXY> points2D;

    do {
        sprintf(coordFile, "%s%d", "../outputs/surfaces/coordTrans-", oldStart);
        readOdometry(Albot, coordFile);
        
        sprintf(pointFile, "%s%d", "../outputs/pointCloud/points2D-", oldStart+1);
        points2D = readASCIIPoints2D(pointFile);   
        cout<<"(Read)"<<endl<<coordFile<<endl<<pointFile<<endl;


        //write again.
        sprintf(coordFile, "%s%d", "../outputs/renamedFiles/coordTrans-", newStart);
        Albot.saveTravelInfo(Albot.getLastLocomotion().distance, Albot.getLastLocomotion().angle, coordFile);
        
        sprintf(pointFile, "%s%d", "../outputs/renamedFiles/points2D-", newStart+1);
        writeASCIIPoints2D(pointFile, points2D);   
        cout<<"(Write)"<<endl<<coordFile<<endl<<pointFile<<endl;

        newStart++;
        oldStart++;

    } while (oldStart <= oldEnd);
    return 0;
}

