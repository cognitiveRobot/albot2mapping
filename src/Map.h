/* 
 * File:   Mapper.h
 * Author: Guillaume Diallo-Mulliez
 *
 * Created on May 30, 2013, 1:22 AM
 */

#ifndef MAPPER_H
#define	MAPPER_H


#include "View.h"


/* ------------------------- Namespaces ------------------------- */
using namespace std;
using namespace cv;

#define DISPARITY_HEIGHT 240.
#define DISPARITY_WIDTH 320.

class Map
{
    
private:
     
    vector <View> Views;
    vector <Obstacle> Obst;
    vector <Point3f> rbtPos;
    Mat drawing;
    int sizeX;
    int sizeY;
    int M;

    
       
public:
    

    Map(int _sizeX, int _sizeY);
    ~Map();

    void update(View newView);                          // Update the map according to the newView
    bool isBehind(Obstacle Old, Obstacle New, Point3f rbtPos); /* NOT SURE IF DONE CORRECTLY */ // Check if Old and New obstacles are concealing each other
    void coordTransf(Point3f *target, Point3f newCenter, double hX, double hY);                 // Transform the coordinates of *target with newCenter and homothetic transformation in X & Y directions
    void rotate(Point2f* target, Point2f Center,  float angle);                                 // Rotate *target point of angle around Center
    void display();                                     // Display map in output file
     
};



#endif	/* MAPPER_H */

