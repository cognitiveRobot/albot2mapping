//#include <bits/stl_vector.h>
#include <fstream>
#include "View.h"


/* Functions definitions for Obstacles class */

Obstacle::Obstacle()
{
    
}
Obstacle::~Obstacle()
{
    
}
    

void Obstacle::addPoint(Point2f newPoint)
{
    points.push_back(newPoint);
}

void Obstacle::coordTransf(Point2f newCenter, float hX, float hY)
{
    
    for (unsigned int i=0; i< points.size(); i++)
    {
        
        // Change the center of the frame reference
        points[i].x -= newCenter.x ;
        points[i].y -= newCenter.y;
        
        
        // Homothetic transformation to adapt to axes
        points[i].x *= hX;
        points[i].y *= hY;    
      
      
        
    }
    
}


void Obstacle::rotate(Point3f Center)
{
    float distance;
    float teta, angle;
    
    
        teta = 0;
        angle = Center.z;
        angle *= M_PI;
        angle /= (float) 180;
        
        
        P1.x = P1.x;
        P1.y = P1.y;
        

        distance = sqrt(P1.x*P1.x + P1.y*P1.y);                        // Getting distance from center

        if(P1.x < 0){
                teta = atan(P1.y/P1.x) + M_PI;
        }
        else teta = atan(P1.y/P1.x);
        

        // Set angle for rotated point
        teta += angle;
        
   
    // Set new position
        P1.x = distance*cos(teta);
        P1.y = distance*sin(teta);
    
        


       P2.x = P2.x;
       P2.y = P2.y;
       
       distance = sqrt(P2.x*P2.x + P2.y*P2.y);                        // Getting distance from center

        if(P2.x < 0){
                teta = atan(P2.y/P2.x) + M_PI;
        }
        else teta = atan(P2.y/P2.x);
        

        // Set angle for rotated point
        teta += angle;
        
   
    // Set new position
        P2.x = distance*cos(teta);
        P2.y = distance*sin(teta);
    

}

vector <Point2f> Obstacle::getPoints()
{
    return points;
}

void Obstacle::clearPoints()
{
    points.clear();
}

void Obstacle::setP1(float X, float Y)
{
    P1.x = X;
    P1.y = Y;
}

void Obstacle::setP2(float X, float Y)
{
    P2.x = X;
    P2.y = Y;
}

Point2f Obstacle::getP1()
{
    return P1;
}

Point2f Obstacle::getP2()
{
    return P2;
}



void Obstacle::setSurface()
{
    
        Vec4f vecLine;                          // Orientation vector
        vector <Point2f> myPoints = points;
        float distance = 0;                     // Size of the surface
        
        /* Set surface size by measuring the longest distance between 2 points */
        for(unsigned i=0; i<myPoints.size(); i++)
        {
                for(unsigned j=0; j<myPoints.size(); j++)
                {
                    if (distance < sqrt( (myPoints[i].x-myPoints[j].x)*(myPoints[i].x-myPoints[j].x) + (myPoints[i].y-myPoints[j].y)*(myPoints[i].y-myPoints[j].y) ) )
                        distance = sqrt( (myPoints[i].x-myPoints[j].x)*(myPoints[i].x-myPoints[j].x) + (myPoints[i].y-myPoints[j].y)*(myPoints[i].y-myPoints[j].y) );   
                }
        }
        
        /* Set the orientation vector to fit with the set of points*/
        fitLine(myPoints, vecLine, CV_DIST_L2, 0, 0.01, 0.01);
        
        /* Set the ends of the surface */
        setP1(myPoints[0].x, myPoints[0].y);
        setP2(distance*vecLine[0] + P1.x, distance*vecLine[1] + P1.y);

}



/* Functions definitions for View class */

View::View()
{
    Id = 0;
    step = DISPARITY_WIDTH / 100;
    sizeX = 500;
    sizeY = 500;
}

View::~View()
{
    
}

void View::setId(int value)
{
    Id = value;
}
int View::getId()
{
    return Id;
}


void View::setRobotPos(float X, float Y, float angle)
{
    robot.x = X;
    robot.y = Y;
    robot.z = angle;
}

 Point3f View::getRobotPos()
 {
     return robot;
 }


void View::setView(TriclopsContext triclops, TriclopsImage16 depthImage, Point3f robotPos)
{
    clearView();
    cout<<endl<<"Getting view from image"<<endl;
   
    RNG rng(time(NULL));
    Obstacle tmpObst = Obstacle();
    double avgZ = 0;
    double preAvgZ = 0;
    int nbPoints = 0, n = 0;
    int disparity;
    float x, y, z, leftX, rightX;
    int boundX, boundY, boundW, boundH;
    int count=0;
    
 
    
    for (int i = 0; i < DISPARITY_WIDTH - step; i +=step) {             //For each slices of the image
        count++;
        boundX = i;
        boundY = DISPARITY_HEIGHT / 2;
        boundW = step;
        boundH = 25;
        avgZ = 0;
        n = 0;
        nbPoints = 0;
        
        
       
        
        disparity = depthImage.data[boundY * depthImage.rowinc + boundX + boundW];
        triclopsRCD16ToXYZ(triclops, boundY, boundX, disparity, &x, &y, &z);
        leftX = x;
        triclopsRCD16ToXYZ(triclops, boundY, boundX + boundW, disparity, &x, &y, &z);
        rightX = x;


        /* Get the average depth value avgZ */
        for (int j = 0; j < boundH; j++) {
            for (int k = 0; k < boundW; k++) {

                disparity = depthImage.data[(j + boundY) * depthImage.rowinc / 2 + (k + boundX)];
                if (disparity != 0 && disparity < 65280 && disparity > 10) {
                    triclopsRCD16ToXYZ(triclops, j + boundY, k + boundX, disparity, &x, &y, &z);
                    if (z < 10) {
                        nbPoints++;
                        avgZ += z;
                    }
                }
                n++;
                
            }
        }
        avgZ /= nbPoints;
        
        /* Set the new points for the View */
        Point2f newPoint(i + 1/2, avgZ);
        
       
        if (avgZ==avgZ)
        {
             if (abs(avgZ-preAvgZ)<0.15)                // If close enough to previous depth value
                {
                        tmpObst.addPoint(newPoint);     // Consider it belongs to same Obstacle
                        
                }
                else
                {
                        Point2f Center((float) DISPARITY_WIDTH/2, 0.15);
                        tmpObst.coordTransf(Center, sizeX/DISPARITY_WIDTH, 100);        // Adapt the coordinates
                        addObst(tmpObst);                                               // Add the Obstacle to the View
                        tmpObst.clearPoints();                                          // Clear the temporary Obstacle
                        tmpObst.addPoint(newPoint);                                     // Create new temporary Obstacle with new point
                }
             preAvgZ=avgZ;
        }
        
        
        
        
    }
    
    //save surfaces like laser
    char sname[50];
    sprintf(sname, "%s%d", "../outputs/surfaces/surfaces-",Id);
    saveSurfaces(Obst,sname);

    // Update the position of the robot for this new view
    robot.x = robotPos.x;
    robot.y = robotPos.y;
    robot.z = robotPos.z;
                         
}


void View::setSurfaces()
{
    for (unsigned int i = 0; i<Obst.size(); i++)                // For each obstacle
    {
        if (Obst[i].getPoints().size() > 3)                     // If the Obstacle is relevant
                Obst[i].setSurface();                           // Set surface
    }
}

void View::rotate()
{
    for (unsigned int i = 0; i<Obst.size(); i++)                // For each obstacle
    {
        Obst[i].rotate(robot);                                  // Rotate according to robot angle
    }
}

void View::translate()
{
    /* Translate the ends P1 & P2 of each Obstacle according to robot position */
    for (unsigned int i = 0; i<Obst.size(); i++)                
    {
        Obst[i].setP1((float)Obst[i].getP1().x + robot.x/10, (float)Obst[i].getP1().y + robot.y/10);            
        Obst[i].setP2((float)Obst[i].getP2().x + robot.x/10, (float)Obst[i].getP2().y + robot.y/10);
    }
}


void View::cleanView()
{
    vector <Obstacle> tmpObsts;
    
    for (unsigned int i = 0; i<Obst.size(); i++)                // For each obstacle
    {
        if(Obst[i].getPoints().size() > 4 )                                                                   // If there is enough points in 1 obstacle...
        {       
            tmpObsts.push_back(Obst[i]);
        }
    }
    Obst.clear();
    Obst = tmpObsts;
}



 void View::addObst(Obstacle newObst)
 {
     Obst.push_back(newObst);
 }
 
 
 vector <Obstacle> View::getObsts()
 {
     return Obst;
 }

 
 void View::clearView()
 {
     Obst.clear();
 }
 
 
 Mat View::display()
{
    vector <Obstacle> tmp1Obst;              // Temporary obstacles vector to update Obst
    drawing = Mat::zeros(Size(sizeX, sizeY), CV_8UC3);
    drawing.setTo(Scalar(255, 255, 255));
    Point2f P;
    Obstacle curObst;
    

    
    // Draw the robot
    Point2f rbt0(sizeX/2, sizeY-15);
    Point2f rbt1(rbt0.x - 15, rbt0.y -15);
    Point2f rbt2(rbt0.x + 15, rbt0.y +15);
    Point2f rbt3(rbt0.x, rbt0.y -30);
    rectangle(drawing, rbt1, rbt2, Scalar(0,0,255), 3, 8, 0);
    line(drawing, rbt0, rbt3, Scalar(0,0,0), 2, 8, 0);
     
for (unsigned int i = 0; i<Obst.size(); i++)
{
curObst = Obst[i];                  // Transform the coordinates to have the right frame of reference

        if(curObst.getPoints().size() > 4 ) // If there is enough points in 1 obstacle...
{tmp1Obst.push_back(Obst[i]);
}
}
Obst.clear();
Obst = tmp1Obst;

    // Draw the obstacles
cout << Obst.size() << " Obstacles in this view" << endl;
unsigned int numline = 0;
    for (unsigned int i = 0; i<Obst.size(); i++)                // For each obstacle
    {
        curObst = Obst[i];                  // Transform the coordinates to have the right frame of reference

        if(curObst.getPoints().size() > 4 ) // If there is enough points in 1 obstacle...
        {       
                curObst.setSurface();                                                     // Construct a line with the points
                // Adapt the point to the openCV Mat drawing
                curObst.setP1(rbt0.x + curObst.getP1().x,rbt0.y - curObst.getP1().y );
                curObst.setP2(rbt0.x + curObst.getP2().x,rbt0.y - curObst.getP2().y );

                line(drawing, curObst.getP1(), curObst.getP2(), Scalar(0,0,255), 3, 8, 0);    // Draw that line
       numline++;

//tmp1Obst.push_back(Obst[i]);
 }
//curObst = tmp1Obst[i];
        
        // Draw the points
//cout << curObst.getPoints().size() << "Obstacles points in this view" << endl;
        for (unsigned int j = 0; j<curObst.getPoints().size(); j++)
        {    
                P.x = rbt0.x + curObst.getPoints()[j].x;
                P.y = rbt0.y - curObst.getPoints()[j].y;
                circle(drawing, P, 1, Scalar(255, 0, 0), 1, 8, 0);      
        }

    }
    cout << numline<< " Number of lines in this view" << endl;
//Obst.clear();
            //Obst = tmp1Obst;

    // Display the view in a file
    char filename[50];
    sprintf(filename, "%s%d%s", "../outputs/Views/View", Id, ".jpg");
    imwrite(filename, drawing);
    
    return drawing;
}

 void View::saveSurfaces(vector <Obstacle> obstacles, char * filename) {
     cout<<"Saving surface file"<<endl;
 ofstream outFile (filename, ios::out);

    // Output ASCII header (row and column)
    outFile << obstacles.size() << " " << 4 << endl;

    // 8 digits should be more than enough
   // outFile << fixed;
    //outFile.precision(10);

    for (int i=0;i<int(obstacles.size());i++)
    {
	//outFile << obstacles[i].getID() << " ";
        if(obstacles[i].getPoints().size() > 4) {
        outFile << obstacles[i].getPoints()[0].x << " ";
        outFile << obstacles[i].getPoints()[0].y << " ";
        outFile << obstacles[i].getPoints()[obstacles[i].getPoints().size()-1].x << " ";
        outFile << obstacles[i].getPoints()[obstacles[i].getPoints().size()-1].y << endl;
        }
    }

    outFile.close();
    cout<<"Surface file saved"<<endl;
 }