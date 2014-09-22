#include "Map.h"


Map::Map(int _sizeX, int _sizeY):sizeX(_sizeX), sizeY(_sizeY)
{
    sizeX = 2000;
    sizeY = 2000;
    M = 0;
}


Map::~Map()
{
    
}


void Map::coordTransf(Point3f *target, Point3f newCenter, double hX, double hY)
{
    
        // Homothetic transformation to adapt to axes
        target->x *= hX;
        target->y *= hY;        
        // Change the center of the frame reference
        target->x += newCenter.x;
        target->y += newCenter.y;

}


void Map::rotate(Point2f* target, Point2f Center,  float angle)
{
    float r;
    float teta;
    
    target->x = target->x - Center.x;
    target->y = target->y - Center.y;
    target->y = -target->y;

        r = sqrt(target->x*target->x + target->y*target->y);                        // Getting distance from center

        if(target->x < 0){
            if(target->y < 0){
                teta = atan(target->y/target->x) - M_PI;
            }
            else{
                teta = atan(target->y/target->x) + M_PI;
            }
        }
        else teta = atan(target->y/target->x);
        

        // Set angle for rotated point
        angle *= M_PI;
        angle /= (float) 180;
        teta += angle;
        
   
    // Set new position
        target->x = r*cos(teta);
        target->y = r*sin(teta);
    
        target->x = target->x + Center.x;
        target->y = Center.y - target->y;
        
    
}


void Map::update(View newView)
{
    // Robot
        Point3f tmpPos;
        Point3f O(sizeX/2, sizeY/2, 0);
       //Point3f O(sizeX/2, sizeY-15, 0);
        
        
        // Comparing obstacle variables
        vector <Obstacle> tmpObst,tmpObst1 ;              // Temporary obstacles vector to update Obst
        int counter;
        unsigned int Osize,Osize1 ;                     // Original size of Obst before adding new Obstacles to the map
        
        cout << Obst.size() << " Intial Obstacles in the map" << endl;
        /* Adapt newView to compare Obstacles to previous ones */
        newView.setSurfaces();
        newView.rotate();
        newView.translate();
        
        
        if(Obst.size() == 0){           // If the map is empty, place the robot in initial position and add obstacles
                // Set robot position on the map and add it to rbtPos
cout << Obst.size() << "No Obstacle in the map" << endl;
                tmpPos = newView.getRobotPos();
                coordTransf(&tmpPos, O,(double) 1/10, (double) -1/10);
                newView.setRobotPos(tmpPos.x, tmpPos.y, tmpPos.z);
                rbtPos.push_back(tmpPos);
             
                // Add the Obstacles to the map
                Obst = newView.getObsts();
             
                newView.clearView();
cout << Obst.size() << " start Obstacle in the map" << endl;
        }        
        else                            // Else compare obstacles
        {
            
            // Move the robot
            tmpPos = newView.getRobotPos();
    
            coordTransf(&tmpPos, O,(double) 1/10, (double) -1/10);
            newView.setRobotPos(tmpPos.x, tmpPos.y, tmpPos.z);
            rbtPos.push_back(tmpPos);
            
            Osize = Obst.size();Osize1 = Obst.size();
   //old and then new        
 for(unsigned int j=0; j<newView.getObsts().size(); j++)         // For each obstacle of the new View
{
                counter=0;

            for(unsigned int i=0; i < Osize; i++)           // For every obstacle of the map
            
                {
                    
                 if(isBehind(Obst[i], newView.getObsts()[j], rbtPos[rbtPos.size()]) || isBehind(newView.getObsts()[j], Obst[i], rbtPos[rbtPos.size()]))     // Check if any Obstacles are concealing others
                   {  
                        counter++;

                   }
 if(j == 0 ){ 

tmpObst.push_back(Obst[i]);  // Add it to the updated map
                    
                 }  
               }
                
              if(counter == 0 ){                                      // If an old Obstacle isn't concealing nor concealed....
tmpObst.push_back(newView.getObsts()[j]);      // add all obstacles from new view     // Add the new Obstacles
	                        
             }
         }
     
// new and then old
 //for(unsigned int i=0; i < Osize; i++)           // For every obstacle of the map

//{
           //     counter=0;

          // for(unsigned int j=0; j<newView.getObsts().size(); j++)         // For each obstacle of the new View
            
                              
         //      {
                    
       //          if(isBehind(Obst[i], newView.getObsts()[j], rbtPos[rbtPos.size()]) || isBehind(newView.getObsts()[j], Obst[i], rbtPos[rbtPos.size()]))     // Check if any Obstacles are concealing others
                // {  
              //          counter++;

          //          }
//if(i == 0 ){ 
//tmpObst.push_back(newView.getObsts()[j]);
 
                    
     //             }  
   //             }
   //             cout << counter << "Counter" << endl;
 //               if(counter == 0 ){  
//tmpObst.push_back(Obst[i]);  // Add it to the updated map
                                    // If an old Obstacle isn't concealing nor concealed....
       //add all obstacles from new view     // Add the new Obstacles
	                        
   //           }
 //          }
 
// both new and old
//for(unsigned int i=0; i < Osize1; i++)           // For every obstacle of the map

//{tmpObst.push_back(Obst[i]); 
//}
//           for(unsigned int j=0; j<newView.getObsts().size(); j++)         // For each obstacle of the new View
                        
      //          {
                  
//tmpObst.push_back(newView.getObsts()[j]);
                    
//                  }  
               
Obst.clear();
           Obst = tmpObst; 
                                                     // replace the map by the updated one
       cout << tmpObst.size() << " Obstacles in the new map" << endl;
		tmpObst.clear();    
             
        }
    
}

bool Map::isBehind(Obstacle Old, Obstacle New, Point3f rbtPos)
{
    bool ret = false;
    
    // Translate to compare as same view
    Old.setP1(Old.getP1().x - rbtPos.x, Old.getP1().y - rbtPos.y);
    Old.setP2(Old.getP2().x - rbtPos.x, Old.getP2().y - rbtPos.y);
    New.setP1(Old.getP1().x - rbtPos.x, New.getP1().y - rbtPos.y);
    New.setP2(Old.getP2().x - rbtPos.x, New.getP2().y - rbtPos.y);
    
    
    if(Old.getP1().y > 0 && Old.getP2().y > 0){
        if(Old.getP1().x == 0){
                if(New.getP1().x > 0 && New.getP1().y > (float) Old.getP2().y/Old.getP2().x*New.getP1().x && New.getP2().x > 0 && New.getP2().y > (float) Old.getP2().y/Old.getP2().x*New.getP2().x){
                        ret = true;
                }
        }
        else if(Old.getP1().x < 0 && Old.getP2().x == 0){
                if( (New.getP1().y > (float) Old.getP1().y/Old.getP1().x*New.getP1().x && New.getP1().x < 0) || (New.getP2().y > (float) Old.getP1().y/Old.getP1().x*New.getP2().x && New.getP2().x < 0) ){
                        ret = true;
                }
        }
        else if(Old.getP1().x < 0 && Old.getP2().x < 0){
                if( (New.getP1().y > (float) Old.getP1().y/Old.getP1().x*New.getP1().x && New.getP1().y < (float) Old.getP2().y/Old.getP2().x*New.getP1().x) || (New.getP2().y > (float) Old.getP1().y/Old.getP1().x*New.getP2().x && New.getP2().y < (float) Old.getP2().y/Old.getP2().x*New.getP2().x) ){
                  ret = true;
                }
        }
        else if(Old.getP1().x < 0 && Old.getP2().x > 0){
                if( (New.getP1().y > (float) Old.getP1().y/Old.getP1().x*New.getP1().x && New.getP1().y > (float) Old.getP2().y/Old.getP2().x*New.getP1().x) || (New.getP2().y > (float) Old.getP1().y/Old.getP1().x*New.getP2().x && New.getP2().y > (float) Old.getP2().y/Old.getP2().x*New.getP2().x) ){
                        ret = true;
                }
        }
        else if(Old.getP1().x > 0 && Old.getP2().x > 0){
                if( (New.getP1().y < (float) Old.getP1().y/Old.getP1().x*New.getP1().x && New.getP1().y > (float) Old.getP2().y/Old.getP2().x*New.getP1().x) || (New.getP2().y < (float) Old.getP1().y/Old.getP1().x*New.getP2().x && New.getP2().y > (float) Old.getP2().y/Old.getP2().x*New.getP2().x) ){
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


void Map::display()
{
    
    drawing = Mat::zeros(Size(sizeX, sizeY), CV_8UC3);
    drawing.setTo(Scalar (255, 255, 255));
    Point2f P11,P21, P1, P2;
    Point2f O(0,0);
    Obstacle curObst1;
    
    // Draw robot positions
    for (unsigned int i = 0; i<rbtPos.size(); i++)
    {  
        Point2f rbt0(rbtPos[i].x, rbtPos[i].y);
        Point2f rbt1(rbt0.x - 15, rbt0.y - 15);
        Point2f rbt2(rbt0.x + 15, rbt0.y + 15);
        Point2f rbt3(rbt0.x, rbt0.y - 30);
        rotate(&rbt3, rbt0, rbtPos[i].z);                               // Set the robot into the right direction
        rectangle(drawing, rbt1, rbt2, Scalar(0,0,255), 3, 8, 0);
        line(drawing, rbt0, rbt3, Scalar(0,0,0), 3, 8, 0); 
    }
    
    // Draw Obstacles Surfaces

//cout << Obst.size() << " Obstacles drawn in new map" << endl;
    for (unsigned int i = 0; i<Obst.size(); i++)
    { 
        // Adapt the point to the openCV Mat drawing
       P1 = Point2f(rbtPos[0].x + Obst[i].getP1().x,rbtPos[0].y - Obst[i].getP1().y );
        P2 = Point2f(rbtPos[0].x + Obst[i].getP2().x,rbtPos[0].y - Obst[i].getP2().y );
       line(drawing, P1, P2, Scalar(0,0,255), 3, 8, 0);                // Draw the actual line
    }
        


//new codes for consistency  
//unsigned int numline1 = 0;
//for (unsigned int i = 0; i<Obst.size(); i++)
  //  {
       // curObst1 = Obst[i];                  // Transform the coordinates to have the right frame of reference

    //    if(curObst1.getPoints().size() > 4 ) // If there is enough points in 1 obstacle...
     //   {       
    //            curObst1.setSurface();                                                     // Construct a line with the points
                // Adapt the point to the openCV Mat drawing
    //            curObst1.setP1(rbtPos[0].x + curObst1.getP1().x,rbtPos[0].y - curObst1.getP1().y );
    //            curObst1.setP2(rbtPos[0].x + curObst1.getP2().x,rbtPos[0].y - curObst1.getP2().y );
//
    //            line(drawing, curObst1.getP1(), curObst1.getP2(), Scalar(0,0,255), 3, 8, 0);    // Draw that line
    //   numline1++;
// }
//}
//cout << numline1<< " Number of lines in this map" << endl;
// new code completes



    // Display the map in a file
    char filename[50];
    sprintf(filename, "%s%d%s", "../outputs/Maps/Map", M, ".jpg");
    imwrite(filename, drawing);
    
    
    M++;
}
