/*
 * Author : Segolene Minjard
 */

#include "Node.h"
#include "PointAndSurface.h"
#include "GeometryFuncs.h"

Node::Node() {
    point=PointXY();
    accessible=true;
    opened=false;
    closed=false;
}

Node::Node(vector<Surface>& surfaces, float x, float y) {
    point=PointXY(x,y);
    opened=false;
    closed=false;
    accessible=true;
    
    vector<PointXY> robotSpace;
    robotSpace.push_back(PointXY(x-ROBOT_WIDTH/2, y-ROBOT_WIDTH/2));
    robotSpace.push_back(PointXY(x-ROBOT_WIDTH/2, y+ROBOT_WIDTH/2));
    robotSpace.push_back(PointXY(x+ROBOT_WIDTH/2, y+ROBOT_WIDTH/2));
    robotSpace.push_back(PointXY(x+ROBOT_WIDTH/2, y-ROBOT_WIDTH/2));
    for(unsigned int i=0; i<surfaces.size(); i++){
        if(surfaceIntersectsPolygon(surfaces[i], robotSpace)){
            accessible=false;
            break;
        }
    }
}
Node::~Node(){
    
}

bool Node::isAccessible() {
    return accessible;
}

Node* Node::getParent() {
    return parent;
}
void Node::setParent(Node *parent){
    this->parent=parent;
}

double Node::computeF(Node *previousNode){
    f=this->point.distFrom(previousNode->point);
    return f;
}

double Node::computeH(Node *goal){
    PointXY intermerdiate (goal->point.getX(), this->point.getY());
    h=this->point.distFrom(intermerdiate)+intermerdiate.distFrom(goal->point);
    return h;
}

double Node::computeG(){
    g=f+h;
    return g;
}