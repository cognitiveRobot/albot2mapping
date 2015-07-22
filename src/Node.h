#ifndef NODE_H
#define NODE_H

#include "PointAndSurface.h"
#include "Surface.h"

const int ROBOT_WIDTH=700;

class Node {
private:
    double f;
    double h;
    double g;

    Node *parent;

    bool accessible;

public:
    PointXY point;
    
    bool empty;

    Node();
    Node(vector<Surface>& surfaces, float x, float y, Node *parent);
    ~Node();
    
     bool operator==(const Node & other) const {

        return this->point == other.point;
    }
     
     bool operator<(const Node & other) const {

        return (this->point.getX()<other.point.getX()) || (this->point.getX()==other.point.getX() && this->point.getY()<other.point.getY());
    }

    bool isAccessible();
    
    Node* getParent();
    void setParent(Node *parent);

    double computeF(Node *goal);
    
    double getF();
    
    void setF(double f);
    
    vector<Node*> findNeighbours(vector<Surface> surfaces);
    
    bool isClosed(list<Node*> closedSet);
    
    Node* inOpenSet(list<Node*> openSet);

};

#endif /* NODE_H */