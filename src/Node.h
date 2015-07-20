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
    bool opened;
    bool closed;

    Node();
    Node(vector<Surface>& surfaces, float x, float y);
    ~Node();

    bool isAccessible();
    
    Node* getParent();
    void setParent(Node *parent);
    
    double computeF(Node *previousNode);
    
    double computeH(Node *goal);

    double computeG();

};

#endif /* NODE_H */