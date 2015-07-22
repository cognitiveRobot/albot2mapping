/*
 * Author : Segolene Minjard
 */

#include "Node.h"
#include "PointAndSurface.h"
#include "GeometryFuncs.h"

Node::Node() {
    point = PointXY();
    accessible = true;
    empty = true;
}

Node::Node(vector<Surface>& surfaces, float x, float y, Node *parent) {
    point = PointXY(x, y);
    accessible = true;
    empty = false;
    this->parent = parent;

    vector<PointXY> robotSpace;
    robotSpace.push_back(PointXY(x - ROBOT_WIDTH / 2, y - ROBOT_WIDTH / 2));
    robotSpace.push_back(PointXY(x - ROBOT_WIDTH / 2, y + ROBOT_WIDTH / 2));
    robotSpace.push_back(PointXY(x + ROBOT_WIDTH / 2, y + ROBOT_WIDTH / 2));
    robotSpace.push_back(PointXY(x + ROBOT_WIDTH / 2, y - ROBOT_WIDTH / 2));
    for (unsigned int i = 0; i < surfaces.size(); i++) {
        if (surfaceIntersectsPolygon(surfaces[i], robotSpace)) {
            accessible = false;
            break;
        }
    }
}

Node::~Node() {

}

bool Node::isAccessible() {
    return accessible;
}

Node* Node::getParent() {
    return parent;
}

void Node::setParent(Node *parent) {
    this->parent = parent;
}

double Node::computeF(Node *goal) {
    g = this->point.distFrom(parent->point);
    PointXY intermerdiate(goal->point.getX(), this->point.getY());
    h = this->point.distFrom(intermerdiate) + intermerdiate.distFrom(goal->point);
    f = g + h;
    return f;
}

double Node::getF() {
    return f;
}

void Node::setF(double f) {
    this->f = f;
}

vector<Node*> Node::findNeighbours(vector<Surface> surfaces) {
    Node *neighbourParent = this;
    vector<Node*> neighbours;
    neighbours.push_back(new Node(surfaces, point.getX() - ROBOT_WIDTH / 2, point.getY() - ROBOT_WIDTH / 2, neighbourParent));
    neighbours.push_back(new Node(surfaces, point.getX() - ROBOT_WIDTH / 2, point.getY(), neighbourParent));
    neighbours.push_back(new Node(surfaces, point.getX() - ROBOT_WIDTH / 2, point.getY() + ROBOT_WIDTH / 2, neighbourParent));
    neighbours.push_back(new Node(surfaces, point.getX(), point.getY() - ROBOT_WIDTH / 2, neighbourParent));
    neighbours.push_back(new Node(surfaces, point.getX(), point.getY() + ROBOT_WIDTH / 2, neighbourParent));
    neighbours.push_back(new Node(surfaces, point.getX() + ROBOT_WIDTH / 2, point.getY() - ROBOT_WIDTH / 2, neighbourParent));
    neighbours.push_back(new Node(surfaces, point.getX() + ROBOT_WIDTH / 2, point.getY(), neighbourParent));
    neighbours.push_back(new Node(surfaces, point.getX() + ROBOT_WIDTH / 2, point.getY() + ROBOT_WIDTH / 2, neighbourParent));

    return neighbours;
}

bool Node::isClosed(list<Node*> closedSet) {
    list<Node*>::iterator it = closedSet.begin();
    while ((*it)->point.getX() <= this->point.getX() && it != closedSet.end()) {
        if ((*it)->point.getX() == this->point.getX() && (*it)->point.getY() == this->point.getY()) {
            return true;
        }
        it++;
    }
    return false;
}

Node* Node::inOpenSet(list<Node*> openSet) {
    list<Node*>::iterator it = openSet.begin();
    while ((*it)->point.getX() <= this->point.getX() && it != openSet.end()) {
        if ((*it)->point.getX() == this->point.getX() && (*it)->point.getY() == this->point.getY()) {
            return *it;
        }
        it++;
    }
    return 0;
}