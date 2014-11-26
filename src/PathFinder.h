#ifndef PATH_FINDER_H
#define PATH_FINDER_H

#include "Robot.h"



class PathFinder {
private:
    AngleAndDistance nextGoal;
    
public:
    PathFinder();
    ~PathFinder();
    
    
    bool findNextGoal(const View & cView, AngleAndDistance & nG);
};


#endif /* PATH_FINDER_H */