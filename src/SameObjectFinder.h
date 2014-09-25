#ifndef SAME_OBJECT_FINDER_H
#define SAME_OBJECT_FINDER_H

#include <vector>
#include <utility> 	// pair
#include "View.h"

class SameObjectFinder {
public:
	std::vector<pair<Obstacle, Obstacle>> findSameObjects(
			std::vector<Obstacle> obstacles1, std::vector<Obstacle> obstacles2);
};

#endif /* SAME_OBJECT_FINDER_H */
