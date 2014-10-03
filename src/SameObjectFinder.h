#ifndef SAME_OBJECT_FINDER_H
#define SAME_OBJECT_FINDER_H

#include <vector>
#include <utility> 	// pair
#include "View.h"

class SameObjectFinder {
public:
	/**
	 * @return the id-pairs of the same surfaces
	 */
	std::vector<pair<int, int>> findSameObjects(
			std::vector<Surface> surfaces1, std::vector<Surface> surfaces2);
};

#endif /* SAME_OBJECT_FINDER_H */
