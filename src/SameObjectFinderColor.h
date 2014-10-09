#ifndef SAME_OBJECT_FINDER_COLOR_H
#define SAME_OBJECT_FINDER_COLOR_H

#include "SameObjectFinder.h"

class SameObjectFinderColor: public SameObjectFinder {
private:
	float match(Surface surface1, Surface surface2);
	float match(Color c1, Color c2);
public:
	std::map<int, int> findSameSurfaces(std::vector<Surface> surfaces1,
			std::vector<Surface> surfaces2);
};

#endif /* SAME_OBJECT_FINDER_COLOR_H */
