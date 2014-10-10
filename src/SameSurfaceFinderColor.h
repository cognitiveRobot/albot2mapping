#ifndef SAME_SURFACE_FINDER_COLOR_H
#define SAME_SURFACE_FINDER_COLOR_H

#include "SameSurfaceFinder.h"

class SameSurfaceFinderColor: public SameSurfaceFinder {
private:
	const float SameSurfaceFinderColor::MINIMUM_MATCH_VALUE;

	float match(Surface surface1, Surface surface2);
	float match(Color c1, Color c2);
public:
	std::map<int, int> findSameSurfaces(std::vector<Surface> surfaces1,
			std::vector<Surface> surfaces2);
};

#endif /* SAME_SURFACE_FINDER_COLOR_H */
