#include "SameObjectFinderColor.h"
#include <math.h>

void print(std::map<int, int> matchIds, std::map<int, float> matchValues) {
	std::map<int, int>::const_iterator idIter = matchIds.begin();
	std::map<int, float>::const_iterator valIter = matchValues.begin();
	while (idIter != matchIds.end() && valIter != matchValues.end()) {
		printf("[%d] %d (value %.2f)\n", idIter->first, idIter->second,
				valIter->second);
		idIter++;
		valIter++;
	}
}

std::map<int, int> SameObjectFinderColor::findSameSurfaces(
		std::vector<Surface> surfaces1, std::vector<Surface> surfaces2) {
	std::map<int, int> bestMatchIds;
	std::map<int, float> bestMatchValues;
	// TODO: handle >1 surfaces matching with the same surface
	printf("%d old surfaces, %d new ones\n", surfaces1.size(),
			surfaces2.size());
	for (std::vector<Surface>::size_type i1 = 0; i1 < surfaces1.size(); i1++) {
		for (std::vector<Surface>::size_type i2 = 0; i2 < surfaces2.size();
				i2++) {
			float matchValue = match(surfaces1[i1], surfaces2[i2]);
			int s1Id = surfaces1[i1].getId(), s2Id = surfaces2[i2].getId();
//			printf("Comparing s1 #%d with s2 #%d\n", s1Id, s2Id);
			if (bestMatchValues.count(s1Id) == 0 // not yet assigned
			|| matchValue > bestMatchValues[s1Id]) { // or worse value
				bestMatchIds[s1Id] = s2Id;
				bestMatchValues[s1Id] = matchValue;
			}
		}
	}
	printf("%d same surface ids found:\n", bestMatchIds.size());
	print(bestMatchIds, bestMatchValues);
	return bestMatchIds;
}

float SameObjectFinderColor::match(Surface s1, Surface s2) {
	std::vector<Color> fixedColors = s1.getColors();
	std::vector<Color> movingColors = s2.getColors();
	float bestMatch = 0;
	// start by putting s2 left of s1 using the moveIndex
	// and leave s1 fixed
	// then move s2 step by step to the right using the moveIndex
	int totalMoveDistance = fixedColors.size() + movingColors.size() - 1;
	for (int moveIndex = -movingColors.size() + 1;
			moveIndex < totalMoveDistance; moveIndex++) {
		int leftBound = std::max(0, moveIndex); // most left area that overlaps between surfaces
		int rightBound = std::min(fixedColors.size(),
				moveIndex + movingColors.size());
		float matchValue = 0;
		// compare all color areas in current bounds
		for (int i = leftBound; i < rightBound; i++) {
			Color c1 = fixedColors[i];
			Color c2 = movingColors[-moveIndex + i]; // make moveIndex positive for vector
			matchValue += match(c1, c2);
		}
		// update if better
		bestMatch = std::max(bestMatch, matchValue);
	}
	return bestMatch;
}

float SameObjectFinderColor::match(Color c1, Color c2) {
	int diff = std::abs(c1.red - c2.red) + std::abs(c1.green - c2.green)
			+ std::abs(c1.blue - c2.blue);
	float maxDiff = 255 + 255 + 255;
	return maxDiff / diff;
}
