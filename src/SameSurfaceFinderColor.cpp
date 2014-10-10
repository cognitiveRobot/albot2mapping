#include "SameSurfaceFinderColor.h"
#include <math.h>
#include <stack>

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

std::map<int, int> SameSurfaceFinderColor::findSameSurfaces(
		std::vector<Surface> surfaces1, std::vector<Surface> surfaces2) {
	std::map<int, int> bestMatchIds12; // maps from surfaces1 ids to surfaces2 ids
	std::map<int, int> bestMatchIds21; // maps from surfaces2 ids to surfaces1 ids
	std::map<int, float> bestMatchValues;
	// TODO: handle >1 surfaces matching with the same surface
	printf("%d old surfaces, %d new ones\n", surfaces1.size(),
			surfaces2.size());

	std::stack<Surface> surfaces1ToHandle;
	for (std::vector<Surface>::size_type i1 = 0; i1 < surfaces1.size(); i1++) {
		surfaces1ToHandle.push(surfaces1[i1]);
	}

	while (!surfaces1ToHandle.empty()) {
		Surface s1 = surfaces1ToHandle.top();
		surfaces1ToHandle.pop();
		int s1Id = s1.getId();

		for (std::vector<Surface>::size_type i2 = 0; i2 < surfaces2.size();
				i2++) {
			int s2Id = surfaces2[i2].getId();
			printf("Comparing s1 #%d with s2 #%d\n", s1Id, s2Id);
			float matchValue = match(s1, surfaces2[i2]);
			if (bestMatchValues.count(s1Id) != 0 // already assigned
			&& bestMatchValues[s1Id] >= matchValue) { // and new value not better
				continue;
			}
			if (bestMatchIds21.count(s2Id) != 0) { // match already exists
				int currentS2S1Match = bestMatchIds21[s2Id]; // s1 match for this s2
				float currentMatchValue = bestMatchValues[currentS2S1Match];
				if (currentMatchValue >= matchValue) { // not good enough
					continue;
				} else { // new match is better
					surfaces1ToHandle.push(surfaces1[currentS2S1Match]); // back on stack
				}
			}

			bestMatchIds12[s1Id] = s2Id;
			bestMatchIds21[s2Id] = s1Id;
			bestMatchValues[s1Id] = matchValue;
		}
	}

	printf("%d same surface ids found:\n", bestMatchIds12.size());
	print(bestMatchIds12, bestMatchValues);
	return bestMatchIds12;
}

float SameSurfaceFinderColor::match(Surface s1, Surface s2) {
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

float SameSurfaceFinderColor::match(Color c1, Color c2) {
	int diff = std::abs(c1.red - c2.red) + std::abs(c1.green - c2.green)
			+ std::abs(c1.blue - c2.blue);
	diff = std::max(diff, 1);
	float maxDiff = 255 + 255 + 255;
	return maxDiff / diff;
}
