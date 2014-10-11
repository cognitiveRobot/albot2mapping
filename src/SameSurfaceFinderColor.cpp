#include "SameSurfaceFinderColor.h"
#include <math.h>
#include <stack>

//float SameSurfaceFinderColor::MINIMUM_MATCH_VALUE = 100; // chosen rather arbitrarily so far

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
	std::map<int, Surface> bestMatches21; // maps from surfaces2 ids to surfaces1
	std::map<int, float> bestMatchValues12, bestMatchValues21;

	printf("%d old surfaces, %d new ones\n", surfaces1.size(),
			surfaces2.size());

	// TODO: two s1 map to same s2

	// hold a stack of surfaces1 that have yet to find a perfect partner
	// if a s1 is replaced by another s1', s1 will be re-added to the stack
	std::stack<Surface> surfaces1ToHandle;
	for (std::vector<Surface>::size_type i1 = 0; i1 < surfaces1.size(); i1++) {
		surfaces1ToHandle.push(surfaces1[i1]);
	}

	// compare all surfaces with each other
	// iterating until a perfect partner has been found for everyone
	while (!surfaces1ToHandle.empty()) {
		Surface s1 = surfaces1ToHandle.top();
		surfaces1ToHandle.pop();
		int s1Id = s1.getId();

		for (std::vector<Surface>::size_type i2 = 0; i2 < surfaces2.size();
				i2++) {
			Surface s2 = surfaces2[i2];
			int s2Id = s2.getId();
			float matchValue = match(s1, s2);
			if (bestMatchIds12.count(s1Id) != 0 // s2 match for s1 already assigned
			&& bestMatchValues12[s1Id] >= matchValue) { // and new value not better
				continue;
			}
			if (bestMatches21.count(s2Id) != 0) { // s1 match already exists
				Surface currentS1Match = bestMatches21[s2Id]; // s1 match for this s2
				int currentS1MatchId = currentS1Match.getId();
				float currentMatchValue = bestMatchValues12[currentS1MatchId];
				if (currentMatchValue >= matchValue) { // not good enough
					continue;
				} else { // new match is better
					bestMatchIds12.erase(currentS1MatchId);
					bestMatches21.erase(s2Id);
					bestMatchValues12.erase(currentS1MatchId);
					surfaces1ToHandle.push(currentS1Match); // back on stack
				}
			}

			bestMatchIds12[s1Id] = s2Id;
			bestMatches21[s2Id] = s1;
			bestMatchValues12[s1Id] = matchValue;
		}
	}

	printf("%d same surface ids found:\n", bestMatchIds12.size());
	print(bestMatchIds12, bestMatchValues12);
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
