#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <map>
#include "Color.h"
#include "SameSurfaceFinderColor.h"

const int debug = 1;

const Color red(255, 0, 0);
const Color green(0, 255, 0);
const Color blue(0, 0, 255);

const Color yellow(255, 255, 0);
const Color purple(255, 0, 255);
const Color cyan(0, 255, 255);
const Color white(255, 255, 255);

Surface find(std::vector<Surface> surfaces, int id) {
    for (std::vector<Surface>::size_type i = 0; i < surfaces.size(); i++) {
        if (surfaces[i].getId() == id)
            return surfaces[i];
    }
    throw std::invalid_argument("Id does not exist");
}

void print(std::vector<Color> colors) {
    printf("[");
    for (std::vector<Color>::size_type i = 0; i < colors.size(); i++) {
        printf("(r%d g%d b%d)", colors[i].red, colors[i].green, colors[i].blue);
        if (i < colors.size() - 1)
            printf(", ");
    }
    printf("]");
}

int main(int argc, char *argv[]) {
    Surface tmpSurface;
    std::vector<Color> colors;
    SameSurfaceFinderColor sameSurfaceFinder;

    std::vector<Surface> surfaces1, surfaces2;
    /* 1 */
    colors.clear();
    tmpSurface.reset();
    colors.push_back(red);
    colors.push_back(yellow);
    tmpSurface.setColors(colors);
    surfaces1.push_back(tmpSurface);

    colors.clear();
    tmpSurface.reset();
    colors.push_back(red);
    colors.push_back(yellow);
    colors.push_back(cyan);
    tmpSurface.setColors(colors);
    surfaces1.push_back(tmpSurface);

    colors.clear();
    tmpSurface.reset();
    colors.push_back(green);
    colors.push_back(blue);
    tmpSurface.setColors(colors);
    surfaces1.push_back(tmpSurface);

    /* 2 */
    colors.clear();
    tmpSurface.reset();
    colors.push_back(green);
    colors.push_back(blue);
    colors.push_back(yellow);
    tmpSurface.setColors(colors);
    surfaces2.push_back(tmpSurface);

    colors.clear();
    tmpSurface.reset();
    colors.push_back(red);
    colors.push_back(yellow);
    colors.push_back(cyan);
    tmpSurface.setColors(colors);
    surfaces2.push_back(tmpSurface);

    colors.clear();
    tmpSurface.reset();
    colors.push_back(purple);
    colors.push_back(white);
    tmpSurface.setColors(colors);
    surfaces2.push_back(tmpSurface);

    /* find */
    std::map<int, int> sameSurfaces = sameSurfaceFinder.findSameSurfaces(
            surfaces1, surfaces2);
    typedef std::map<int, int>::iterator it_type;
    for (it_type iterator = sameSurfaces.begin();
            iterator != sameSurfaces.end(); iterator++) {
        int s1Id = iterator->first, s2Id = iterator->second;
        Surface s1 = find(surfaces1, s1Id), s2 = find(surfaces2, s2Id);
        std::vector<Color> s1Colors = s1.getColors(), s2Colors = s2.getColors();
        printf("Match: s1#%d ", s1Id);
        print(s1Colors);
        printf(" with s2#%d ", s2Id);
        print(s2Colors);
        printf("\n");
    }
}
