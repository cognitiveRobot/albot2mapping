#include "ImageReader.h"
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <iostream>     // std::cout
#include <fstream>      // std::ifstream
/**
 * Partly from http://stackoverflow.com/questions/20582494/converting-any-colored-ppm-image-into-a-colored-image-with-only-8-colors
 */
void ImageReader::readPPM(char * filename, uint16_t targetColors[][240 /* View::COLOR_IMAGE_HEIGHT */]) {
	printf("Reading PPM %s\n", filename);
	std::ifstream in;
	in.open(filename);

	std::string header;
	int cols, rows, colors;
	int r, g, b;

	in >> header >> cols >> rows >> colors;

	for (int i = 0; i < rows; i++) {
		for (int j = 0; j < cols; j++) {
			in >> r >> g >> b;
			uint16_t rgb = rgb565_from_triplet(r, g, b);
			targetColors[i][j] = rgb;
		}
	}
//	printf("%d pixels\n", rows * cols);
	in.close();
}

uint16_t ImageReader::rgb565_from_triplet(uint8_t red, uint8_t green,
		uint8_t blue) {
	red >>= 3;
	green >>= 2;
	blue >>= 3;
	return (red << 11) | (green << 5) | blue;
}
