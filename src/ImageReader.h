/*
 * File:   ImageReader.h
 * Author: Martin Schrimpf
 *
 * Created on September 23, 2014, 7:13 PM
 */

#ifndef IMAGE_READER_H
#define	IMAGE_READER_H

#include "View.h"

class ImageReader {
public:
	void readPPM(char * filename, uint16_t colors[][240 /* View::COLOR_IMAGE_HEIGHT */]);
	uint16_t rgb565_from_triplet(uint8_t red, uint8_t green, uint8_t blue);
};

#endif /* IMAGE_READER_H */
