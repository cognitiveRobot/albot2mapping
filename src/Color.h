#ifndef COLOR_H
#define COLOR_H

#include <stdint.h>

class Color {
public:
	int red, green, blue;
	Color();
	Color(int red, int green, int blue);
	void setRGB(int red, int green, int blue);

	int getRGB565();
	static int rgb565FromTriplet(uint8_t red, uint8_t green,
			uint8_t blue);
};

#endif /* COLOR_H */
