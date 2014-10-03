#ifndef COLOR_H
#define COLOR_H

#include <stdint.h>
#include <vector>

class Color {
public:
	int red, green, blue;
	/**
	 * Zero values for red, green and blue
	 */
	Color();
	Color(int red, int green, int blue);
	void setRGB(int red, int green, int blue);

	int getRGB();
	int getRGB565();
	void toCMYK(float* cmyk);

	Color mix(Color other);
	static Color calculateAverageColor(std::vector<Color> colors);

	static Color fromCMYK(float c, float m, float y, float k);
	static Color fromCMYK(float cmyk[4]);
	static int rgb565FromTriplet(uint8_t red, uint8_t green, uint8_t blue);
	static Color random();
};

#endif /* COLOR_H */
