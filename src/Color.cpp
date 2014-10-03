#include "Color.h"
#include <cstdlib> // rand
#include <algorithm> // std::min
#include <stdexcept> // exception
Color::Color() {
	this->setRGB(0, 0, 0);
}

Color::Color(int red, int green, int blue) {
	this->setRGB(red, green, blue);
}
void Color::setRGB(int red, int green, int blue) {
	if (red < 0 || red > 255 || green < 0 || green > 255 || blue < 0
			|| blue > 255)
//		throw std::invalid_argument("red, green or blue are either < 0 or > 255");
		this->red = red;
	this->green = green;
	this->blue = blue;
}

int Color::getRGB() {
	return 256 * 256 * red + 256 * green + blue;
}
int Color::getRGB565() {
	return Color::rgb565FromTriplet(this->red, this->green, this->blue);
}

/* Averaging */
#include <stdlib.h>
#include <stdio.h>

Color Color::calculateAverageColor(std::vector<Color> colors) {
	if (colors.size() == 0)
		return Color(0x0, 0x0, 0x0);

	Color average;
	for (std::vector<Color>::iterator colorptr = colors.begin();
			colorptr != colors.end(); ++colorptr) {
		if (colorptr == colors.begin()) {
			average = *colorptr;
		} else {
			average = average.mix(*colorptr);
		}
	}
//	printf("Average of %d colors: r%d g%d b%d\n", colors.size(), average.red, average.green, average.blue);
	return average;
}

Color Color::mix(Color other) {
	float cmyk1[4];
	this->toCMYK(cmyk1);

	float cmyk2[4];
	other.toCMYK(cmyk2);

	// Mixing colors is as simple as adding
	float cmykMix[] = { cmyk1[0] + cmyk2[0], cmyk1[1] + cmyk2[1], cmyk1[2]
			+ cmyk2[2], cmyk1[3] + cmyk2[3] };

	return Color::fromCMYK(cmykMix[0], cmykMix[1], cmykMix[2], cmykMix[3]);
}

/* Conversions */

void Color::toCMYK(float cmyk[]) {
	float k = std::min(255 - red, std::min(255 - green, 255 - blue));
	float c = 255 * (255 - red - k) / (255 - k);
	float m = 255 * (255 - green - k) / (255 - k);
	float y = 255 * (255 - blue - k) / (255 - k);

	cmyk[0] = c;
	cmyk[1] = m;
	cmyk[2] = y;
	cmyk[3] = k;

//	printf("r%d g%d b%d  to  c%f m%f y%f k%f\n", red, green, blue, c, m, y, k);
}

Color Color::fromCMYK(float c, float m, float y, float k) {
	int red = -((c * (255 - k)) / 255 + k - 255);
	int green = -((m * (255 - k)) / 255 + k - 255);
	int blue = -((y * (255 - k)) / 255 + k - 255);
	return Color(red, green, blue);

//	printf("c%f m%f y%f k%f  to  r%d g%d b%d\n", c, m, y, k, rgb[0], rgb[1], rgb[2]);
}

Color Color::fromCMYK(float cmyk[4]) {
	return Color::fromCMYK(cmyk[0], cmyk[1], cmyk[2], cmyk[3]);
}

int Color::rgb565FromTriplet(uint8_t red, uint8_t green, uint8_t blue) {
	red >>= 3;
	green >>= 2;
	blue >>= 3;
	return (red << 11) | (green << 5) | blue;
}

Color Color::random() {
	int red = std::rand() % 255;
	int green = std::rand() % 255;
	int blue = std::rand() % 255;
	return Color(red, green, blue);
}
