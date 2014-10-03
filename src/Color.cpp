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
//	if (red < 0 || red > 255 || green < 0 || green > 255 || blue < 0
//			|| blue > 255)
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

void Color::normalize(float rgbNormalized[3]) {
	rgbNormalized[0] = (float) red / 255;
	rgbNormalized[2] = (float) green / 255;
	rgbNormalized[2] = (float) blue / 255;
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

	// Mixing colors is as simple as adding according to http://stackoverflow.com/a/10142804/2225200
	float cmykMix[] = { cmyk1[0] + cmyk2[0], cmyk1[1] + cmyk2[1], cmyk1[2]
			+ cmyk2[2], cmyk1[3] + cmyk2[3] };
	Color::validateCMYK(cmykMix);

	return Color::fromCMYK(cmykMix[0], cmykMix[1], cmykMix[2], cmykMix[3]);
}

/* Conversions */

void Color::toCMYK(float cmyk[]) {
	float rgbNormalized[3];
	normalize(rgbNormalized);
	float redN = rgbNormalized[0], greenN = rgbNormalized[1], blueN =
			rgbNormalized[2];

	float black = std::min(1 - redN, std::min(1 - greenN, 1 - blueN));
	float cyan = (1 - redN - black) / (1 - black);
	float magenta = (1 - greenN - black) / (1 - black);
	float yellow = (1 - blueN - black) / (1 - black);

	cmyk[0] = cyan;
	cmyk[1] = magenta;
	cmyk[2] = yellow;
	cmyk[3] = black;
}

Color Color::fromCMYK(float c, float m, float y, float k) {
	float redNormalized = 1.0 - (c * (1.0 - k) + k);
	float greenNormalized = 1.0 - (m * (1.0 - k) + k);
	float blueNormalized = 1.0 - (y * (1.0 - k) + k);
	int red = redNormalized * 255, green = greenNormalized * 255, blue =
			blueNormalized * 255;
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

void Color::validateCMYK(float cmyk[4]) {
	if (cmyk[0] > 1.0 || cmyk[0] < 0 || cmyk[1] > 1.0 || cmyk[1] < 0
			|| cmyk[2] > 1.0 || cmyk[2] < 0 || cmyk[3] > 1.0 || cmyk[3] < 0) {
		char message[200];
		sprintf(message,
				"CMYK %.2f|%.2f|%.2f|%.2f is not in the range [0.0 - 1.0]",
				cmyk[0], cmyk[1], cmyk[2], cmyk[3]);
		throw std::invalid_argument(message);
	}
}
