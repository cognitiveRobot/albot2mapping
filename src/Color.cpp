#include "Color.h"

Color::Color() {
}
Color::Color(int red, int green, int blue) {
	this->setRGB(red, green, blue);
}
void Color::setRGB(int red, int green, int blue) {
	this->red = red;
	this->green = green;
	this->blue = blue;
}

int Color::getRGB565() {
	return Color::rgb565FromTriplet(this->red, this->green, this->blue);
}

int Color::rgb565FromTriplet(uint8_t red, uint8_t green, uint8_t blue) {
	red >>= 3;
	green >>= 2;
	blue >>= 3;
	return (red << 11) | (green << 5) | blue;
}
