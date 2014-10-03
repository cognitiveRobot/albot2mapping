#include <stdio.h>
#include <stdlib.h>
#include "Color.h"

void checkEquals(Color c1, Color c2) {
	int redEq = c1.red == c2.red;
	int greenEq = c1.green == c2.green;
	int blueEq = c1.blue == c2.blue;
	if (!redEq || !greenEq || !blueEq) {
		printf("Colors are different in terms of%s%s%s: (%d|%d|%d) <> (%d|%d|%d)\n",
				!redEq ? " red" : "", !greenEq ? " green" : "",
				!blueEq ? " blue" : "", c1.red, c1.green, c1.blue, c2.red,
				c2.green, c2.blue);
	}
}

int main(int argc, char *argv[]) {
	Color red(255, 0, 0);
	Color green(0, 255, 0);
	Color blue(0, 0, 255);

	/* conversion */
	float cmyk[4];
	red.toCMYK(cmyk);
	Color redPrime = Color::fromCMYK(cmyk);
	checkEquals(red, redPrime);

	/* mixing */
	Color avgRG = red.mix(green);
	printf("Average of red and green: r%d g%d b%d\n", avgRG.red, avgRG.green,
			avgRG.blue);
}
