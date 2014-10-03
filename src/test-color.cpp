#include <stdio.h>
#include <stdlib.h>
#include "Color.h"

const int debug = 1;

void testMixing(Color c1, Color c2, Color expected);
void testConversion(Color col);
void checkEquals(Color c1, Color c2);

int main(int argc, char *argv[]) {

	Color red(255, 0, 0);
	Color green(0, 255, 0);
	Color blue(0, 0, 255);

	Color yellow(255, 255, 0);
	Color purple(255, 0, 255);

	/* conversion */
	testConversion(red);

	/* mixing */
	testMixing(red, green, yellow);
}

void testConversion(Color col) {
	printf("** Converting from RGB to CMYK and back **\n");
	if (debug)
		printf("    %-10s: %d | %d | %d\n", "RGB Color", col.red, col.green,
				col.blue);

	float cmyk[4];
	col.toCMYK(cmyk);
	if (debug)
		printf("    %10s: %.0f%% | %.0f%% | %.0f%% | %.0f%%\n", "CMYK Color",
				cmyk[0] * 100, cmyk[1] * 100, cmyk[2] * 100, cmyk[3] * 100);

	Color colPrime = Color::fromCMYK(cmyk);
	checkEquals(col, colPrime);
}

void checkEquals(Color c1, Color c2) {
	int redEq = c1.red == c2.red;
	int greenEq = c1.green == c2.green;
	int blueEq = c1.blue == c2.blue;
	if (!redEq || !greenEq || !blueEq) {
		printf(
				"XX Colors are different in terms of%s%s%s: (%d|%d|%d) <> (%d|%d|%d)\n",
				!redEq ? " red" : "", !greenEq ? " green" : "",
				!blueEq ? " blue" : "", c1.red, c1.green, c1.blue, c2.red,
				c2.green, c2.blue);
	}
}

void testMixing(Color c1, Color c2, Color expected) {
	Color avg = c1.mix(c2);
	if (debug)
		printf("Average of (%d|%d|%d) and (%d|%d|%d): %d | %d | %d\n", c1.red,
				c1.green, c1.blue, c2.red, c2.green, c2.blue, avg.red,
				avg.green, avg.blue);
	checkEquals(avg, expected);
}
