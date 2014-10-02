#include <stdio.h>
#include <stdlib.h>
#include "Color.h"

int main(int argc, char *argv[]) {
	Color red(255, 0, 0);
	Color green(0, 255, 0);
	Color blue(0, 0, 255);
	Color avgRG = red.mix(green);
	printf("Average of red and green: r%d g%d b%d\n", avgRG.red, avgRG.green,
			avgRG.blue);
}
