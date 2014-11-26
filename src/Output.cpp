#include "Output.h"
#define _USE_MATH_DEFINES

Output::Output() {

}

Output::~Output() {

}

int Output::writePpm(char* szFilename, unsigned char* pucBuffer, int width, int height) {
    FILE* stream;
    stream = fopen(szFilename, "wb");
    if (stream == NULL) {
        perror("Can't open image file");
        return 1;
    }

    fprintf(stream, "P6\n%u %u 255\n", width, height);
    fwrite(pucBuffer, 3 * width, height, stream);
    fclose(stream);
    return 0;
}



















