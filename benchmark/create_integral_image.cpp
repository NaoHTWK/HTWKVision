#include "create_integral_image.h"

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <x86intrin.h>

#include <chrono>
#include <functional>
#include <iostream>

#include "create_integral_image_config.h"
#include "create_integral_image_naive.h"
#include "create_integral_image_naive_unrolled.h"
#include "create_integral_image_two_rows.h"
#include "create_integral_image_planar_yuv.h"

using namespace std;
using namespace std::chrono;

extern int integralImg[IMAGE_WIDTH * IMAGE_HEIGHT];

void measureTime(std::function<void (void)> expr)
{
    high_resolution_clock::time_point t1 = high_resolution_clock::now();
    expr();
    high_resolution_clock::time_point t2 = high_resolution_clock::now();
    duration<double> timespan = duration_cast<duration<double>>(t2 - t1);
    std::cout << "time: " << timespan.count() * 1000 << "ms\n\n";
}

bool testImages(int* integralImg, int* integralImgOrig) {
    for(int i = 0; i < IMAGE_WIDTH*IMAGE_HEIGHT; i++) {
        if(integralImg[i] != integralImgOrig[i]) {
            printf("Difference y,x: %d,%d now %d vs should be %d\n", i/IMAGE_HEIGHT, i%IMAGE_WIDTH, integralImg[i], integralImgOrig[i]);
            return false;
        }
    }

    return true;
}

int main(int argc, char** argv)
{
    uint8_t* img;
    uint8_t* imgCpy;

//    int rc0 = posix_memalign((void**)&yout, 16, RESULT_WIDTH_IN_M128*RESULT_HEIGHT*sizeof(*yout));
//    int rc1 = posix_memalign((void**)&uout, 16, RESULT_WIDTH_IN_M128*RESULT_HEIGHT*sizeof(*uout));
//    int rc2 = posix_memalign((void**)&vout, 16, RESULT_WIDTH_IN_M128*RESULT_HEIGHT*sizeof(*vout));
    int rc3 = posix_memalign((void**)&img,  16, PIXEL_PER_IMAGE);
    int rc4 = posix_memalign((void**)&imgCpy,16, PIXEL_PER_IMAGE);
//    int rc3 = posix_memalign((void**)&fieldMap, 16, width*height*sizeof(*fieldMap));

    if((/* rc0|rc1|rc2| */rc3|rc4) != 0) {
        printf("Error allocating memory for SSE! %s %d\n", __FILE__, __LINE__);
        exit(1);
    }

    FILE* fp = fopen("image.yuv422", "r");
    fread(img, PIXEL_PER_IMAGE, 1, fp);
    fclose(fp);

    createIntegralImageNaiveInit();
    createIntegralImageNaiveUnrolledInit();
    createIntegralImageTwoRowsInit();
    createIntegralImagePlanarYuvInit();

    int count = atoi(argv[1]);

    int integralImgOrig[IMAGE_WIDTH * IMAGE_HEIGHT];

    createIntegralImageNaive(img);
    memcpy(integralImgOrig, integralImg, IMAGE_WIDTH*IMAGE_HEIGHT*sizeof(*integralImg));

    printf("Recrusive variant\n");
    measureTime([&] {
        for(int i = 0; i < count; i++) {
            createIntegralImageNaive(img);
        }
    });

    printf("Recrusive variant unrolled with a flavor of SSE\n");
    measureTime([&] {
        for(int i = 0; i < count; i++) {
            createIntegralImageNaiveUnrolled(img);
        }
    });

    if(testImages(integralImg, integralImgOrig) != true)
        return -1;


//    printf("Version from paper unrolled 4 Times\n");
//    measureTime([&] {
//        for(int i = 0; i < count; i++) {
//            createIntegralImageTwoRows(img);
//        }
//    });

//    if(testImages(integralImg, integralImgOrig) != true)
//        return -1;

    printf("Version from planar yuv\n");
    measureTime([&] {
        for(int i = 0; i < count; i++) {
            createIntegralImagePlanarYuv(img);
        }
    });


    return 0;
}
