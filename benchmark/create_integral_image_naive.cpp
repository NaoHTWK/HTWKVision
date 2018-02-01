#include "create_integral_image_naive.h"

#include "create_integral_image_config.h"

#include <stdlib.h>

static const int iWidth=IMAGE_WIDTH/INTEGRAL_SCALE;
static const int iHeight=IMAGE_HEIGHT/INTEGRAL_SCALE;
int integralImg[IMAGE_WIDTH * IMAGE_HEIGHT];

static int8_t *lutCr;
static int8_t *lutCb;

void createIntegralImageNaiveInit()
{
    lutCb=(int8_t*)malloc(sizeof(*lutCb)*IMAGE_WIDTH);
    lutCr=(int8_t*)malloc(sizeof(*lutCr)*IMAGE_WIDTH);
    for(int i=1;i<IMAGE_WIDTH;i++){
        if((i&1)==0){
            lutCb[i]=5;
            lutCr[i]=3;
        }else{
            lutCb[i]=3;
            lutCr[i]=5;
        }
        while(lutCr[i]+i*2>=IMAGE_WIDTH*2)
            lutCr[i]-=4;
        while(lutCb[i]+i*2>=IMAGE_WIDTH*2)
            lutCb[i]-=4;
    }
    lutCb[0]=1;
    lutCr[0]=3;
}

inline uint8_t getCr(const uint8_t * const img, int32_t x, int32_t y) {
    return img[((x + y * IMAGE_WIDTH) << 1) + lutCr[x]];
}

inline uint8_t getY(const uint8_t * const img, int32_t x, int32_t y) {
    return img[(x + y * IMAGE_WIDTH) << 1];
}

#define getValue getCr

void createIntegralImageNaive(uint8_t *img) {
    integralImg[0]=getValue(img,0,0);
    for(int x=1;x<iWidth;x++){
        integralImg[x]=integralImg[x-1]+(getValue(img,x*INTEGRAL_SCALE,0));
    }
    for(int y=1;y<iHeight;y++){
        int sum=0;
        for(int x=0;x<iWidth;x++){
            int addr=x+y*iWidth;
            sum+=(getValue(img,x*INTEGRAL_SCALE,y*INTEGRAL_SCALE));
            integralImg[addr]=sum+integralImg[addr-iWidth];
        }
    }
}
