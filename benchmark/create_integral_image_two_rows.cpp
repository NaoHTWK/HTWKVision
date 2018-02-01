#include "create_integral_image_two_rows.h"

#include "create_integral_image_config.h"

#include <stdlib.h>
#include <xmmintrin.h>

static const int iWidth=IMAGE_WIDTH/INTEGRAL_SCALE;
static const int iHeight=IMAGE_HEIGHT/INTEGRAL_SCALE;
extern int integralImg[IMAGE_WIDTH * IMAGE_HEIGHT];

static int8_t *lutCr;
static int8_t *lutCb;

void createIntegralImageTwoRowsInit()
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

/**
 * Paper: Integral Images: Efficient Algorithms for Their Computation and Storage in Resource-Constrained Embedded Vision Systems
 */
void createIntegralImageTwoRows(uint8_t *img) {
    integralImg[0]=getValue(img,0,0);
    for(int x=1;x<iWidth;x++){
        integralImg[x]=integralImg[x-1]+(getValue(img,x*INTEGRAL_SCALE,0));
    }

    for(int y=1;y<iHeight;y+=4){
        int sum0=0;
        int sum1=0;
        int sum2=0;
        int sum3=0;
        for(int x=0;x<iWidth;x++){
            const int addr0=x+(y+0)*iWidth;
            const int addr1=x+(y+1)*iWidth;
            const int addr2=x+(y+2)*iWidth;
            const int addr3=x+(y+3)*iWidth;

            sum0+=(getValue(img,x*INTEGRAL_SCALE,(y+0)*INTEGRAL_SCALE)); /* S(x+0,y) */
            sum1+=(getValue(img,x*INTEGRAL_SCALE,(y+1)*INTEGRAL_SCALE)); /* S(x+1,y) */
            sum2+=(getValue(img,x*INTEGRAL_SCALE,(y+2)*INTEGRAL_SCALE)); /* S(x+2,y) */
            sum3+=(getValue(img,x*INTEGRAL_SCALE,(y+3)*INTEGRAL_SCALE)); /* S(x+3,y) */

            const int tmpImg = integralImg[addr0-iWidth];
            const int tmp1 = sum0+sum1;
            const int tmp2 = tmp1+sum2;
            const int tmp3 = tmp2+sum3;

            integralImg[addr0]=tmpImg+sum0;
            integralImg[addr1]=tmpImg+tmp1;
            integralImg[addr2]=tmpImg+tmp2;
            integralImg[addr3]=tmpImg+tmp3;
        }
    }

    int sum0=0;
    int sum1=0;
    int sum2=0;

    const int y = iHeight-3;
    for(int x=0;x<iWidth;x++){
        const int addr0=x+(y+0)*iWidth;
        const int addr1=x+(y+1)*iWidth;
        const int addr2=x+(y+2)*iWidth;

        sum0+=(getValue(img,x*INTEGRAL_SCALE,(y+0)*INTEGRAL_SCALE)); /* S(x,y) */
        sum1+=(getValue(img,x*INTEGRAL_SCALE,(y+1)*INTEGRAL_SCALE)); /* S(x,y) */
        sum2+=(getValue(img,x*INTEGRAL_SCALE,(y+2)*INTEGRAL_SCALE)); /* S(x,y) */

        const int tmpImg = integralImg[addr0-iWidth];
        const int tmp1 = sum0+sum1;
        const int tmp2 = tmp1+sum2;

        integralImg[addr0]=tmpImg+sum0;
        integralImg[addr1]=tmpImg+tmp1;
        integralImg[addr2]=tmpImg+tmp2;
    }


}
