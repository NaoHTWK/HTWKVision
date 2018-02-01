#include "integral_image.h"

#include <cstdlib>
#include <iostream>
#include <emmintrin.h>
#include <ext_math.h>

namespace htwk {

const int IntegralImage::SRC_IMAGE_WIDTH=640;
const int IntegralImage::SRC_IMAGE_HEIGHT=480;

const int IntegralImage::iWidth  = IntegralImage::SRC_IMAGE_WIDTH/IntegralImage::INTEGRAL_SCALE;
const int IntegralImage::iHeight = IntegralImage::SRC_IMAGE_HEIGHT/IntegralImage::INTEGRAL_SCALE;

IntegralImage::IntegralImage(int width, int height, int8_t *lutCb, int8_t *lutCr)
: BaseDetector(width, height, lutCb, lutCr)
{
    if(posix_memalign((void**)&integralImg, 16, iWidth*iHeight*sizeof(int)) != 0) {
        printf("Error: Couldn't allocate aligned memory for integral image of the ball detector. %s %d\n", __FILE__, __LINE__);
        fflush(stdout);
        exit(1);
    }
}

IntegralImage::~IntegralImage()
{
    free(integralImg);
}


#define getValue(img, x, y) (getY((img), (x), (y)) + 3*getCr((img), (x), (y)))
//#define getValue(img, x, y) getCr((img), (x), (y))

void IntegralImage::proceed(uint8_t *img) const {
    integralImg[0]=getValue(img,0,0);
    for(int x=1;x<iWidth;x++){
        integralImg[x]=integralImg[x-1]+(getValue(img,x*INTEGRAL_SCALE,0));
    }


    if (INTEGRAL_SCALE == 2) {
        const int factor = 4;
        __m128i y_mask = _mm_set1_epi32(0xff);
        __m128i v_mask = _mm_set1_epi32(0xff000000);
        __m128i fff0 = _mm_setr_epi32(-1, -1, -1, 0);
        for(int y=1;y<iHeight;y++){
            __m128i sum=_mm_set1_epi32(0);
            for(int x=0;x<iWidth/factor;x++){
                const int addr0=x*factor+y*iWidth;

                const __m128i i0 = _mm_load_si128((__m128i*)&(integralImg[addr0-iWidth]));
                // yuyv yuyv yuyv yuyv
                // x  x x  x x  x x  x
                // mask v
                //    x    x    x    x
                // epi32 v2=(v>>23 (epi32+) v>>24)
                // mask y
                // x    x    x    x
                // v2 (epi32+) y
                int xr = x * factor * INTEGRAL_SCALE;
                __m128i ycr = _mm_load_si128((__m128i*)&img[(xr + y*INTEGRAL_SCALE*width)<<1]);
                __m128i y = _mm_and_si128(ycr, y_mask);
                __m128i v = _mm_and_si128(ycr, v_mask);
                __m128i vs7 = _mm_srli_epi32(v, 23);
                __m128i vs8 = _mm_srli_epi32(v, 24);
                __m128i val = _mm_add_epi32(_mm_add_epi32(vs7, vs8), y);
                __m128i val012c = _mm_and_si128(val, fff0);
                __m128i valc000 = _mm_shuffle_epi32(val012c, _MM_SHUFFLE(0, 0, 0, 3));
                __m128i valcc11 = _mm_shuffle_epi32(val012c, _MM_SHUFFLE(1, 1, 3, 3));
                __m128i valccc2 = _mm_shuffle_epi32(val012c, _MM_SHUFFLE(2, 3, 3, 3));
                __m128i iSum0 = _mm_add_epi32(_mm_add_epi32(
                                                  _mm_add_epi32(val, sum),
                                                  _mm_add_epi32(valc000, valcc11)),
                                              valccc2);
                sum = _mm_shuffle_epi32(iSum0, _MM_SHUFFLE(3, 3, 3, 3));
                const __m128i r0 = _mm_add_epi32(i0, iSum0);
                _mm_stream_si128((__m128i*)&integralImg[addr0], r0);
            }
        }
    } else {
        const int factor = 16;
        for(int y=1;y<iHeight;y++){
            int sum=0;
            for(int x=0;x<iWidth/factor;x++){
                const int addr0=x*factor+y*iWidth+0;
                const int addr1=x*factor+y*iWidth+4;
                const int addr2=x*factor+y*iWidth+8;
                const int addr3=x*factor+y*iWidth+12;

                const __m128i i0 = _mm_load_si128((__m128i*)&(integralImg[addr0-iWidth]));
                const __m128i i1 = _mm_load_si128((__m128i*)&(integralImg[addr1-iWidth]));
                const __m128i i2 = _mm_load_si128((__m128i*)&(integralImg[addr2-iWidth]));
                const __m128i i3 = _mm_load_si128((__m128i*)&(integralImg[addr3-iWidth]));

                const int cr0  = sum  + getValue(img,(x*factor+ 0)*INTEGRAL_SCALE,y*INTEGRAL_SCALE);
                const int cr1  = cr0  + getValue(img,(x*factor+ 1)*INTEGRAL_SCALE,y*INTEGRAL_SCALE);
                const int cr2  = cr1  + getValue(img,(x*factor+ 2)*INTEGRAL_SCALE,y*INTEGRAL_SCALE);
                const int cr3  = cr2  + getValue(img,(x*factor+ 3)*INTEGRAL_SCALE,y*INTEGRAL_SCALE);
                const int cr4  = cr3  + getValue(img,(x*factor+ 4)*INTEGRAL_SCALE,y*INTEGRAL_SCALE);
                const int cr5  = cr4  + getValue(img,(x*factor+ 5)*INTEGRAL_SCALE,y*INTEGRAL_SCALE);
                const int cr6  = cr5  + getValue(img,(x*factor+ 6)*INTEGRAL_SCALE,y*INTEGRAL_SCALE);
                const int cr7  = cr6  + getValue(img,(x*factor+ 7)*INTEGRAL_SCALE,y*INTEGRAL_SCALE);
                const int cr8  = cr7  + getValue(img,(x*factor+ 8)*INTEGRAL_SCALE,y*INTEGRAL_SCALE);
                const int cr9  = cr8  + getValue(img,(x*factor+ 9)*INTEGRAL_SCALE,y*INTEGRAL_SCALE);
                const int cr10 = cr9  + getValue(img,(x*factor+10)*INTEGRAL_SCALE,y*INTEGRAL_SCALE);
                const int cr11 = cr10 + getValue(img,(x*factor+11)*INTEGRAL_SCALE,y*INTEGRAL_SCALE);
                const int cr12 = cr11 + getValue(img,(x*factor+12)*INTEGRAL_SCALE,y*INTEGRAL_SCALE);
                const int cr13 = cr12 + getValue(img,(x*factor+13)*INTEGRAL_SCALE,y*INTEGRAL_SCALE);
                const int cr14 = cr13 + getValue(img,(x*factor+14)*INTEGRAL_SCALE,y*INTEGRAL_SCALE);
                const int cr15 = cr14 + getValue(img,(x*factor+15)*INTEGRAL_SCALE,y*INTEGRAL_SCALE);
                sum = cr15;

                const __m128i iSum0 = _mm_setr_epi32( cr0,  cr1,  cr2,  cr3);
                const __m128i iSum1 = _mm_setr_epi32( cr4,  cr5,  cr6,  cr7);
                const __m128i iSum2 = _mm_setr_epi32( cr8,  cr9, cr10, cr11);
                const __m128i iSum3 = _mm_setr_epi32(cr12, cr13, cr14, cr15);

                const __m128i r0 = _mm_add_epi32(i0, iSum0);
                const __m128i r1 = _mm_add_epi32(i1, iSum1);
                const __m128i r2 = _mm_add_epi32(i2, iSum2);
                const __m128i r3 = _mm_add_epi32(i3, iSum3);

                _mm_stream_si128((__m128i*)&integralImg[addr0], r0);
                _mm_stream_si128((__m128i*)&integralImg[addr1], r1);
                _mm_stream_si128((__m128i*)&integralImg[addr2], r2);
                _mm_stream_si128((__m128i*)&integralImg[addr3], r3);
            }
        }
    }
}

}//namespace htwk
