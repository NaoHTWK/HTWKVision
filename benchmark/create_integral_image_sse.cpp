#include "create_integral_image_sse.h"

#include "create_integral_image_config.h"

#include <stdlib.h>

#include <x86intrin.h>

const int RESULT_WIDTH  = IMAGE_WIDTH /INTEGRAL_SCALE;
const int RESULT_HEIGHT = IMAGE_HEIGHT/INTEGRAL_SCALE;

const int RESULT_WIDTH_IN_M128 = RESULT_WIDTH/16 /* 128 bit */;
const int RESULT_HEIGHT_IN_M128 = RESULT_HEIGHT/16;

const int BYTES_PER_ITERATION=16 /* 128 bit operations */ * 4 /* four at a time */;
const int BYTES_PER_LINE = BYTES_PER_PIXEL*IMAGE_WIDTH;

__m128i* iimg;

void createIntegralImageSSEInit()
{
    int rc0 = posix_memalign((void**)&iimg, 16, RESULT_WIDTH_IN_M128*RESULT_HEIGHT*sizeof(*iimg));
}

/* Shuffle so we have the first 4 int32 */
static const char cshuffleInt0To3[]  ={0, -1,-1,-1, 1,-1,-1,-1, 2,-1,-1,-1, 3,-1,-1,-1};
static const char cshuffleInt4To7[]  ={5, -1,-1,-1, 6,-1,-1,-1, 7,-1,-1,-1, 8,-1,-1,-1};
static const char cshuffleInt81To11[]={9, -1,-1,-1,10,-1,-1,-1,11,-1,-1,-1,12,-1,-1,-1};
static const char cshuffleInt12To15[]={12,-1,-1,-1,13,-1,-1,-1,14,-1,-1,-1,15,-1,-1,-1};

static const __m128i shuffleInt0To3  =_mm_loadu_si128((__m128i*)cshuffleInt0To3);
static const __m128i shuffleInt4To7  =_mm_loadu_si128((__m128i*)cshuffleInt4To7);
static const __m128i shuffleInt81To11=_mm_loadu_si128((__m128i*)cshuffleInt81To11);
static const __m128i shuffleInt12To15=_mm_loadu_si128((__m128i*)cshuffleInt12To15);

#define CONV_U8_U32(src, d1, d2, d3, d4) \
    (d1) = _mm_shuffle_epi8((src), shuffleInt0To3); \
    (d2) = _mm_shuffle_epi8((src), shuffleInt4To7); \
    (d3) = _mm_shuffle_epi8((src), shuffleInt81To11); \
    (d4) = _mm_shuffle_epi8((src), shuffleInt12To15);

__m128i addU32(__m128i a, __m128i& acc)
{
    __m128i tmp1 = _mm_slli_si128(a, 32);
    __m128i tmp2 = _mm_slli_si128(a, 64);
    __m128i tmp3 = _mm_slli_si128(a, 96);

    __m128i sum = _mm_add_epi32(acc, a);
    sum = _mm_add_epi32(sum, tmp1);
    sum = _mm_add_epi32(sum, tmp2);
    sum = _mm_add_epi32(sum, tmp3);

    acc = _mm_srli_si128(a, 96);
    return sum;
}

void createIntegralImageSSE(uint8_t *img)
{
    for(int x = 0; x < RESULT_WIDTH_IN_M128; x++)
    {
        __m128i acc = _mm_load_si128((const __m128i*)img+BYTES_PER_ITERATION);

        __m128i tmp = _mm_shuffle_epi8(acc, shuffleInt0To3);
    }
}
