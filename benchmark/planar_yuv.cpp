#include "planar_yuv.h"

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <x86intrin.h>

#include <chrono>
#include <functional>
#include <iostream>

using namespace std;
using namespace std::chrono;

const int IMAGE_SCALE = 1;
const int IMAGE_WIDTH  = 640;
const int IMAGE_HEIGHT = 480;

const int RESULT_WIDTH  = IMAGE_WIDTH/IMAGE_SCALE;
const int RESULT_WIDTH_HALF = IMAGE_WIDTH/2/IMAGE_SCALE;

const int RESULT_HEIGHT = IMAGE_HEIGHT/IMAGE_SCALE;
const int RESULT_HEIGHT_HALF = IMAGE_HEIGHT/2/IMAGE_SCALE;

const int RESULT_WIDTH_IN_M128 = RESULT_WIDTH/16 /* 128 bit */;
const int RESULT_WIDTH_HALF_IN_M128 = RESULT_WIDTH_HALF/16 /* 128 bit */;
const int RESULT_HEIGHT_IN_M128 = RESULT_HEIGHT/16;

const int BYTES_PER_PIXEL = 2;
const int BYTES_PER_ITERATION=16 /* 128 bit operations */ * 4 /* four at a time */;
const int BYTES_PER_LINE = BYTES_PER_PIXEL*IMAGE_WIDTH;

/*
 * We compose one y m128i from two. We have or the result together */
static const char yout_half_cshuffle_part1[]={ 0, 2, 4, 6,  8,10,12,14, -1,-1,-1,-1, -1,-1,-1,-1};
static const char yout_half_cshuffle_part2[]={-1,-1,-1,-1, -1,-1,-1,-1,  0, 2, 4, 6,  8,10,12,14};
static const __m128i yout_half_shuffle_part1=_mm_loadu_si128((__m128i*)yout_half_cshuffle_part1);
static const __m128i yout_half_shuffle_part2=_mm_loadu_si128((__m128i*)yout_half_cshuffle_part2);

/* Cb:
 * Before:                                                                                                        ------2x Carry over ---
 *                                                                                                                |       |             |
 *                                                                                                                |       |             v
 *  1   x   3       5   x   3        5  xx  3      5  xx  3        5  xx  3     5    xx   3     5    xx   3       5  xx   3             xx
 * Y0 Cb0  Y1 Cr0  Y2 Cb1  Y3 Cr1 | Y4 Cb2 Y5 Cr2 Y6 Cb3 Y7 Cr3 | Y8 Cb4 Y9 Cr4 Y10 Cb5 Y11 Cr5 Y12 Cb6 Y13 Cr6 Y14 Cb7 Y15 Cr7 | Y16 [Cb8]
 *  0   1   2   3   4   5   6   7 |  8   9 10  11 12  13 14  15 | 16  17 18  19  20  21  22  23  24  25  26  27  28  29  30  31 |  32  33
 */
static const char uout_full_part1_cshuffle[]={1,1,2,2,3,3,4,4,5,5,6,6,7,7,8,8};
static const char uout_full_part2_cshuffle[]={9,9,10,10,11,11,12,12,13,13,14,14,15,15,-1,-1};
static const char uout_full_carry_over_cshuffle[]={-1,-1,-1,-1, -1,-1,-1,-1, -1,-1,-1,-1, -1,-1,0,0};
static const char uout_full_end_cshuffle[]={9,9,10,10,11,11,12,12,13,13,14,14,15,15,15,15};
static const char uout_full_begin_cshuffle[]={0,1,2,2,3,3,4,4,5,5,6,6,7,7,8,8};
static const __m128i uout_full_shuffle_part1=_mm_loadu_si128((__m128i*)uout_full_part1_cshuffle);
static const __m128i uout_full_shuffle_part2=_mm_loadu_si128((__m128i*)uout_full_part2_cshuffle);
static const __m128i uout_full_carry_over_shuffle=_mm_loadu_si128((__m128i*)uout_full_carry_over_cshuffle);
static const __m128i uout_full_end_shuffle=_mm_loadu_si128((__m128i*)uout_full_end_cshuffle);
static const __m128i uout_full_begin_shuffle=_mm_loadu_si128((__m128i*)uout_full_begin_cshuffle);

/* Shuffle so the start matches to the indexes to lutCb (one is missing and everything is shifted forward) */
static const char uout_half_start_cshuffle[]={0,2,3,4,5,6,7,8,9,10,11,12,13,14,15,-1};
/* Shuffle so the end matches to the indexes to lutCb (duplication of last element) */
static const char uout_half_end_cshuffle[]={1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,15};
static const __m128i uout_half_start_shuffle=_mm_loadu_si128((__m128i*)uout_half_start_cshuffle);
static const __m128i uout_half_end_shuffle=_mm_loadu_si128((__m128i*)uout_half_end_cshuffle);


/* Cr:
 * Before:                                                |---- Carry over ---|
 *                                                        |                   v
 *  3       5   x   3       5  xx    3      5  xx  3      5  xx    3      5   xx
 * Y0 Cb0  Y1 Cr0  Y2 Cb1  Y3 Cr1 | Y4 Cb2 Y5 Cr2 Y6 Cb3 Y7 Cr3 | Y8 Cb4 Y9 [Cr4]
 *  0   1   2   3   4   5   6   7 |  8   9 10  11 12  13 14  15 | 16  17 18   19
 *
 * Cr0 Cr1 Cr1 Cr2 Cr2 Cr3 Cr3 Cr4 | Cr4 Cr5 ...
 * Shuffle stuff according to lubCr
 */
static const char vout_full_part1_cshuffle[]={0,1,1,2,2,3,3,4,4,5,5,6,6,7,7,8};
static const char vout_full_part2_cshuffle[]={8,9,9,10,10,11,11,12,12,13,13,14,14,15,15,-1};
static const char vout_end_cshuffle[]={8,9,9,10,10,11,11,12,12,13,13,14,14,15,15,15};
static const __m128i vout_full_shuffle_part1=_mm_loadu_si128((__m128i*)vout_full_part1_cshuffle);
static const __m128i vout_full_shuffle_part2=_mm_loadu_si128((__m128i*)vout_full_part2_cshuffle);
static const __m128i vout_end_shuffle=_mm_loadu_si128((__m128i*)vout_end_cshuffle);

/*
 * See: https://software.intel.com/en-us/node/503876 YCbCr 422
 * Before:
 * Y0 Cb0  Y1 Cr0  Y2 Cb1  Y3 Cr1 Y4 Cb2 Y5 Cr2 Y6 Cb3 Y7 Cr3
 *  0   1   2   3   4   5   6   7  8   9 10  11 12  13 14  15
 *
 * After:
 * Y0 Y1 Y2 Y3 Y4 Y5  Y6  Y7 Cb0 Cb1 Cb2 Cb3 Cr0 Cr1 Cr2 Cr3
 *  0  2  4  6  8 10  12  14   1   5   9  13   3   7  11  15
 */
static const unsigned char cshuffle[]={0,2,4,6,8,10,12,14,1,5,9,13,3,7,11,15};
static const __m128i shuffle=_mm_loadu_si128((__m128i*)cshuffle);

static void print(__m128 x) {
    uint8_t* tt = (uint8_t*)&x;
    for(int i = 0; i < 16; i++) {
        printf("%3d ", tt[i]);
    }
    printf("\n");
}

template<bool isEven, bool createFullY, bool createHalfY, bool createCb, bool createCr>
static void createPlanarYUV(PlanarYUVState* state, const uint8_t * const img, const int x, const int y, __m128i& cb, __m128i& cr)
{
    /* Result:
     *  LSB Y00 Y01 Y02 Y03 Y04 Y05 Y06 Y07 Cb00 Cb01 Cb02 Cb03 Cr00 Cr01 Cr02 Cr03
     *  LSB Y08 Y09 Y10 Y11 Y12 Y13 Y14 Y15 Cb04 Cb05 Cb06 Cb07 Cr04 Cr05 Cr06 Cr07
     *  LSB Y16 Y17 Y18 Y19 Y20 Y21 Y22 Y23 Cb08 Cb09 Cb10 Cb11 Cr08 Cr09 Cr10 Cr11
     *  LSB Y24 Y25 Y26 Y27 Y28 Y29 Y30 Y32 Cb12 Cb13 Cb14 Cb15 Cr12 Cr13 Cr14 Cr15
     */

    __m128i *in=(__m128i*)(img+y*IMAGE_SCALE*BYTES_PER_LINE+x*BYTES_PER_ITERATION);

    __m128i im0=_mm_shuffle_epi8(_mm_load_si128(&(in[0])),shuffle);
    __m128i im1=_mm_shuffle_epi8(_mm_load_si128(&(in[1])),shuffle);
    __m128i im2=_mm_shuffle_epi8(_mm_load_si128(&(in[2])),shuffle);
    __m128i im3=_mm_shuffle_epi8(_mm_load_si128(&(in[3])),shuffle);

    if(createFullY || createHalfY) {
        /* Result:
         *  LSB Y00 Y01 Y02 Y03 Y04 Y05 Y06 Y07 Y08 Y09 Y10 Y11 Y12 Y13 Y14 Y15
         *  LSB Y16 Y17 Y18 Y19 Y20 Y21 Y22 Y23 Y24 Y25 Y26 Y27 Y28 Y29 Y30 Y32
         */
        __m128i yout0=_mm_unpacklo_epi64(im0,im1);
        __m128i yout1=_mm_unpacklo_epi64(im2,im3);

        if(createFullY) {
            _mm_stream_si128(&(state->yout[y*RESULT_WIDTH_IN_M128+x*2+1]), yout1);
            _mm_stream_si128(&(state->yout[y*RESULT_WIDTH_IN_M128+x*2]),   yout0);
        }

        if(createHalfY && isEven) {
            __m128i yout_half_part1=_mm_shuffle_epi8(yout0, yout_half_shuffle_part1);
            __m128i yout_half_part2=_mm_shuffle_epi8(yout1, yout_half_shuffle_part2);
            _mm_stream_si128(&(state->yout_half[y/2*RESULT_WIDTH_HALF_IN_M128+x]), _mm_or_si128(yout_half_part1, yout_half_part2));
        }
    }

    if(createCb || createCr) {
        /* Result:
         *  LSB Cb00 Cb01 Cb02 Cb03 Cr00 Cr01 Cr02 Cr03 Cb04 Cb05 Cb06 Cb07 Cr04 Cr05 Cr06 Cr07
         *  LSB Cb08 Cb09 Cb10 Cb11 Cr08 Cr09 Cr10 Cr11 Cb12 Cb13 Cb14 Cb15 Cr12 Cr13 Cr14 Cr15
         */
        __m128i ic0=_mm_unpackhi_epi64(im0,im1);
        __m128i ic1=_mm_unpackhi_epi64(im2,im3);

        /* Result: _MM_SHUFFLE is MSB to LSB
         *  0 Cb00 Cb01 Cb02 Cb03
         *  1 Cr00 Cr01 Cr02 Cr03
         *  2 Cb04 Cb05 Cb06 Cb07
         *  3 Cr04 Cr05 Cr06 Cr07
         *
         *  LSB Cb00 Cb01 Cb02 Cb03 Cb04 Cb05 Cb06 Cb07 Cr00 Cr01 Cr02 Cr03 Cr04 Cr05 Cr06 Cr07
         *
         *  3 Cb08 Cb09 Cb10 Cb11
         *  2 Cr08 Cr09 Cr10 Cr11
         *  1 Cb12 Cb13 Cb14 Cb15
         *  0 Cr12 Cr13 Cr14 Cr15
         *
         *  LSB Cb08 Cb09 Cb10 Cb11 Cb12 Cb13 Cb14 Cb15 Cr08 Cr09 Cr10 Cr11 Cr12 Cr13 Cr14 Cr15
         */
        __m128i is0=_mm_shuffle_epi32(ic0,_MM_SHUFFLE(3,1,2,0));
        __m128i is1=_mm_shuffle_epi32(ic1,_MM_SHUFFLE(3,1,2,0));

        /* Result: Cb00 Cb01 Cb02 Cb03 Cb04 Cb05 Cb06 Cb07 Cb08 Cb09 Cb10 Cb11 Cb12 Cb13 Cb14 Cb15 */
        if(createCb) cb = _mm_unpacklo_epi64(is0,is1);

        /* Result: Cr00 Cr01 Cr02 Cr03 Cr04 Cr05 Cr06 Cr07 Cr08 Cr09 Cr10 Cr11 Cr12 Cr13 Cr14 Cr15 */
        if(createCr) cr = _mm_unpackhi_epi64(is0,is1);
    }
}

inline void createCrElements(PlanarYUVState* state, const __m128i crValue, __m128i& crCarryOver, const int pos) {

    /* Prepare the second part. The last element is 0 for the carry over */
    __m128i crTmp = _mm_shuffle_epi8(crValue, vout_full_shuffle_part2);

    /* Add the carry over.
     * Before:  crTmp:     LSB: Cr8 Cr9 Cr9 Cr10 Cr10 Cr11 Cr11 Cr12 Cr12 Cr13 Cr13 Cr14 Cr14 Cr15 Cr15 0
     *          carryOver: LSB:   0   0   0    0    0    0    0    0    0    0    0    0    0    0    0 Cr0
     * After:              LSB: Cr8 Cr9 Cr9 Cr10 Cr10 Cr11 Cr11 Cr12 Cr12 Cr13 Cr13 Cr14 Cr14 Cr15 Cr15 Cr0
     */
    crTmp = _mm_or_si128(crTmp, crCarryOver);

    _mm_stream_si128(&(state->vout_full[pos+1]), crTmp);
    _mm_stream_si128(&(state->vout_full[pos]), _mm_shuffle_epi8(crValue, vout_full_shuffle_part1));

    /*****************************************************************************************************************************/
    /* Shift the first element to the most right to use it as carry over and add it later.
     * Before: LSB: Cr0 Cr1 Cr2 Cr3 Cr4 Cr5 Cr6 Cr6 Cr7 Cr8 Cr9 Cr10 Cr11 Cr12 Cr13 Cr14 Cr15
     * After:  LSB:   0   0   0   0   0   0   0   0   0   0   0    0    0    0    0    0 Cr0
      */
    crCarryOver = _mm_slli_si128(crValue, 15);
}

inline void createCbElements(PlanarYUVState* state, const __m128i cbValue, __m128i& cbCarryOver, const int pos) {

    /* Prepare the second part. The last two elements are 0 for the carry over */
    __m128i cbTmp = _mm_shuffle_epi8(cbValue, uout_full_shuffle_part2);

    /* Add the carry over.
     * Before:  CbTmp:     LSB: Cb8 Cb8 Cb9 Cb9 Cb10 Cb10 Cb11 Cb11 | Cb12 Cb12 Cb13 Cb13 Cb14 Cb14    0   0
     *          carryOver: LSB:   0   0   0   0    0    0    0    0 |    0    0    0    0    0    0  Cb0 Cb0
     * After:              LSB: Cb8 Cb8 Cb9 Cb9 Cb10 Cb10 Cb11 Cb11 | Cb12 Cb12 Cb13 Cb13 Cb14 Cb14  Cb0 Cb0
     */
    cbTmp = _mm_or_si128(cbTmp, cbCarryOver);

    _mm_stream_si128(&(state->uout_full[pos+1]), cbTmp);
    _mm_stream_si128(&(state->uout_full[pos]), _mm_shuffle_epi8(cbValue, uout_full_shuffle_part1));

    /* carryOver: LSB: 0 0 0 0 0 0 0 0 | 0 0 0 0 0 0 Cb0 Cb0 */
    cbCarryOver = _mm_shuffle_epi8(cbValue, uout_full_carry_over_shuffle);
}

inline void createHalfCbElement(PlanarYUVState* state, const __m128i cbValue, const __m128i cbCarryOver, const int pos)
{
    /* carryOver: LSB: 0 0 0 0 0 0 0 0 | 0 0 0 0 0 0 Cb0 Cb0
       carryOver: LSB: 0 0 0 0 0 0 0 0 | 0 0 0 0 0 0   0 Cb0 */
    __m128i cbCarryOverHalf = _mm_slli_si128(cbCarryOver, 1);


    /* Before:  tmp: LSB: Cb1 Cb2 Cb3 Cb4 Cb5 Cb6 Cb6 Cb7 Cb8 Cb9 Cb10 Cb11 Cb12 Cb13 Cb14 Cb15 0
     *          co:  LSB:   0   0   0   0   0   0   0   0   0   0    0    0    0    0    0    0 Cb0
     * After:        LSB: Cb1 Cb2 Cb3 Cb4 Cb5 Cb6 Cb6 Cb7 Cb8 Cb9 Cb10 Cb11 Cb12 Cb13 Cb14 Cb15 Cb0
     */
    __m128i tmp = _mm_srli_si128(cbValue, 1);
    tmp = _mm_or_si128(cbCarryOverHalf, tmp);

    _mm_stream_si128(&(state->uout_half[pos]), tmp);
}

template<bool isEven, bool createFullY, bool createHalfY, bool createFullCb, bool createHalfCb, bool createFullCr, bool createHalfCr>
static void createPlanarYuvLine(PlanarYUVState* state, const uint8_t * const img, const int y)
{
    __m128i cb0, cb1;
    __m128i cr0, cr1;

    /* In that line: process last two elements first. The last element need some extra shuffeling */
    createPlanarYUV<isEven, createFullY, createHalfY, createFullCb||createHalfCb, createFullCr||createHalfCr>(state, img, BYTES_PER_LINE/BYTES_PER_ITERATION-1, y, cb0, cr0);
    createPlanarYUV<isEven, createFullY, createHalfY, createFullCb||createHalfCb, createFullCr||createHalfCr>(state, img, BYTES_PER_LINE/BYTES_PER_ITERATION-2, y, cb1, cr1);

    /*****************************************************************************************************************************/

    __m128i cbCarryOver;
    if(createFullCb || createHalfCb) {

        if(createFullCb) {
            state->uout_full[y*RESULT_WIDTH_IN_M128+RESULT_WIDTH_IN_M128-2] = _mm_shuffle_epi8(cb0, uout_full_shuffle_part1);
            state->uout_full[y*RESULT_WIDTH_IN_M128+RESULT_WIDTH_IN_M128-1] = _mm_shuffle_epi8(cb0, uout_full_end_shuffle);
        }

        /* carryOver: LSB: 0 0 0 0 0 0 0 0 | 0 0 0 0 0 0 Cb0 Cb0 */
        cbCarryOver = _mm_shuffle_epi8(cb0, uout_full_carry_over_shuffle);

        if(createHalfCb && isEven) {
            cb0 = _mm_shuffle_epi8(cb0, uout_half_end_shuffle);
            state->uout_half[y/2*RESULT_WIDTH_HALF_IN_M128+RESULT_WIDTH_HALF_IN_M128-1] = cb0;
            createHalfCbElement(state, cb1, cbCarryOver, y/2*RESULT_WIDTH_HALF_IN_M128+RESULT_WIDTH_HALF_IN_M128-2);
        }

        createCbElements(state, cb1, cbCarryOver, y*RESULT_WIDTH_IN_M128+RESULT_WIDTH_IN_M128-4);
    }

    /*****************************************************************************************************************************/

    __m128i crCarryOver;
    if(createFullCr || createHalfCr) {

        if(createFullCr) {
            _mm_stream_si128(&(state->vout_full[y*RESULT_WIDTH_IN_M128+RESULT_WIDTH_IN_M128-2]), _mm_shuffle_epi8(cr0, vout_full_shuffle_part1));
            _mm_stream_si128(&(state->vout_full[y*RESULT_WIDTH_IN_M128+RESULT_WIDTH_IN_M128-1]), _mm_shuffle_epi8(cr0, vout_end_shuffle));
        }

        if(createHalfCr && isEven) {
            _mm_stream_si128(&(state->vout_half[y/2*RESULT_WIDTH_HALF_IN_M128+RESULT_WIDTH_HALF_IN_M128-1]), cr0);
            _mm_stream_si128(&(state->vout_half[y/2*RESULT_WIDTH_HALF_IN_M128+RESULT_WIDTH_HALF_IN_M128-2]), cr1);
        }

        /* Shift the first element to the most right to use it as carry over and add it later.
         * Before: LSB: Cr0 Cr1 Cr2 Cr3 Cr4 Cr5 Cr6 Cr6 Cr7 Cr8 Cr9 Cr10 Cr11 Cr12 Cr13 Cr14 Cr15
         * After:  LSB:   0   0   0   0   0   0   0   0   0   0   0    0    0    0    0    0 Cr0
          */
        crCarryOver = _mm_slli_si128(cr0, 15);
        createCrElements(state, cr1, crCarryOver, y*RESULT_WIDTH_IN_M128+RESULT_WIDTH_IN_M128-4);
    }

    for(int x = BYTES_PER_LINE/BYTES_PER_ITERATION-3; x >= 1;x--) {
        createPlanarYUV<isEven, createFullY, createHalfY, createFullCb||createHalfCb, createFullCr||createHalfCr>(state, img, x, y, cb0, cr0);

        if(createFullCb || createHalfCb) {
            if(createHalfCb && isEven) createHalfCbElement(state, cb0, cbCarryOver, y/2*RESULT_WIDTH_HALF_IN_M128+x);
            createCbElements(state, cb0, cbCarryOver, y*RESULT_WIDTH_IN_M128+x*2);
        }

        if(createFullCr || createHalfCr) {
            createCrElements(state, cr0, crCarryOver, y*RESULT_WIDTH_IN_M128+x*2);
            if(createHalfCr && isEven) _mm_stream_si128(&(state->vout_half[y/2*RESULT_WIDTH_HALF_IN_M128+x]), cr0);
        }
    }

    /* This is the first element which has a special ordering. */
    const int x = 0;
    createPlanarYUV<isEven, createFullY, createHalfY, createFullCb||createHalfCb, createFullCr||createHalfCr>(state, img, x, y, cb0, cr0);

    if(createFullCb || createHalfCb) {
        if(createHalfCb && isEven) {
            /* See uout_start_cshuffle for ordering */
            cb1 = _mm_shuffle_epi8(cb0, uout_half_start_shuffle);
            /* Add carry over from tmp. There is no new carry over. */
            state->uout_half[y/2*RESULT_WIDTH_HALF_IN_M128+x] = _mm_or_si128(cb1, _mm_slli_si128(cbCarryOver, 1));
        }

        if(createFullCb) {
            /* Create first Cb element */
            __m128i cbTmp = _mm_shuffle_epi8(cb0, uout_full_shuffle_part2);
            _mm_stream_si128(&(state->uout_full[y*RESULT_WIDTH_IN_M128+1]), _mm_or_si128(cbTmp, cbCarryOver));
            _mm_stream_si128(&(state->uout_full[y*RESULT_WIDTH_IN_M128+0]), _mm_shuffle_epi8(cb0, uout_full_begin_shuffle));
        }
    }

    if(createFullCr || createHalfCr) {
        /* Last Cr Element */
        if(createFullCr) createCrElements(state, cr0, crCarryOver, y*RESULT_WIDTH_IN_M128+x*2);
        if(createHalfCr && isEven) _mm_stream_si128(&(state->vout_half[y/2*RESULT_WIDTH_HALF_IN_M128+x]), cr0);
    }
}

void PlanarYUV_SetImage(PlanarYUVState* s, const uint8_t * const img)
{
    /* We process the field from the end to the top. We have to carry over one Cb value to the elements before */
    for(int y = RESULT_HEIGHT-1; y >= 0; y-=2) {
        createPlanarYuvLine<false, false, false, false, false, true, false>(s, img, y);
        createPlanarYuvLine<true, false, false, false, false, true, false>(s, img, y-1);
    }
}

void PlanarYUV_Init(PlanarYUVState** state)
{
    _PlanarYUVState* s = new _PlanarYUVState;

    int rc0 = posix_memalign((void**)&(s->yout), 16, RESULT_WIDTH_IN_M128*RESULT_HEIGHT*sizeof(*(s->yout)));
    int rc1 = posix_memalign((void**)&(s->yout_half), 16, RESULT_WIDTH_HALF*RESULT_HEIGHT_HALF*sizeof(*(s->yout_half)));

    int rc2 = posix_memalign((void**)&(s->uout_half), 16, RESULT_WIDTH_HALF*RESULT_HEIGHT_HALF*sizeof(*(s->uout_half)));
    int rc3 = posix_memalign((void**)&(s->uout_full), 16, RESULT_WIDTH_IN_M128*RESULT_HEIGHT*sizeof(*(s->uout_full)));

    int rc4 = posix_memalign((void**)&(s->vout_half), 16, RESULT_WIDTH_HALF*RESULT_HEIGHT_HALF*sizeof(*(s->vout_half)));
    int rc5 = posix_memalign((void**)&(s->vout_full), 16, RESULT_WIDTH_IN_M128*RESULT_HEIGHT*sizeof(*(s->vout_full)));

    if((rc0|rc1|rc2|rc3|rc4|rc5) != 0) {
        printf("Error allocating memory for SSE! %s %d\n", __FILE__, __LINE__);
        exit(1);
    }

    *state = s;
}
