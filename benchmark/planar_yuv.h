#ifndef __PLANAR_YUV_H__
#define __PLANAR_YUV_H__

#include <stdint.h>
#include <x86intrin.h>

struct _PlanarYUVState {
    __m128i* yout;
    __m128i* yout_half;
    __m128i* uout_half;
    __m128i* uout_full;
    __m128i* vout_half;
    __m128i* vout_full;
};

typedef struct _PlanarYUVState PlanarYUVState;

void PlanarYUV_Init(PlanarYUVState** state);
void PlanarYUV_SetImage(PlanarYUVState* state, const uint8_t * const img);

inline uint8_t PlanarYUV_getY(PlanarYUVState* state, int32_t x, int32_t y) {
    const uint8_t* img = (uint8_t*)state->yout;
    return img[x + y * 640];
}

inline uint8_t PlanarYUV_getCb(PlanarYUVState* state, int32_t x, int32_t y) {
    const uint8_t* img = (uint8_t*)state->uout_full;
    return img[x + y * 640];
}

inline uint8_t PlanarYUV_getCr(PlanarYUVState* state, int32_t x, int32_t y) {
    const uint8_t* img = (uint8_t*)state->vout_full;
    return img[x + y * 640];
}

#endif
