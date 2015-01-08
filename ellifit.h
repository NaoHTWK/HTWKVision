#ifndef __ELLIFIT_H__
#define __ELLIFIT_H__

#include <vector>

#include <point_2d.h>

namespace htwk {

void elli_init();
bool fit(std::vector<point_2d> points,float *result) __attribute__((nonnull));
int choldc(float a[][7], int n, float l[][7]);
int inverse(float TB[][7], float InvB[][7], int N);

}  // namespace htwk

#endif  // __ELLIFIT_H__
