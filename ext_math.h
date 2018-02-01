#ifndef __EXT_MATH_H__
#define __EXT_MATH_H__

#include <algorithm>
#include <vector>

namespace ext_math {

template <typename T> T clamp(T min_val, T val, T max_val) {
    return std::max(std::min(max_val, val), min_val);
}

template <typename T> int sgn(T val) {
    return (val > T(0)) - (val < T(0));
}

union floatint{
    float f;
    int i;
};

inline float invSqrt(float x){
    floatint fi;
    fi.f=x;
    float xhalf = 0.5f*fi.f;
    fi.i = 0x5f375a86-(fi.i>>1); // gives initial guess y0
    fi.f = fi.f*(1.5f-xhalf*fi.f*fi.f); // Newton step, repeating increases accuracy
    return fi.f;
}

template <typename T> T medianOfThree(T a, T b, T c){
    return a > b ? a < c ? a : b < c ? c : b : b < c ? b : a < c ? c : a;
}

template <typename T> T medianOfFive(T a, T b, T c, T d, T e)
{
        return b < a ? d < c ? b < d ? a < e ? a < d ? e < d ? e : d
                                                                                                 : c < a ? c : a
                                                                                 : e < d ? a < d ? a : d
                                                                                                 : c < e ? c : e
                                                                 : c < e ? b < c ? a < c ? a : c
                                                                                                 : e < b ? e : b
                                                                                 : b < e ? a < e ? a : e
                                                                                                 : c < b ? c : b
                                                 : b < c ? a < e ? a < c ? e < c ? e : c
                                                                                                 : d < a ? d : a
                                                                                 : e < c ? a < c ? a : c
                                                                                                 : d < e ? d : e
                                                                 : d < e ? b < d ? a < d ? a : d
                                                                                                 : e < b ? e : b
                                                                                 : b < e ? a < e ? a : e
                                                                                                 : d < b ? d : b
                                 : d < c ? a < d ? b < e ? b < d ? e < d ? e : d
                                                                                                 : c < b ? c : b
                                                                                 : e < d ? b < d ? b : d
                                                                                                 : c < e ? c : e
                                                                 : c < e ? a < c ? b < c ? b : c
                                                                                                 : e < a ? e : a
                                                                                 : a < e ? b < e ? b : e
                                                                                                 : c < a ? c : a
                                                 : a < c ? b < e ? b < c ? e < c ? e : c
                                                                                                 : d < b ? d : b
                                                                                 : e < c ? b < c ? b : c
                                                                                                 : d < e ? d : e
                                                                 : d < e ? a < d ? b < d ? b : d
                                                                                                 : e < a ? e : a
                                                                                 : a < e ? b < e ? b : e
                                                                                                 : d < a ? d : a;
}


using vec2df = std::vector<std::vector<float> >;

static inline vec2df operator+(const vec2df &v1,const vec2df &v2){
    vec2df ret(v1.size());
    for(size_t r=0;r<v1.size();r++){
        ret[r]=std::vector<float>(v1[r].size());
        for(size_t c=0;c<v1[r].size();c++){
            ret[r][c]=v1[r][c]+v2[r][c];
        }
    }
    return ret;
}

static inline vec2df operator-(const vec2df &v1,const vec2df &v2){
    vec2df ret(v1.size());
    for(size_t r=0;r<v1.size();r++){
        ret[r]=std::vector<float>(v1[r].size());
        for(size_t c=0;c<v1[r].size();c++){
            ret[r][c]=v1[r][c]-v2[r][c];
        }
    }
    return ret;
}

static inline vec2df operator*(const vec2df &v1,const vec2df &v2){
    vec2df ret(v1.size());
    for(size_t r=0;r<v1.size();r++){
        ret[r]=std::vector<float>(v1[r].size());
        for(size_t c=0;c<v1[r].size();c++){
            ret[r][c]=v1[r][c]*v2[r][c];
        }
    }
    return ret;
}

static inline vec2df operator/(const vec2df &v1,const vec2df &v2){
    vec2df ret(v1.size());
    for(size_t r=0;r<v1.size();r++){
        ret[r]=std::vector<float>(v1[r].size());
        for(size_t c=0;c<v1[r].size();c++){
            ret[r][c]=v1[r][c]/v2[r][c];
        }
    }
    return ret;
}

static inline vec2df createVec2df(int r,int c){
    vec2df v(r);
    for(int i=0;i<r;i++)
        v[i]=std::vector<float>(c);
    return v;
}

template <typename T> int fm(T arr[], int b, int n) {
    int f = b;
    int c;
    for(c = b + 1; c < n; c++)
        if(arr[c] < arr[f])
            f = c;
    return f;
}

template <typename T> void isort(T arr[], int n) {
    int s, w;
    T sm;
    for(s = 0; s < n - 1; s++) {
        w = fm(arr, s, n);
        sm = arr[w];
        arr[w] = arr[s];
        arr[s] = sm;
    }
}

}  // namespace ext_math

#endif  // __EXT_MATH_H__

