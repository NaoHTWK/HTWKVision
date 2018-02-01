#ifndef __FAST_MATH_H__
#define __FAST_MATH_H__

#include <list>
#include <string>
#include <vector>

#include <cstdint>
#include <cmath>
#include <cstdio>

#undef M_PI
#define M_PI (3.14159265358979311600f)
#define M_PI2 (M_PI/2.f)
//LUT - LookUp Table
#define LUT_SIZE 4096

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

template <typename T> T clamp(T min_val, T val, T max_val) {
    return std::max(std::min(max_val, val), min_val);
}

template <typename T> T minmax(T min_val, T val, T max_val) {
    return clamp(min_val, val, max_val);
}

// clamps the values to [-PI, PI]
template <typename T> T normalizeRotation(T val) {
	while (val > M_PI )
		val -= M_PI * 2.f;
	while (val < -M_PI )
		val += M_PI * 2.f;
	return val;
}

template <typename T> float euclideanDistance(T dx, T dy) {
	return sqrtf(dx*dx+dy*dy);
}

union floatint{
	float f;
	int i;
};

void genAtan2f();
float linInterpol(float *lut,float i);
float atan2Fast(float y, float x);

inline float fast_fabsf(float f){
        floatint fi;
        fi.f=f;
        fi.i &= 0x7fffffff;
        return fi.f; // convert bits back to float
}

#define PI_FLOAT     3.14159265f
#define PIBY2_FLOAT  1.5707963f
// |error| < 0.005
inline float approx_atan2f(float y, float x) {
    if (x == 0.0f) {
        if (y > 0.0f) return PIBY2_FLOAT;
        if (y == 0.0f) return 0.0f;
        return -PIBY2_FLOAT;
    }
    float atan;
    float z = y/x;
    if (fast_fabsf(z) < 1.0f) {
        atan = z/(1.0f + 0.28f*z*z);
        if (x < 0.0f) {
            if (y < 0.0f) return atan - PI_FLOAT;
            return atan + PI_FLOAT;
        }
    } else {
        atan = PIBY2_FLOAT - z/(z*z + 0.28f);
        if (y < 0.0f) return atan - PI_FLOAT;
    }
    return atan;
}

inline float approx_sin(float x) {
    constexpr float B = 4.f/M_PI;
    constexpr float C = -4.f/(M_PI*M_PI);

    float y = B * x + C * x * std::abs(x);

    const float P = 0.225;
    return P * (y * std::abs(y) - y) + y;   // Q * y + P * y * abs(y)
}

inline float approx_cos(float x) {
    x += M_PI / 2.f;
    if (x > M_PI)
        x -= M_PI * 2.f;
    return approx_sin(x);
}

/* deprecated use boost:
 * #include <boost/random/mersenne_twister.hpp>
 * #include <boost/random/uniform_01.hpp>
 *
 * boost::random::mt19937 rng;
 * boost::random::uniform_01<> dist;
 *
 * return dist(rng);
 */
float frand();
void merge(int lo, int m, int hi,float *a,float *b);
void mergesort(int lo, int hi,float *a,float *b);
void sort(float *a,int n);
void isort(float arr[],int n);
void isort(int arr[],int n);
float nextGaussian();
float invSqrt(float x);

template <typename T> int sgn(T val) {
   return (val > T(0)) - (val < T(0));
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

template<typename T1,typename T2>
static float dist(const T1 &pos1,const T2 &pos2){
	return sqrtf((pos1.x-pos2.x)*(pos1.x-pos2.x)+(pos1.y-pos2.y)*(pos1.y-pos2.y));
}

template <class U> struct closer {
    const U& a;
    explicit closer(const U& a) : a(a) {}

    template<class T>
    bool operator() (const T& x, const T& y) const { return dist(x, a) < dist(y, a); }
};

template<class U, class T>
bool closer_to(const U& what, const T& c1, const T& c2) { return dist(c1, what) < dist(c2, what); };

#endif
