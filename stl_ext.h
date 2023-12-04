#pragma once

#include <chrono>
#include <cmath>
#include <cstdint>
#include <optional>
#include <thread>
#include <type_traits>
#include <vector>

#include <easy/profiler.h>

#ifndef M_PIf
#define M_PIf (static_cast<float>(M_PI))
#endif

/**
 * @brief Returns lower_val if p < lower_bound, upper_val if p > upper_bound, linear interpolation otherwise.
 * @attention Requires lower_bound <= upper_bound.
 * @example You have a speed (speed_close) at which the ball should be approached when you're close to the ball and
 *          another speed (speed_far) at which the ball should be approached when you're far away.
 *          Let's say that when you're closer than 0.5m speed_close should be used and when you're farther than 1m
 *          speed_far.
 *          If the ball is between 0.5-1m you want a smooth transition between the 2 speeds. You can now just run
 *          clamped_linear_interpolation(distance_to_ball, speed_close, speed_far, 0.5f, 1.f) and get exactly the speed
 *          you want.
 */
template <typename T>
T clamped_linear_interpolation(float p, const T& lower_val, const T& upper_val, float lower_bound, float upper_bound) {
    if (p < lower_bound)
        return lower_val;
    if (p > upper_bound)
        return upper_val;
    if (p == lower_bound && p == upper_bound)
        return (lower_val + upper_val) * .5f;
    T m = (upper_val - lower_val) / (upper_bound - lower_bound);
    return lower_val + (p - lower_bound) * m;
}

template <typename Callable, typename... Args>
[[nodiscard]] std::thread launch_named_thread(const std::string& name, bool low_prio, Callable&& callable, Args&&... args) {
    return std::thread([ callable, args...]() {
        // EASY_THREAD(name.c_str());
        // pthread_setname_np(pthread_self(), name.c_str());
        pthread_setschedprio(pthread_self(), sched_get_priority_min(sched_getscheduler(pthread_self())));
        callable(args...);
    });
}

template <typename Map>
std::optional<typename Map::mapped_type> find(Map& map, const typename Map::key_type& key) {
    auto it = map.find(key);
    if (it != map.end()) {
        return it->second;
    }
    return {};
}

template <typename Map>
std::optional<const typename Map::mapped_type> find(const Map& map, const typename Map::key_type& key) {
    auto it = map.find(key);
    if (it != map.end()) {
        return it->second;
    }
    return {};
}

template <typename Map>
std::optional<typename Map::mapped_type*> find_ptr(Map& map, const typename Map::key_type& key) {
    auto it = map.find(key);
    if (it != map.end()) {
        return &it->second;
    }
    return {};
}

template <typename C1, typename C2, typename F>
void combined_for_each(C1&& c1, C2&& c2, F f) {
    auto it = std::begin(c2);
    for (auto& e1 : c1) {
        f(e1, *it);
        it++;
    }
}

template <typename C1, typename C2, typename F>
void zip(C1&& c1, C2&& c2, F f) {  // alias for combined_for_each
    combined_for_each(std::forward<C1>(c1), std::forward<C2>(c2), std::forward<F>(f));
}

//! Returns the value or the boundaries [0..1]
inline float limit01(float f) {
    return std::min(1.f, std::max(0.f, f));
}

//! Returns the value or the boundaries [-1..1]
inline float limitpm1(float f) {
    return std::min(1.f, std::max(-1.f, f));
}

//! Returns true if val lies within [lower_bound..upper_bound)
template <typename T1, typename T2, typename T3>
bool within(T1 val, T2 lower_bound, T3 upper_bound) {
    return val >= lower_bound && val < upper_bound;
}

constexpr float operator"" _deg(long double deg) {
    return static_cast<float>(deg) * M_PIf / 180.f;
}

constexpr float operator"" _deg(unsigned long long int deg) {
    return deg * M_PIf / 180.f;
}

template <typename T1, typename T2, typename R>
R clamp(R value, T1 low, T2 high) {
    return std::min(std::max(low, value), high);
}

template <typename T>
int sgn(T v) {
    return v < 0 ? -1 : (v > 0 ? 1 : 0);
}

inline int64_t time_us() {
    using namespace std::chrono;
    return duration_cast<microseconds>(system_clock::now().time_since_epoch()).count();
}

//! Function converts seconds to us. It will NOT work with something else!
static constexpr int64_t operator"" _s(long double seconds) {
    return (int64_t)(seconds * 1'000'000ull);
}

static constexpr int64_t operator"" _s(unsigned long long seconds) {
    return (int64_t)(seconds * 1'000'000ull);
}

//! Function converts ms to us. It will NOT work with something else!
static constexpr int64_t operator"" _ms(long double seconds) {
    return (int64_t)(seconds * 1'000ull);
}

static constexpr int64_t operator"" _ms(unsigned long long seconds) {
    return (int64_t)(seconds * 1'000ull);
}

//! Function converts us to us. It will NOT work with something else!
static constexpr int64_t operator"" _us(long double seconds) {
    return (int64_t)(seconds * 1ull);
}

static constexpr int64_t operator"" _us(unsigned long long seconds) {
    return (int64_t)(seconds * 1ull);
}

//! clamps the values to [-PI, PI]
template <typename T>
T normalizeRotation(T val) {
    while (val > M_PI)
        val -= M_PI * 2.f;
    while (val < -M_PI)
        val += M_PI * 2.f;
    return val;
}

template <typename T>
T angleDiff(T angle1, T angle2) {
    return normalizeRotation(angle1 - angle2);
}

template <typename C>
typename C::value_type angleAvg(const C& c) {
    typename C::value_type cos_sum = 0;
    typename C::value_type sin_sum = 0;
    for (const typename C::value_type& v : c) {
        cos_sum += cos(v);
        sin_sum += sin(v);
    }
    return atan2(sin_sum / c.size(), cos_sum / c.size());
}

inline float invSqrt(float x) {
    union floatint {
        float f;
        int i;
    };

    floatint fi{.f = x};
    float xhalf = 0.5f * fi.f;
    fi.i = 0x5f375a86 - (fi.i >> 1);             // gives initial guess y0
    fi.f = fi.f * (1.5f - xhalf * fi.f * fi.f);  // Newton step, repeating increases accuracy
    return fi.f;
}

template <typename T>
T medianOfThree(T a, T b, T c) {
    return a > b ? a < c ? a : b < c ? c : b : b < c ? b : a < c ? c : a;
}

template <typename T>
T medianOfFive(T a, T b, T c, T d, T e) {
    return b < a ? d < c ? b < d ? a < e ? a < d ? e < d ? e : d : c < a ? c : a : e < d ? a < d ? a : d : c < e ? c : e
                                 : c < e ? b < c ? a < c ? a : c : e < b ? e : b : b < e ? a < e ? a : e : c < b ? c : b
                         : b < c ? a < e ? a < c ? e < c ? e : c : d < a ? d : a : e < c ? a < c ? a : c : d < e ? d : e
                                 : d < e ? b < d ? a < d ? a : d : e < b ? e : b : b < e ? a < e ? a : e : d < b ? d : b
                 : d < c ? a < d ? b < e ? b < d ? e < d ? e : d : c < b ? c : b : e < d ? b < d ? b : d : c < e ? c : e
                                 : c < e ? a < c ? b < c ? b : c : e < a ? e : a : a < e ? b < e ? b : e : c < a ? c : a
                         : a < c ? b < e ? b < c ? e < c ? e : c : d < b ? d : b : e < c ? b < c ? b : c : d < e ? d : e
                                 : d < e ? a < d ? b < d ? b : d : e < a ? e : a
                                         : a < e ? b < e ? b : e : d < a ? d : a;
}

template <typename Enumeration>
constexpr auto as_integer(Enumeration const value) {
    return static_cast<typename std::underlying_type<Enumeration>::type>(value);
}

template <typename F>
constexpr int round_int(F f) {
    return static_cast<int>(f >= static_cast<F>(0) ? f + static_cast<F>(0.5) : f - static_cast<F>(0.5));
}
