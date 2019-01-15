#ifndef POINT_2D_H
#define POINT_2D_H

#include <cfloat>
#include <cmath>
#include <tuple>
#include <fast_math.h>

namespace htwk {

struct point_2d {
    float x;
    float y;

    point_2d() = default;
    template <typename P>
    point_2d(const P& p) : x(p.x), y(p.y) {}
    constexpr point_2d(float x, float y) : x(x), y(y) {}

    point_2d& operator+=(const point_2d& rhs) {
        x += rhs.x;
        y += rhs.y;
        return *this;
    }

    friend point_2d operator+(point_2d lhs, const point_2d& rhs) {
        lhs += rhs;
        return lhs;
    }

    point_2d& operator-=(const point_2d& rhs) {
        x -= rhs.x;
        y -= rhs.y;
        return *this;
    }

    friend point_2d operator-(point_2d lhs, const point_2d& rhs) {
        lhs -= rhs;
        return lhs;
    }

    friend point_2d operator*(point_2d lhs, float rhs) {
        lhs.x *= rhs;
        lhs.y *= rhs;
        return lhs;
    }

    friend point_2d operator*(float lhs, const point_2d& rhs) {
        return rhs*lhs;
    }

    point_2d& operator/=(float rhs) {
        x /= rhs;
        y /= rhs;
        return *this;
    }

    friend point_2d operator/(point_2d lhs, float rhs) {
        lhs /= rhs;
        return lhs;
    }

    constexpr point_2d operator-() const {
        return point_2d(-x, -y);
    }

    inline float norm() const {
        return sqrtf(x*x+y*y);
    }

    inline float magnitude() const {
        return sqrtf(x*x+y*y);
    }

    inline float norm_sqr() const {
        return x*x+y*y;
    }

    inline point_2d mul_elem(const point_2d& b) const {
        return {x * b.x, y * b.y};
    }

    inline float dot(const point_2d& b) const {
        return x * b.x + y * b.y;
    }

    float angle_to(const point_2d& b) const {
      return std::acos(dot(b) / (norm() * b.norm()));
    }

    inline std::tuple<float,float> tuple() const {
        return std::make_tuple(x, y);
    }

    point_2d rotated(float angle) {
        return {x * cosf(angle) - y * sinf(angle), x * sinf(angle) + y * cosf(angle)};
    }

    point_2d rotated_approx(float angle) {
        float c = approx_cos(angle);
        float s = approx_sin(angle);
        return {x * c - y * s, x * s + y * c};
    }

    point_2d normalized() const {
        float n = norm();
        if (n == 0) return {0, 0};
        return {x / n, y / n};
    }

    point_2d normal() const {
        return {-y, x};
    }

    friend bool operator==(const point_2d& lhs, const point_2d& rhs) {
        return lhs.x == rhs.x && lhs.y == rhs.y;
    }

    friend bool operator!=(const point_2d& lhs, const point_2d& rhs) {
        return !(lhs == rhs);
    }
};

}  // namespace htwk

#endif // POINT_2D_H
