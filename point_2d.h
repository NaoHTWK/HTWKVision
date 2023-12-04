#pragma once

#include <stl_ext.h>

#include <cfloat>
#include <cmath>
#include <iostream>
#include <tuple>

class CamPose;

namespace htwk {

struct point_2d {
    float x = 0.f;
    float y = 0.f;

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
        return rhs * lhs;
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
        return {-x, -y};
    }

    operator std::tuple<float, float>() const {
        return std::make_tuple(x, y);
    }

    inline float norm() const {
        return sqrtf(x * x + y * y);
    }

    inline float magnitude() const {
        return sqrtf(x * x + y * y);
    }

    inline float norm_sqr() const {
        return x * x + y * y;
    }

    inline point_2d mul_elem(const point_2d& b) const {
        return {x * b.x, y * b.y};
    }

    inline point_2d div_elem(const point_2d& b) const {
        return {x / b.x, y / b.y};
    }

    float dot(const point_2d& b) const {
        return x * b.x + y * b.y;
    }

    // result in [-M_PI..M_PI]
    float angle_to(const point_2d& b) const {
        return normalizeRotation(std::atan2(b.y, b.x) - std::atan2(y, x));
    }

    std::tuple<float, float> tuple() const {
        return std::make_tuple(x, y);
    }

    point_2d rotated(float angle) const {
        return {x * std::cos(angle) - y * std::sin(angle), x * std::sin(angle) + y * std::cos(angle)};
    }

    point_2d normalized() const {
        float n = norm();
        if (n == 0)
            return {0, 0};
        return {x / n, y / n};
    }

    point_2d normal() const {
        return {-y, x};
    }

    float to_direction() const {
        return std::atan2(y, x);
    }

    float dist(const point_2d& other) const {
        return std::sqrt((x - other.x) * (x - other.x) + (y - other.y) * (y - other.y));
    }

    float dist_sqr(const point_2d& other) const {
        return (x - other.x) * (x - other.x) + (y - other.y) * (y - other.y);
    }

    // Angular distance (in rad) between 2 points in relative coordinates. Don't use with absolute coordinates, it won't
    // work. Convert them to relative first if you need it.
    float angular_dist(const point_2d& other, const CamPose& cam_pose) const;

    friend bool operator==(const point_2d& lhs, const point_2d& rhs) {
        return lhs.x == rhs.x && lhs.y == rhs.y;
    }

    friend bool operator!=(const point_2d& lhs, const point_2d& rhs) {
        return !(lhs == rhs);
    }

    friend std::ostream& operator<<(std::ostream& os, const point_2d& p) {
        os << "(" << p.x << ", " << p.y << ")";
        return os;
    }

    void print() {
        printf("x: %f, y: %f\n", x, y);
    }
};

}  // namespace htwk

namespace std {
template <>
struct hash<htwk::point_2d> {
    size_t operator()(const htwk::point_2d& k) const {
        return hash<float>()(k.x) ^ (hash<float>()(k.y) << 1);
    }
};
}  // namespace std
