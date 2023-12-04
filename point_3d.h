#pragma once

#include <cmath>

struct point_3d {
    float x{};
    float y{};
    float z{};

    point_3d() = default;
    ~point_3d() = default;
    point_3d(const point_3d&) = default;
    point_3d(point_3d&&) = default;
    point_3d& operator=(const point_3d&) = default;
    point_3d& operator=(point_3d&&) = default;

    constexpr point_3d(float x, float y, float z) : x(x), y(y), z(z) {}

    float norm() const {
        return std::sqrt(x * x + y * y + z * z);
    }

    float norm_sqr() const {
        return x * x + y * y + z * z;
    }

    void normalize() {
        float in = 1.f / norm();
        x *= in;
        y *= in;
        z *= in;
    }

    point_3d rotated_x(float angle) const {
        return {x, y * std::cos(angle) - z * std::sin(angle), y * std::sin(angle) + z * std::cos(angle)};
    }

    point_3d rotated_y(float angle) const {
        return {x * std::cos(angle) + z * std::sin(angle), y, -x * std::sin(angle) + z * std::cos(angle)};
    }

    point_3d rotated_z(float angle) const {
        return {x * std::cos(angle) - y * std::sin(angle), x * std::sin(angle) + y * std::cos(angle), z};
    }

    float dot(const point_3d& b) {
        return x * b.x + y * b.y + z * b.z;
    }

    float angle_diff(const point_3d& b) {
        return acos(dot(b) / (norm() * b.norm()));
    }

    point_3d& operator+=(const point_3d& rhs) {
        x += rhs.x;
        y += rhs.y;
        z += rhs.z;
        return *this;
    }

    point_3d& operator-=(const point_3d& rhs) {
        x -= rhs.x;
        y -= rhs.y;
        z -= rhs.z;
        return *this;
    }

    point_3d& operator*=(float rhs) {
        x *= rhs;
        y *= rhs;
        z *= rhs;
        return *this;
    }

    point_3d& operator/=(float rhs) {
        x /= rhs;
        y /= rhs;
        z /= rhs;
        return *this;
    }

    friend point_3d operator+(point_3d lhs, const point_3d& rhs) {
        lhs += rhs;
        return lhs;
    }

    friend point_3d operator-(point_3d lhs, const point_3d& rhs) {
        lhs -= rhs;
        return lhs;
    }

    friend point_3d operator*(point_3d lhs, float rhs) {
        lhs *= rhs;
        return lhs;
    }

    friend point_3d operator/(point_3d lhs, float rhs) {
        lhs /= rhs;
        return lhs;
    }

    friend point_3d operator*(float lhs, point_3d rhs) {
        rhs *= lhs;
        return rhs;
    }
};
