#pragma once

#include <optional>

#include <point_2d.h>

namespace htwk {

struct Line {
    float px1 = 0;
    float py1 = 0;
    float px2 = 0;
    float py2 = 0;

    Line(const Line&) = default;
    Line(Line&&) = default;
    Line& operator=(const Line&) = default;
    Line& operator=(Line&&) = default;

    Line() = delete;
    ~Line() = default;

    Line(const point_2d& p1, const point_2d& p2) : px1(p1.x), py1(p1.y), px2(p2.x), py2(p2.y) {}

    Line(float px1, float py1, float px2, float py2) : px1(px1), py1(py1), px2(px2), py2(py2) {}

    point_2d p1() const {
        return {px1, py1};
    }
    point_2d p2() const {
        return {px2, py2};
    }

    struct OrthogonalProjection {
        const point_2d p;
        const float n;
        const bool inside;
    };

    OrthogonalProjection orthogonalProjection(const point_2d& z) const {
        point_2d u_ = u();
        float n = (z - p1()).dot(u_) / u_.dot(u_);
        return {n * u_ + p1(), n, n >= 0 && n <= 1};
    }

    point_2d closestTo(const point_2d& z) const {
        if (u().norm() == 0)
            return p1();
        OrthogonalProjection p = orthogonalProjection(z);
        if (p.n <= 0) {
            return p1();
        } else if (p.n >= 1) {
            return p2();
        } else {
            return p.p;
        }
    }

    // Returns > 0 if the point is left of the line, == 0 if it's on the line, and < 0 if it's right of the line.
    // If you have a left-handed coordinate system (like the image coordinates) you'll have to flip the sign.
    float side(const point_2d& p) const {
        return (py1 - py2) * p.x + (px2 - px1) * p.y + px1 * py2 - py1 * px2;
    }

    // Intersection with other only if it lies within this line and other.
    std::optional<point_2d> intersect_within(const Line& other) const {
        if (auto p = intersect(other))
            return orthogonalProjection(*p).inside && other.orthogonalProjection(*p).inside ? p : std::nullopt;
        return std::nullopt;
    }

    // Intersection point might be outside!
    std::optional<point_2d> intersect(const Line& other) const {
        float ax = px1 - px2;
        float ay = py1 - py2;
        float bx = other.px1 - other.px2;
        float by = other.py1 - other.py2;
        float ab = ax * -by + ay * bx;

        if (ab == 0)
            return {};

        float qb_ = other.px1 * -by + other.py1 * bx;
        float pa_ = px1 * -ay + py1 * ax;
        point_2d z(1.f / ab * (qb_ * ax - pa_ * bx), 1.f / ab * (qb_ * ay - pa_ * by));
        return z;
    }

    // Orthogonal distance.
    float extendedDistance(const point_2d& z) {
        convertToHessian();
        return std::abs(z.dot(n) - d);
    }

    float distance(const point_2d& z) {
        point_2d u = p2() - p1();
        point_2d v1 = z - p1();

        if (u.norm() == 0) {
            return (p1() - z).norm();
        }
        float a = v1.dot(u) / u.dot(u);
        if (a < 0)
            return (p1() - z).norm();
        else if (a > 1)
            return (p2() - z).norm();
        else {
            convertToHessian();
            return std::abs(z.dot(n) - d);
        }
    }

    // Orthogonal distance.
    std::optional<float> innerDistance(const point_2d& z) {
        point_2d u = p2() - p1();
        point_2d v1 = z - p1();

        if (u.norm() == 0) {
            return (p1() - z).norm();
        }
        float a = v1.dot(u) / u.dot(u);
        if (a < 0 || a > 1)
            return {};
        else {
            convertToHessian();
            return std::abs(z.dot(n) - d);
        }
    }

    // result in [-M_PI..M_PI]
    float angle_to(const Line& l) const {
        return u().angle_to(l.u());
    }

    float norm() const {
        point_2d p1(px1, py1);
        point_2d p2(px2, py2);
        point_2d u = p2 - p1;
        return u.norm();
    }

    float norm_sqr() const {
        point_2d p1(px1, py1);
        point_2d p2(px2, py2);
        point_2d u = p2 - p1;
        return u.norm_sqr();
    }

    Line reversed() const {
        return {px2, py2, px1, py1};
    }

    Line rotated(float angle) const {
        return {p1().rotated(angle), p2().rotated(angle)};
    }

    point_2d u() const {
        return p2() - p1();
    }

    std::string toString() const {
        return "[" + std::to_string(px1) + "," + std::to_string(py1) + "]->[" + std::to_string(px2) + "," +
               std::to_string(py2) + "]";
    }

    friend bool operator==(const Line& lhs, const Line& rhs) {
        return lhs.p1() == rhs.p1() && lhs.p2() == rhs.p2();
    }

    friend bool operator!=(const Line& lhs, const Line& rhs) {
        return !(lhs == rhs);
    }
    Line& operator+=(const point_2d& rhs) {
        px1 += rhs.x;
        py1 += rhs.y;
        px2 += rhs.x;
        py2 += rhs.y;
        hessian_available = false;
        return *this;
    }

    friend Line operator+(Line lhs, const point_2d& rhs) {
        lhs += rhs;
        return lhs;
    }

    Line& operator-=(const point_2d& rhs) {
        px1 -= rhs.x;
        py1 -= rhs.y;
        px2 -= rhs.x;
        py2 -= rhs.y;
        hessian_available = false;
        return *this;
    }

    friend Line operator-(Line lhs, const point_2d& rhs) {
        lhs -= rhs;
        return lhs;
    }

    void convertToHessian() {
        if (hessian_available)
            return;
        n = u().normal().normalized();
        if (p1().dot(n) < 0)
            n = -n;
        d = p1().dot(n);
        hessian_available = true;
    }

private:
    bool hessian_available = false;
    float d = 0;
    point_2d n{0, 0};
};

}  // namespace htwk
