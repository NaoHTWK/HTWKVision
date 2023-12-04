#ifndef COLOR_H
#define COLOR_H

#include <cmath>

namespace htwk {

struct color {
    int cy;
    int cb;
    int cr;

    color() = default;
    color(int _cy, int _cb, int _cr) : cy(_cy), cb(_cb), cr(_cr) {}
    color(const color&) = default;
    color(color&&) = default;
    color& operator=(const color&) = default;
    color& operator=(color&&) = default;
    ~color() = default;

    float dist(const color& o, float color_weight) {
        color d = *this - o;
        d.cb *= color_weight;
        d.cr *= color_weight;
        return std::sqrt(d.cy * d.cy + d.cb * d.cb + d.cr * d.cr);
    }

    color& operator+=(const color& rhs) {
        cy += rhs.cy;
        cb += rhs.cb;
        cr += rhs.cr;
        return *this;
    }
    friend color operator+(color lhs, const color& rhs) {
        lhs += rhs;
        return lhs;
    }
    color& operator-=(const color& rhs) {
        cy -= rhs.cy;
        cb -= rhs.cb;
        cr -= rhs.cr;
        return *this;
    }
    friend color operator-(color lhs, const color& rhs) {
        lhs -= rhs;
        return lhs;
    }
    friend color operator*(color lhs, float rhs) {
        lhs.cy *= rhs;
        lhs.cb *= rhs;
        lhs.cr *= rhs;
        return lhs;
    }
    friend color operator*(float lhs, const color& rhs) {
        return rhs * lhs;
    }
    color& operator/=(float rhs) {
        cy /= rhs;
        cb /= rhs;
        cr /= rhs;
        return *this;
    }

    friend color operator/(color lhs, float rhs) {
        lhs /= rhs;
        return lhs;
    }
};

}  // namespace htwk

#endif  // COLOR_H
