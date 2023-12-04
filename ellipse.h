#pragma once

#include <vector>

namespace htwk {

class Ellipse{
public:
    float a{},b{},c{},d{},e{},f{};
    float a1{},b1{},c1{},d1{},e1{},f1{};
    float ta{}, tb{};
    float brennpunkt{};
    float trans[2][2]{{},{}};
    float translation[2]{};
    bool found{false};

    Ellipse() = default;

    Ellipse(float ellipse[6])
        : a(ellipse[0]), b(ellipse[1]), c(ellipse[2]),
          d(ellipse[3]), e(ellipse[4]), f(ellipse[5]) {}

    Ellipse(const std::vector<float> &p)
        : a(p[0]), b(p[1]), c(p[2]), d(p[3]), e(p[4]), f(p[5]) {}
};

}  // namespace htwk
