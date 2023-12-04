#pragma once

#include <point_2d.h>

#include <algorithm>

namespace htwk {

struct BoundingBox {
    BoundingBox() : a(), b(), prob(0.f) {}
    BoundingBox(point_2d p1, point_2d p2, float prob)
        : a(std::min(p1.x, p2.x), std::min(p1.y, p2.y)), b(std::max(p1.x, p2.x), std::max(p1.y, p2.y)), prob(prob) {}

    point_2d a;  // top left
    point_2d b;  // bottom right

    float prob;

    float area() const {
        return (b.x - a.x) * (b.y - a.y);
    }

    float iou(const BoundingBox& o) const {
        const float epsilon = 0.0001;

        float x1 = std::max(a.x, o.a.x);
        float y1 = std::max(a.y, o.a.y);
        float x2 = std::min(b.x, o.b.x);
        float y2 = std::min(b.y, o.b.y);

        float width = x2 - x1;
        float height = y2 - y1;

        // There is no overlap
        if (width < 0 || height < 0)
            return 0.f;

        float area_overlap = width * height;
        float area_combined = area() + o.area() - area_overlap;

        return area_overlap / (area_combined + epsilon);
    }

    point_2d center() const {
        return (a + b) / 2;
    }
};

}  // namespace htwk
