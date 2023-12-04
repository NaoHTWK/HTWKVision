#ifndef COLOR_H_
#define COLOR_H_

#include <cstdint>
#include <cstdlib>

namespace NaoControl {

class Color {
public:
    static const Color WHITE;
    static const Color BLACK;
    static const Color RED;
    static const Color BLUE;
    static const Color GREEN;
    static const Color YELLOW;
    static const Color ORANGE;
    static const Color GREY;
    static const Color PINK;
    static const Color MAGENTA;
    static const Color CYAN;
    static const Color PURPLE;
    static const Color BROWN;

    Color(uint8_t _r, uint8_t _g, uint8_t _b, uint8_t _a = 255);
    Color() : r(0), g(0), b(0), a(0) {}

    Color darker() const;
    Color brighter() const;
    Color addAlpha(const uint8_t alpha) const {
        return Color(r, g, b, alpha);
    }

    uint8_t r;
    uint8_t g;
    uint8_t b;
    uint8_t a;

private:
    // protobuf::visualizer::Color color;

    friend class Shape2D;
    friend class Line2D;
    friend class Ellipse2D;
    friend class Rectangle2D;
    friend class Arc2D;
    friend class Text2D;
    friend class Raster2D;
    friend class Parameter;
};

}  // namespace NaoControl
#endif /* COLOR_H_ */
