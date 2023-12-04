#include "viscolor.h"

#include <cmath>

#include <stl_ext.h>
#include <visualizer.pb.h>

namespace NaoControl
{

const Color Color::WHITE   (255, 255, 255);
const Color Color::BLACK   (  0,   0,   0);
const Color Color::RED     (255,   0,   0);
const Color Color::BLUE    (  0,   0, 255);
const Color Color::GREEN   (  0, 255,   0);
const Color Color::YELLOW  (255, 255,   0);
const Color Color::ORANGE  (255, 127,   0);
const Color Color::GREY    (100, 100, 100);
const Color Color::PINK    (255, 105, 180);
const Color Color::MAGENTA (255,   0, 255);
const Color Color::CYAN    (0,   255, 255);
const Color Color::PURPLE  (160,   0, 200);
const Color Color::BROWN   (135,  75,   0);

Color::Color(const uint8_t _r, const uint8_t _g, const uint8_t _b, const uint8_t _a) : r(_r), g(_g), b(_b), a(_a)
{
    color.set_r(_r);
    color.set_g(_g);
    color.set_b(_b);
    color.set_a(_a);
}


Color Color::darker() const
{
    static float FACTOR = 0.9f;
    return Color((int)fmaxf((r * FACTOR), 0),
                 (int)fmaxf((g * FACTOR), 0),
                 (int)fmaxf((b * FACTOR), 0), a);

}

Color Color::brighter() const {
    return Color(clamp(r + 10, 0, 255), clamp(r + 10, 0, 255), clamp(r + 10, 0, 255));
}

} /* namespace naocontrol */
