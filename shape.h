#ifndef SHAPE_H_
#define SHAPE_H_

#include <cstdint>
#include <cstdlib>

#include <boost/shared_ptr.hpp>

#include <line.h>
#include <point_2d.h>
#include "viscolor.h"

namespace protobuf {
namespace visualizer {
    class Ellipsis2D;
    class Rectangle2D;
    class Line2D;
    class Text2D;
    class Arc2D;
    class Raster2D;
    class VisualizerTransaction;
}
}

namespace NaoControl
{
class VisualizerTransaction;

class Shape2D {
private:
    virtual void addToVisualizerTransaction(protobuf::visualizer::VisualizerTransaction* vis) const = 0;
    friend class VisualizerTransaction;

public:
    /** This class is not intended to by copied */
    Shape2D(Shape2D& s) = delete;
    Shape2D(Shape2D&& s) = delete;
    Shape2D& operator=(const Shape2D&) = delete;
    Shape2D& operator=(Shape2D&&) = delete;

    Shape2D() = default;
    virtual ~Shape2D() = default;
};

using Shape2DPtr = boost::shared_ptr<Shape2D>;

#define V_NEVER 0

class Line2D : public Shape2D {
private:
    protobuf::visualizer::Line2D* line;

    void addToVisualizerTransaction(protobuf::visualizer::VisualizerTransaction* vis) const override;

public:
    /** This class is not intended to by copied */
    Line2D(Line2D& l) = delete;
    Line2D(Line2D&& l) = delete;
    Line2D& operator=(const Line2D&) = delete;
    Line2D& operator=(Line2D&&) = delete;

    enum Type {
        NORMAL = 0,
        ARROW  = 1
    };

    /**
     * Describe a line from start point (x, y) to end point. If fadingTimeInMs is
     * zero (V_NEVER) means that it is displayed forever.
     *
     * The points have to be supplied in meter -> 1 = 1 meter
     */
    Line2D(float _x1, float _y1, float _x2, float _y2, const Color &_c = Color::WHITE, uint32_t _fadingTimeInMs = V_NEVER);
    Line2D(const htwk::point_2d& p1, const htwk::point_2d& p2, const Color &c = Color::WHITE, uint32_t fadingTimeInMs = V_NEVER) :
        Line2D(p1.x, p1.y, p2.x, p2.y, c, fadingTimeInMs) {}
    Line2D(const htwk::Line& l, const Color &c = Color::WHITE, uint32_t fadingTimeInMs = V_NEVER) :
        Line2D(l.p1(), l.p2(), c, fadingTimeInMs) {}

    Line2D* setType(Type t);

    ~Line2D() override;
};

/**
 * Represent a ellipsis which fill a bounding box
 */
class Ellipse2D : public Shape2D {
private:
    protobuf::visualizer::Ellipsis2D* ellipse;

    void addToVisualizerTransaction(protobuf::visualizer::VisualizerTransaction* vis) const override;

public:
    /** This class is not intended to by copied */
    Ellipse2D(Ellipse2D& e) = delete;
    Ellipse2D(Ellipse2D&& e) = delete;
    Ellipse2D operator=(const Ellipse2D&) = delete;
    Ellipse2D operator=(Ellipse2D&&) = delete;

    /**
     * Describe a Ellipsis as central point with a height, width,
     * with a color and a fading time in ms. If the fading time is zero (V_NEVER),
     * it is displayed forever.
     *
     * The points have to be supplied in meter -> 1 = 1 meter
     *
     */
    Ellipse2D(float x, float y, float w, float h, const Color& _c = Color::WHITE, uint32_t _fadingTimeInMs = V_NEVER);
    Ellipse2D(const htwk::point_2d& p, float w, float h, const Color& c = Color::WHITE, uint32_t fadingTimeInMs = V_NEVER) :
        Ellipse2D(p.x, p.y, w, h, c, fadingTimeInMs) {}

    /**
     * Describe a Ellipsis as central point with a height, width,
     * with a color and a fading time in ms. If the fading time is zero (V_NEVER),
     * it is displayed forever.
     *
     * The angle is in degrees and is the rotation of the ellipse counter clockwise.
     *
     * The points have to be supplied in meter -> 1 = 1 meter
     *
     */
    Ellipse2D(float x, float y, float w, float h, float a, const Color& _c = Color::WHITE, uint32_t _fadingTimeInMs = V_NEVER);

    ~Ellipse2D() override;

    Ellipse2D* setFillColor(const Color& fc);
};

class Rectangle2D : public Shape2D {
private:
    protobuf::visualizer::Rectangle2D* rect;

    void addToVisualizerTransaction(protobuf::visualizer::VisualizerTransaction* vis) const override;

public:
    /** This class is not intended to by copied */
    Rectangle2D(Rectangle2D& r) = delete;
    Rectangle2D(Rectangle2D&& r) = delete;
    Rectangle2D& operator=(const Rectangle2D&) = delete;
    Rectangle2D& operator=(Rectangle2D&&) = delete;

    /**
     * Describe a Rectangle as bounding box with a color and a fading time in ms. If the fading time is zero (V_NEVER),
     * it is displayed forever.
     *
     * The points have to be supplied in meter -> 1 = 1 meter
     *
     */
    Rectangle2D(float x, float y, float w, float h, const Color& _c = Color::WHITE, uint32_t _fadingTimeInMs = V_NEVER);
    ~Rectangle2D() override;

    Rectangle2D* setFillColor(const Color& fc);
};

/**
 * Represent a arc which fill a bounding box
 */
class Arc2D : public Shape2D {
private:
    protobuf::visualizer::Arc2D* arc;

    void addToVisualizerTransaction(protobuf::visualizer::VisualizerTransaction* vis) const override;

public:
    /** This class is not intended to by copied */
    Arc2D(Arc2D& e) = delete;
    Arc2D(Arc2D&& e) = delete;
    Arc2D& operator=(const Arc2D&) = delete;
    Arc2D& operator=(Arc2D&&) = delete;

    enum Type {
        OPEN  = 0,
        CHORD = 1,
        PIE   = 2
    };
    /**
     * Describe an Arc as central point with a height, width,with a color and a fading time in ms. If the fading
     * time is zero (V_NEVER), it is displayed forever.
     *
     * The angles are like in java. http://docs.oracle.com/javase/7/docs/api/java/awt/geom/Arc2D.html
     * The arc will always be non-empty and extend counterclockwise from the first point around to the second point.
     * StartAngle = 0 is 9 o'clock, the ArcTyp is default OPEN.
     *
     * The points have to be supplied in meter -> 1 = 1 meter
     *
     * Angles are in degree.
     *
     */
    Arc2D(float x, float y, float w, float h, float startAngle, float arcAngle, const Color& _c = Color::WHITE, uint32_t _fadingTimeInMs = V_NEVER);
    ~Arc2D() override;

    Arc2D* setFillColor(const Color& fc);
    Arc2D* setArcType(Arc2D::Type type);
};

/**
 * Represents a text which is displayed at a specified position
 */
class Text2D : public Shape2D {
private:
    protobuf::visualizer::Text2D* text2d;

    void addToVisualizerTransaction(protobuf::visualizer::VisualizerTransaction* vis) const override;

public:
    /** This class is not intended to by copied */
    Text2D(Text2D& e) = delete;
    Text2D(Text2D&& e) = delete;
    Text2D& operator=(const Text2D&) = delete;
    Text2D& operator=(Text2D&&) = delete;

    /**
     * Describe a Text as x, y point, font size (not scaled), text with the text,
     * with a color and a fading time in ms. If the fading time is zero (V_NEVER),
     * it is displayed forever.
     *
     * The points have to be supplied in meter -> 1 = 1 meter
     *
     */
    Text2D(const htwk::point_2d& pos, float size, const std::string& text, const Color& _c = Color::WHITE,
           uint32_t _fadingTimeInMs = V_NEVER)
        : Text2D(pos.x, pos.y, size, text, _c, _fadingTimeInMs) {}
    Text2D(float x, float y, float size, const std::string& text, const Color& _c = Color::WHITE, uint32_t _fadingTimeInMs = V_NEVER);
    ~Text2D() override;
};

/**
 * Represent a Raster which can be translated. It depends on your transaction mode how coordinates are handled.
 * When you have absolut coordinates -> The middle position is the given position on the field.
 * When you have realtive coordinates -> (0,0) means the middle of the raster is on the robot.
 */
class Raster2D : public Shape2D {
private:
    protobuf::visualizer::Raster2D* raster;

    void addToVisualizerTransaction(protobuf::visualizer::VisualizerTransaction* vis) const override;

public:
    /** This class is not intended to by copied */
    Raster2D(Raster2D& e) = delete;
    Raster2D(Raster2D&& e) = delete;
    Raster2D& operator=(const Raster2D&) = delete;
    Raster2D& operator=(Raster2D&&) = delete;

    /**
     * Describe a Raster. The central point is on the given coordinate, height, width, a rotation and a color.
     * The given values in the raster are alpha values from [0, 255]. There is also a fading time without a function.
     * If fading time is zero (V_NEVER), it is displayed forever.
     *
     * The positions are in have to be supplied in meter -> 1 = 1 meter
     * When you have absolut coordinates -> The middle position is the given position on the field.
     * When you have realtive coordinates -> (0,0) means the middle of the raster is on the robot.
     *
     * Rotation angles are in degree?
     */
    Raster2D(uint32_t width, uint32_t height, float rotation = 0.f, float scale = 0.3f, htwk::point_2d middle = htwk::point_2d(0, 0),
             const Color& c = Color::WHITE, uint32_t fadingTimeInMs = V_NEVER);

    Raster2D(uint32_t width, uint32_t height, std::vector<uint8_t>& data, float rotation = 0.f, float scale = .3f, htwk::point_2d middle = htwk::point_2d(0, 0),
             const Color& c = Color::WHITE, uint32_t fadingTimeInMs = V_NEVER);

    Raster2D* setData(std::vector<uint8_t>& data);

    ~Raster2D() override;
};


} /* namespace naocontrol */
#endif /* SHAPE_H_ */
