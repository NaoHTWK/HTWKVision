#include "shape.h"

#include <cstring>

#include <visualizer.pb.h>

namespace NaoControl
{
Line2D::Line2D(float _x1, float _y1, float _x2, float _y2, const Color &_c, uint32_t _fadingTime) {
    line = new protobuf::visualizer::Line2D();
    line->set_x1(_x1);
    line->set_y1(_y1);
    line->set_x2(_x2);
    line->set_y2(_y2);
    line->set_fadingtime(_fadingTime);
    *line->mutable_color() = _c.color;
}

Line2D::~Line2D()
{
    delete line;
}

void Line2D::addToVisualizerTransaction(protobuf::visualizer::VisualizerTransaction* vis) const
{
    vis->add_lines()->CopyFrom(*line);
}

Line2D* Line2D::setType(Type t)
{
    if(t == NORMAL)
        line->set_type(protobuf::visualizer::Line2D_LineType::Line2D_LineType_NORMAL);
    else
        line->set_type(protobuf::visualizer::Line2D_LineType::Line2D_LineType_ARROW);

    return this;
}


Ellipse2D::Ellipse2D(float x, float y, float w, float h, const Color& _c, uint32_t _fadingTime)
 : Ellipse2D(x, y, w, h, 0, _c, _fadingTime)
{}

Ellipse2D::Ellipse2D(float x, float y, float w, float h, float a, const Color& _c, uint32_t _fadingTime)
{
    ellipse = new protobuf::visualizer::Ellipsis2D();
    ellipse->set_centerx(x);
    ellipse->set_centery(y);
    ellipse->set_width(w);
    ellipse->set_height(h);
    ellipse->set_angle(a);
    ellipse->set_fadingtime(_fadingTime);
    *ellipse->mutable_bordercolor() = _c.color;
}

Ellipse2D* Ellipse2D::setFillColor(const Color &fc)
{
    *ellipse->mutable_fillcolor() = fc.color;
    return this;
}

Ellipse2D::~Ellipse2D()
{
    delete ellipse;
}

void Ellipse2D::addToVisualizerTransaction(protobuf::visualizer::VisualizerTransaction* vis) const
{
    vis->add_ellipses()->CopyFrom(*ellipse);
}


Rectangle2D::Rectangle2D(float x, float y, float w, float h, const Color& _c, uint32_t _fadingTime)
{
    rect = new protobuf::visualizer::Rectangle2D();
    rect->set_upperleftx(x);
    rect->set_upperlefty(y);
    rect->set_width(w);
    rect->set_height(h);
    rect->set_fadingtime(_fadingTime);
    *rect->mutable_bordercolor() = _c.color;
}

Rectangle2D* Rectangle2D::setFillColor(const Color &fc)
{
    *rect->mutable_fillcolor() = fc.color;
    return this;
}

Rectangle2D::~Rectangle2D()
{
    delete rect;
}

void Rectangle2D::addToVisualizerTransaction(protobuf::visualizer::VisualizerTransaction* vis) const
{
    vis->add_rectangles()->CopyFrom(*rect);
}

Arc2D::Arc2D(float x, float y, float w, float h, float startAngle, float arcAngle, const Color& _c, uint32_t _fadingTime)
{
    arc = new protobuf::visualizer::Arc2D();
    arc->set_centerx(x);
    arc->set_centery(y);
    arc->set_width(w);
    arc->set_height(h);
    arc->set_startangle(startAngle);
    arc->set_arcangle(arcAngle);
    arc->set_fadingtime(_fadingTime);
    arc->set_type(protobuf::visualizer::Arc2D_Arc2DType_OPEN);
    *arc->mutable_bordercolor() = _c.color;
}

Arc2D* Arc2D::setFillColor(const Color &fc)
{
    *arc->mutable_fillcolor() = fc.color;
    return this;
}

Arc2D* Arc2D::setArcType(Type type)
{
    arc->set_type((protobuf::visualizer::Arc2D_Arc2DType)type);
    return this;
}

Arc2D::~Arc2D()
{
    delete arc;
}

void Arc2D::addToVisualizerTransaction(protobuf::visualizer::VisualizerTransaction* vis) const
{
    vis->add_arcs()->CopyFrom(*arc);
}

Text2D::Text2D(float x, float y, float size, const std::string& text, const Color& _c, uint32_t _fadingTime)
{
    text2d = new protobuf::visualizer::Text2D();
    text2d->set_x(x);
    text2d->set_y(y);
    text2d->set_size(size);
    text2d->set_text(text);
    text2d->set_fadingtime(_fadingTime);
    *text2d->mutable_color() = _c.color;
}

Text2D::~Text2D()
{
    delete text2d;
}

void Text2D::addToVisualizerTransaction(protobuf::visualizer::VisualizerTransaction* vis) const
{
    vis->add_texts()->CopyFrom(*text2d);
}

Raster2D::Raster2D(uint32_t width, uint32_t height, float rotation, float scale, htwk::point_2d middle, const Color& c,
                   uint32_t fadingTimeInMs) {
    raster = new protobuf::visualizer::Raster2D();
    raster->set_width(width);
    raster->set_height(height);
    raster->set_x(middle.x);
    raster->set_y(middle.y);
    raster->set_scale(scale);
    raster->set_rotation(rotation);
    *raster->mutable_color() = c.color;
    raster->set_fadingtime(fadingTimeInMs);
}

Raster2D::Raster2D(uint32_t width, uint32_t height, std::vector<uint8_t>& data, float rotation, float scale, htwk::point_2d middle,
                   const Color& c, uint32_t fadingTimeInMs)
    : Raster2D(width, height, rotation, scale, middle, c, fadingTimeInMs) {
    setData(data);
}

Raster2D::~Raster2D() {
    delete raster;
}

Raster2D* Raster2D::setData(std::vector<uint8_t>& data) {
    for (uint8_t d : data)
        raster->add_data(d);
    return this;
}

void Raster2D::addToVisualizerTransaction(protobuf::visualizer::VisualizerTransaction* vis) const {
    vis->add_raster()->CopyFrom(*raster);
}

} /* namespace naocontrol */
