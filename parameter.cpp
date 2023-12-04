#include "parameter.h"

#include <cstring>
#include <visualizer.pb.h>

namespace vis = protobuf::visualizer;

namespace NaoControl
{

Parameter::Parameter(vis::Parameter* _param)
{
    param = _param;
}


Parameter::~Parameter()
{
    param->Clear();
    delete param;
}

ParamPtr Parameter::createInt(const char* name, int param)
{
    auto* p = new vis::Parameter();
    p->set_name(name);
    p->add_intparams(param);

    return std::make_shared<NaoControl::Parameter>(p);
}

ParamPtr Parameter::createFloat(const char* name, const float param)
{
    auto* p = new vis::Parameter();
    p->set_name(name);
    p->add_floatparams(param);

    return std::make_shared<NaoControl::Parameter>(p);
}

ParamPtr Parameter::createArray(const char* name, const int* param, int elemCount)
{
    auto* p = new vis::Parameter();
    p->set_name(name);

    const int* it = param;

    for(int i = 0; i < elemCount; i++, it++) {
        p->add_intparams(*it);
    }

    return std::make_shared<NaoControl::Parameter>(p);
}

ParamPtr Parameter::createArray(const char* name, const float* param, int elemCount)
{
    auto* p = new vis::Parameter();
    p->set_name(name);

    const float* it = param;

    for(int i = 0; i < elemCount; i++, it++) {
        p->add_floatparams(*it);
    }

    return std::make_shared<NaoControl::Parameter>(p);
}

ParamPtr Parameter::setColor(const Color& c)
{
    *param->mutable_color() = c.color;
    return shared_from_this();
}


} /* namespace naocontrol */
