#ifndef PARAMETER_H_
#define PARAMETER_H_

#include <cstdlib>
#include <cstdint>

#include <memory>

#include "viscolor.h"

namespace protobuf {
namespace visualizer {
    class Parameter;
    class VisualizerTransaction;
}
}

namespace NaoControl
{
class VisualizerTransaction;

class Parameter;
using ParamPtr = std::shared_ptr<Parameter>;

class Parameter : public std::enable_shared_from_this<Parameter> {
private:
    protobuf::visualizer::Parameter* param;

    friend class VisualizerTransaction;

public:
    /** This class is not intended to by copied */
    Parameter(Parameter& s) = delete;
    Parameter(Parameter&& s) = delete;
    Parameter& operator=(const Parameter&) = delete;
    Parameter& operator=(Parameter&&) = delete;
    explicit Parameter(protobuf::visualizer::Parameter *_param);

    ~Parameter();

    static ParamPtr createInt  (const char* name, int param);
    static ParamPtr createFloat(const char* name, float param);
    static ParamPtr createArray(const char* name, const int*   param, int elemCount);
    static ParamPtr createArray(const char* name, const float* param, int elemCount);

    ParamPtr setColor(const Color& c);
};

}
#endif /* PARAMETER_H_ */
