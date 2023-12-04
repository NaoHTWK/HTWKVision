#include <htwk_vision_config.h>

#include <cstdlib>
#include <cstdio>

#include <boost/filesystem.hpp>
#include <tfliteexecuter.h>

namespace htwk {

HtwkVisionConfig::HtwkVisionConfig() {
    tflitePath = TFLiteExecuter::getTFliteModelPath();
}

}
