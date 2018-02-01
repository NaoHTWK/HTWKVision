#include "testimagedata.h"

TestImageData::TestImageData()
{
    visionResult = VisionPtr(new htwk::HTWKVision(STD_WIDTH, STD_HEIGHT));
}
