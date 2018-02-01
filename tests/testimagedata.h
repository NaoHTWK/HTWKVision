#ifndef TESTIMAGEDATA_H
#define TESTIMAGEDATA_H

#include <memory>
#include <string>

#include <htwk_vision.h>
#include "testutils.h"
#include <visionresult.pb.h>

typedef std::shared_ptr<htwk::HTWKVision> VisionPtr;

class TestImageData
{
public:
    std::string filename;
    VisionPtr visionResult;
    protobuf::test::vision::VisionFrame groundTruthData;

    TestImageData();
};

#endif // TESTIMAGEDATA_H
