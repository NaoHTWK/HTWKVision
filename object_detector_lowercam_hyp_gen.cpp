#include "object_detector_lowercam_hyp_gen.h"

#include <cstring>

#include <easy/profiler.h>

#include <algorithm_ext.h>
#include <line.h>
#include <stl_ext.h>

using namespace std;

namespace htwk {

ObjectDetectorLowCamHypGen::ObjectDetectorLowCamHypGen(int8_t* lutCb, int8_t* lutCr, HtwkVisionConfig &config,
                                                       std::string modelFile,
                                                       std::shared_ptr<ImagePreprocessor> imagePreprocessor)
    : BaseDetector(lutCb, lutCr, config),
      inputWidth(config.lcObjectDetectorConfig.scaledImageWidth),
      inputHeight(config.lcObjectDetectorConfig.scaledImageHeight),
      imagePreprocessor(imagePreprocessor) {

    if(config.lcObjectDetectorConfig.classifyHypData) {
        hypGenExecuter.loadModelFromFile(config.tflitePath + "/" + modelFile, {1, inputHeight, inputWidth, channels});
        inputHypFinder = hypGenExecuter.getInputTensor();
    } else {
        size_t alloc_size = inputWidth*inputHeight*channels;
        // aligned_alloc expects multiple of the alignment size as size.
        inputHypFinder = (float*)aligned_alloc(16, ((alloc_size * sizeof(float) + 15) / 16) * 16);

        if (inputHypFinder == nullptr) {
            fprintf(stdout, "%s:%d: %s error allocation input array!", __FILE__, __LINE__, __func__);
            exit(1);
        }
    }
}

ObjectDetectorLowCamHypGen::~ObjectDetectorLowCamHypGen() {
    if(!config.lcObjectDetectorConfig.classifyHypData) {
        free(inputHypFinder);
    }

}

void ObjectDetectorLowCamHypGen::proceed(CamPose& cam_pose) {
    Timer t("ObjectDetectorLowCamHypGen", 50);
    EASY_FUNCTION();
    std::memcpy(inputHypFinder, imagePreprocessor->getScaledImage().data(),
                imagePreprocessor->getScaledImage().size() * sizeof(float));

    if(!config.lcObjectDetectorConfig.classifyHypData)
        return;

    EASY_BLOCK("ObjectDetectorLowerCam Hyp");
    hypGenExecuter.execute();
    EASY_END_BLOCK;

    const float* result = hypGenExecuter.getOutputTensor();
    outputObject.x = (unscale_x(result[0]) + inputWidth / 2) * (width / inputWidth);
    outputObject.y = (unscale_y(result[1]) + inputHeight / 2) * (height / inputHeight);
    auto radius = LocalizationUtils::getPixelRadius(outputObject, cam_pose, 0.05f);
    outputObject.r = radius ? *radius : -1000;
}

}  // namespace htwk
