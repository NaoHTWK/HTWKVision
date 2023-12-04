#ifndef FIELD_BORDER_DETECTOR_H
#define FIELD_BORDER_DETECTOR_H

#include <base_detector.h>
#include <image_preprocessor.h>
#include <tfliteexecuter.h>

#include <cstring>
#include <memory>

namespace htwk {

class FieldBorderDetector : public BaseDetector {
public:
    FieldBorderDetector();

    FieldBorderDetector(int8_t* lutCb, int8_t* lutCr, HtwkVisionConfig& config);
    FieldBorderDetector(const FieldBorderDetector&) = delete;
    FieldBorderDetector(const FieldBorderDetector&&) = delete;
    FieldBorderDetector& operator=(const FieldBorderDetector&) = delete;
    FieldBorderDetector& operator=(FieldBorderDetector&&) = delete;
    ~FieldBorderDetector();

    void proceed(uint8_t* img);

    const std::vector<int>& getConvexFieldBorder() const {
        return fieldBorderFull;
    }

    const std::vector<float> getInputParameter() {
        //        std::vector<float> tmpInput(channels * inputWidth * inputHeight);
        //        memcpy(tmpInput.data(), input, tmpInput.size() * sizeof(float));
        //        return tmpInput;
        return imgPreprocessor.getScaledImage();
    }

    void drawInputParameter(uint8_t* yuvImage);
    void drawFieldBorder(uint8_t* yuvImage);

private:
    static constexpr int channels = 3;
    const int inputWidth;
    const int inputHeight;
    bool shouldWeClassify;

    std::vector<int> fieldBorderFull;

    ImagePreprocessor imgPreprocessor;

    TFLiteExecuter tflite;
    float* input;

    void createInputData(uint8_t* img);
    void outputToFieldBorder();
};

}  // namespace htwk

#endif  // FIELD_BORDER_DETECTOR_H
