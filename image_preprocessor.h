#ifndef IMAGEPREPROCESSOR_H
#define IMAGEPREPROCESSOR_H

#include <vector>

#include <base_detector.h>

namespace htwk {

class ImagePreprocessor : BaseDetector
{
public:
    ImagePreprocessor();

    ImagePreprocessor(int8_t* lutCb, int8_t* lutCr, HtwkVisionConfig &config, int scaledWidth, int scaledHeight);
    ImagePreprocessor(const ImagePreprocessor&) = delete;
    ImagePreprocessor(const ImagePreprocessor&&) = delete;
    ImagePreprocessor& operator=(const ImagePreprocessor&) = delete;
    ImagePreprocessor& operator=(ImagePreprocessor&&) = delete;
    ~ImagePreprocessor() = default;

    void proceed(uint8_t* img);

    const std::vector<float>& getScaledImage() { return scaledImage; }

    void drawScaledImage(uint8_t *yuvImage);

    // Helper function for the machine learning tools
    void createInputDataRaw(const uint8_t *img, std::vector<float> &dest);
private:
    const int scaledWidth;
    const int scaledHeight;

    std::vector<float> scaledImage;

    void scaleImage(uint8_t *img);
    void scaleImage4(uint8_t *img);
};

}
#endif // IMAGEPREPROCESSOR_H
