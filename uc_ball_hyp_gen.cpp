#include "uc_ball_hyp_gen.h"

#include <algorithm_ext.h>
#include <easy/profiler.h>
#include <line.h>
#include <stl_ext.h>

#include <cstring>
#include "async.h"

using namespace std;

namespace htwk {

UpperCamBallHypothesesGenerator::UpperCamBallHypothesesGenerator(int8_t *lutCb,
                                                                 int8_t *lutCr,
                                                                 HtwkVisionConfig &config,
                                                                 ThreadPool* thread_pool)
    : BaseDetector(lutCb, lutCr, config)
    , patchWidth(config.ucBallHypGeneratorConfig.patchWidth)
    , patchHeight(config.ucBallHypGeneratorConfig.patchHeight)
    , imageWidth(config.ucBallHypGeneratorConfig.scaledImageWidth)
    , imageHeight(config.ucBallHypGeneratorConfig.scaledImageHeight)
    , thread_pool(thread_pool)
{
    const auto& hypConf = config.ucBallHypGeneratorConfig;

    if (hypConf.classifyHypData) {
        for(int i = 0; i < num_threads; i++) {
            hypGenExecuter[i].loadModelFromFile(config.tflitePath + "/" + hypConf.model,
                                             {hypConf.hypothesisCount / num_threads, patchHeight, patchWidth, channels}, 1);
            inputHypFinder[i] = hypGenExecuter[i].getInputTensor();
        }
    } else {
        size_t alloc_size = num_threads * patchWidth * patchHeight * channels;

        for(int i = 0; i < num_threads; i++) {
            // aligned_alloc expects multiple of the alignment size as size.
            inputHypFinder[i] = (float*)aligned_alloc(16, ((alloc_size * sizeof(float) + 15) / 16) * 16);

            if (inputHypFinder[i] == nullptr) {
                fprintf(stdout, "%s:%d: %s error allocation input array!", __FILE__, __LINE__, __func__);
                exit(1);
            }
        }
    }
}

UpperCamBallHypothesesGenerator::~UpperCamBallHypothesesGenerator() {
    if (!config.ucBallHypGeneratorConfig.classifyHypData) {
        for(int i = 0; i < num_threads; i++)
            free(inputHypFinder[i]);
    }
}

void UpperCamBallHypothesesGenerator::proceed(CamPose& cam_pose, std::shared_ptr<ImagePreprocessor> imagePreprocessor) {
    Timer t("UpperCamBallHypothesesGenerator", 50);
    EASY_FUNCTION();
    const int patches_y = imageHeight / patchHeight;
    const int patches_x = imageWidth / patchWidth;

    const float* img = imagePreprocessor->getScaledImage().data();

    EASY_BLOCK("UpperCamBallHypothesesGenerator Prepare");
    for (int py = 0; py < patches_y; py++) {
        float* input = inputHypFinder[py];
        for (int px = 0; px < patches_x; px++) {
            float* output = input + px * (patchWidth * patchHeight * 3);

            int iy = py * patchHeight * (patches_x * patchWidth * 3);
            int ix = px * (patchWidth * 3);

            // printf("y: %d, x: %d = %d\n", py, px, (py * patches_x + px) * (patchWidth * patchHeight * 3));
            for (int y = 0; y < patchHeight; y++) {
                for (int x = 0; x < patchWidth; x++) {
                    output[0 + x * 3 + y * (3 * patchWidth)] = img[0 + (ix + x * 3) + (iy + y * (3 * patchWidth * patches_x))];
                    output[1 + x * 3 + y * (3 * patchWidth)] = img[1 + (ix + x * 3) + (iy + y * (3 * patchWidth * patches_x))];
                    output[2 + x * 3 + y * (3 * patchWidth)] = img[2 + (ix + x * 3) + (iy + y * (3 * patchWidth * patches_x))];
                    // printf("pyx(%d, %d), yx(%d, %d), Y%0.2f, o(%d), i(%d)\n", py, px, y, x, output[0 + x * 3 + y * (3
                    // * patchWidth)], 0 + x * 3 + y * (3 * patchWidth), (ix + x * 3) + (iy + y * (3 * patchWidth *
                    // patches_x)));
                }
            }
        }
    }
    EASY_END_BLOCK;

    if (!config.ucBallHypGeneratorConfig.classifyHypData)
        return;

    EASY_BLOCK("UpperCamBallHypothesesGenerator Hyp");
    TaskScheduler scheduler(thread_pool);

    for(int i = 0; i < num_threads; i++) {
        scheduler.addTask([i, this] {
            EASY_FUNCTION();
            hypGenExecuter[i].execute();
        }, {});
    }
    scheduler.run();
    EASY_END_BLOCK;

    hypotheses.clear();
    for (int y = 0; y < patches_y; y++) {
        const float* result = hypGenExecuter[y].getOutputTensor();

        for (int x = 0; x < patches_x; x++) {
            const float* res = result + x * 2;
            ObjectHypothesis h;
            float ux = unscale_x(res[0]) + patchWidth / 2;  // position in patch
            float ox = ux + (x * patchWidth);               // position in downscaled image
            h.x = ox * (width / imageWidth);                // position in big image

            float uy = (unscale_y(res[1]) + patchHeight / 2);
            float oy = uy + (y * patchHeight);
            h.y = oy * (height / imageHeight);

            auto radius = LocalizationUtils::getPixelRadius(h, cam_pose, 0.05f);
            h.r = radius ? *radius : -1000;

            // printf("%d: %0.2f, %0.2f; %0.2f, %0.2f; %d, %d,\n", (y * patches_x + x) * 2, res[0], res[1], ux, uy, h.x, h.y);

            hypotheses.push_back(h);
        }
    }
}

void UpperCamBallHypothesesGenerator::drawPatch(uint8_t* yuvImage, int px, int py) {
    const int blockSizeWidth = width / patchWidth;
    const int blockSizeHeight = height / patchHeight;
    const int channels = 3;

    const int patches_x = imageWidth / patchWidth;
    float* img = inputHypFinder[0] + (py * patches_x + px) * (patchWidth * patchHeight * 3);

    for (int ySample = 0; ySample < patchHeight; ySample++) {
        const int startY = ySample * blockSizeHeight;

        for (int xSample = 0; xSample < patchWidth; xSample++) {
            const int startX = xSample * blockSizeWidth;

            const int vcy = (int)(img[0 + channels * xSample + ySample * channels * patchWidth] * 255.f);
            const int vcb = (int)(img[1 + channels * xSample + ySample * channels * patchWidth] * 255.f);
            const int vcr = (int)(img[2 + channels * xSample + ySample * channels * patchWidth] * 255.f);
            const uint8_t cy = static_cast<uint8_t>(std::min(std::max(0, vcy), 255));
            const uint8_t cb = static_cast<uint8_t>(std::min(std::max(0, vcb), 255));
            const uint8_t cr = static_cast<uint8_t>(std::min(std::max(0, vcr), 255));

            for (int y = startY; y < startY + blockSizeHeight; y++) {
                for (int x = startX; x < startX + blockSizeWidth; x++) {
                    setYCbCr(yuvImage, x, y, cy, cb, cr);
                }
            }
        }
    }
}

}  // namespace htwk
