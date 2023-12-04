#include <htwk_vision.h>

#include <chrono>
#include <fstream>
#include <iostream>
#include <set>
#include <string>

#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>

#include <htwkcamposeutils.h>
#include <htwkpngimageprovider.h>
#include <htwkyuv422image.h>
#include <localization_utils.h>

using namespace htwk;
using namespace htwk::image;
using namespace std::chrono;
using namespace std;
namespace bpo = boost::program_options;
namespace bf = boost::filesystem;

#define STD_WIDTH 640
#define STD_HEIGHT 480

inline void saveAsPng(uint8_t *img, int width, int height, const std::string &filename, Yuv422Image::Filter filter = Yuv422Image::NONE) {
    static PngImageSaverPtr pngSaver = getPngImageSaverInstace();
    Yuv422Image yuvImage(img, width, height);
    yuvImage.saveAsPng(pngSaver, filename, filter);
}

void handleProgramArguments(bpo::variables_map &varmap, int argc, char **argv) {
    bpo::options_description options_all("");
    auto tmp = options_all.add_options();
    tmp = tmp("help", "Print this help");
    tmp = tmp("input-image,i", bpo::value<std::string>(), "The image that should be processed");
    tmp = tmp("output-image,o", bpo::value<std::string>(), "The image that should be outputed");
    tmp = tmp("width,w", bpo::value<int>(), "The image width that should be outputed");
    tmp = tmp("height,h", bpo::value<int>(), "The image height that should be outputed");

    bpo::store(bpo::parse_command_line(argc, argv, options_all), varmap);  // parse and store

    bpo::notify(varmap);  // update the varmap

    if (varmap.count("help")) {
        std::cout << options_all;
        exit(0);
    }

    if (varmap.count("input-image") == 0) {
        std::cout << options_all;
        exit(0);
    }

    if (varmap.count("output-image") == 0) {
        std::cout << options_all;
        exit(0);
    }

    if (varmap.count("width") == 0) {
        std::cout << options_all;
        exit(0);
    }

    if (varmap.count("height") == 0) {
        std::cout << options_all;
        exit(0);
    }

}

uint8_t *loadFile(const std::string &filename, PngMetadata &metadata) {
    uint8_t *imageYUV422 = nullptr;
    size_t bufferSize = sizeof(uint8_t) * 2 * STD_WIDTH * STD_WIDTH;

    if (posix_memalign((void **)&imageYUV422, 16, bufferSize) != 0) {
        std::cout << "error allocating aligned memory! reason: " << strerror(errno) << std::endl;
        exit(1);
    }

    if (imageYUV422 == nullptr) {
        std::cout << "Couldn't allocate yuv memory. Exit now." << std::endl;
        exit(1);
    }

    PngImageProviderPtr pngImageProvider = getPngImageProviderInstace(STD_WIDTH, STD_HEIGHT);
    pngImageProvider->loadAsYuv422(filename, imageYUV422, bufferSize, metadata);

    return imageYUV422;
}

void copyOriginal(const string &name, uint32_t width, uint32_t height, uint8_t *imageYUV422) {
    saveAsPng(imageYUV422, width, height, name + "_original.png");
}

void processFile(const std::string &filename_in, const std::string &filename_out, int width, int height, ThreadPool* pool) {
    PngMetadata metadata;
    uint8_t *imageYUV422 = loadFile(filename_in, metadata);

    printf("Processing: %s\n", filename_in.c_str());

    HtwkVisionConfig visionConfig;
    visionConfig.isUpperCam = (filename_in.find("_U.") != std::string::npos);
    visionConfig.lcObjectDetectorConfig.classifyObjData = false;
    visionConfig.lcObjectDetectorConfig.classifyHypData = false;
    visionConfig.lcObjectDetectorConfig.scaledImageWidth = width;
    visionConfig.lcObjectDetectorConfig.scaledImageHeight = height;

    HTWKVision vision(visionConfig, pool);

    int w = visionConfig.lcObjectDetectorConfig.scaledImageWidth;
    int h = visionConfig.lcObjectDetectorConfig.scaledImageHeight;

    std::vector<float> input_image(3*w*h, 0);
    //std::vector<uint8_t> test_image(2*640*480, 255);

    //vision.imagePreprocessor->createInputDataRaw(test_image.data(), input_image);
    //vision.imagePreprocessor->proceed(test_image.data());
    vision.lcImagePreprocessor->createInputDataRaw(imageYUV422, input_image);

    FILE* fp = fopen(filename_out.c_str(), "wb");
    int rc = fwrite(input_image.data(), input_image.size()*sizeof(float), 1, fp);

    if(!rc)
        printf("Error writing to cache file. %s\n", strerror(errno));

    fflush(fp);
    fclose(fp);

    free(imageYUV422);
    exit(0);
}

int main(int argc, char **argv) {
    bpo::variables_map varmap;
    handleProgramArguments(varmap, argc, argv);

    ThreadPool pool("Vision", 1);

    std::string filename_in  = varmap["input-image"].as<std::string>();
    std::string filename_out = varmap["output-image"].as<std::string>();

    int width  = varmap["width"].as<int>();
    int height = varmap["height"].as<int>();

    processFile(filename_in, filename_out, width, height, &pool);
}
