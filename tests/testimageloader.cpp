#include "testimageloader.h"

#include <stdio.h>

#include <chrono>
#include <functional>

#include <lodepng/lodepng.h>

using namespace std::chrono;
namespace bf  = boost::filesystem;

TestImageLoader::TestImageLoader()
{

}

void TestImageLoader::loadImages(std::deque<ImageInfo>& images, bf::path startPath)
{
    /* We don't want to see all the loading and converting of pngs. We cache the raw data in memory */
    const bf::recursive_directory_iterator end;

    uint32_t width, height;

    for(auto it = bf::recursive_directory_iterator(startPath); it != end; ++it) {
        if(bf::is_regular_file(*it) && it->path().extension() == ".png") {
            std::cout << "Load " << it->path() << std::endl;
            std::string filename = it->path().string();
            uint8_t* yuv422Image = TestUtils::loadFile(filename, width, height);

            if(width != STD_WIDTH && height != STD_HEIGHT) {
                std::cout << "Ignoring file. Has wrong dimensions" << std::endl;
                free(yuv422Image);
            } else {
                images.push_back(std::make_tuple(filename, yuv422Image));
            }
        }
    }
}

void TestImageLoader::measureTime(std::function<void (void)> expr)
{
    high_resolution_clock::time_point t1 = high_resolution_clock::now();
    expr();
    high_resolution_clock::time_point t2 = high_resolution_clock::now();
    duration<double> timespan = duration_cast<duration<double>>(t2 - t1);
    std::cout << "time: " << timespan.count() * 1000 << "ms\n\n";
}

void TestImageLoader::loadResultFile(TestImageData& test)
{
    std::string filename = test.filename + ".result";
    std::cout << "Load " << filename << std::endl;

    FILE* fp = fopen(filename.c_str(), "r");

    if(fp == NULL) {
        printf("Result file does not exist. This is the end! %s %s %d\n", filename.c_str(), __FILE__, __LINE__);
        exit(-1);
    }

    int size;
    if(fread(&size, sizeof(size), 1, fp) == 0){
        printf("Error reading size from result file! %s %s %d\n", filename.c_str(), __FILE__, __LINE__);
        exit(-1);
    }

    uint8_t* buffer = new uint8_t[size];
    if(fread(buffer, size, 1, fp) == 0){
        printf("Error reading protbuf from result file! %s %s %d\n", filename.c_str(), __FILE__, __LINE__);
        exit(-1);
    }

    fclose(fp);

    if(test.groundTruthData.ParseFromArray(buffer, size) == false) {
        printf("Error parsing protbuf from result file! %s %s %d\n", filename.c_str(), __FILE__, __LINE__);
        exit(-1);
    }

    delete [] buffer;
}

void TestImageLoader::processDirectory(std::list<TestImageData>& testData, bf::path startPath, bool loadResultFile)
{
    std::deque<ImageInfo> images;

    std::cout << "Load images" << std::endl;
    measureTime([&] { loadImages(images, startPath); });

    std::cout << "Processing " << images.size() << " images." << std::endl;
    measureTime([&] {
        std::for_each(images.begin(), images.end(), [&testData](ImageInfo& imgInfo) {
            TestImageData testImageData;
            testImageData.filename = std::get<0>(imgInfo);

            const bool isUpperCam = testImageData.filename.find("upper") != std::string::npos || testImageData.filename.find("_U.") != std::string::npos;

            testImageData.visionResult->proceed(std::get<1>(imgInfo), isUpperCam, true, 0, 0);
            testData.push_back(testImageData);
        });
    });

    std::for_each(images.begin(), images.end(), [](ImageInfo i) { free(std::get<1>(i)); });

    if(!loadResultFile)
        return;

    std::cout << "Load result files" << std::endl;
    measureTime([&] {
        for(TestImageData& test : testData) {
            this->loadResultFile(test);
        }
    });
    std::cout << std::endl;
}
