#ifndef TESTIMAGELOADER_H
#define TESTIMAGELOADER_H

#include <deque>
#include <functional>
#include <tuple>

#include <boost/filesystem.hpp>

#include "testimagedata.h"

class TestImageLoader
{
public:
    TestImageLoader();

    typedef std::tuple<std::string, uint8_t*> ImageInfo;

    void processDirectory(std::list<TestImageData>& testData, boost::filesystem::path startPath, bool loadResultFile);

private:
    void measureTime(std::function<void (void)> expr);
    void loadImages(std::deque<ImageInfo>& images, boost::filesystem::path startPath);
    void loadResultFile(TestImageData& test);
};

#endif // TESTIMAGELOADER_H
