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
#include <hypotheses_generator.h>
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

inline void setY(uint8_t *const img, const uint32_t width, const int32_t x, int32_t y, const uint8_t c) {
    img[(x + y * width) << 1] = c;
}

void drawFieldColor(const string &name, const uint8_t *const orig_img, FieldColorDetector *fcd, int width, int height) {
    uint8_t *img = (uint8_t *)malloc(sizeof(uint8_t) * width * height * 2);
    memcpy(img, orig_img, sizeof(uint8_t) * width * height * 2);
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            int cy = fcd->getY(img, x, y);
            int cb = fcd->getCb(img, x, y);
            int cr = fcd->getCr(img, x, y);
            if (fcd->isGreen(cy, cb, cr)) {
                setY(img, width, x, y, 0);
            }
        }
    }

    saveAsPng(img, width, height, name + "_fieldcolordetector.png");
    free(img);
}

void drawLineSegments(const string &name, const uint8_t *const orig_img, RegionClassifier *rc, int width, int height) {
    const std::vector<LineSegment *> &lineSegments = rc->lineSegments;
    uint8_t *img = (uint8_t *)malloc(sizeof(uint8_t) * width * height * 2);
    memcpy(img, orig_img, sizeof(uint8_t) * width * height * 2);
    for (const LineSegment *ls : lineSegments) {
        if (ls->x < 0 || ls->x >= width || ls->y < 0 || ls->y >= height)
            continue;
        for (float d = 0; d <= 5; d += 0.1) {
            int px = (int)(ls->x + ls->vx * d);
            int py = (int)(ls->y + ls->vy * d);
            if (px < 0 || py < 0 || px >= width || py >= height)
                continue;
            setY(img, width, px, py, 255);
        }
        setY(img, width, ls->x, ls->y, 0);
    }

    saveAsPng(img, width, height, name + "_regionclassifier.png");
}

void drawScanLines(const string &name, const uint8_t *const orig_img, RegionClassifier *rc, int width, int height) {
    uint8_t *img = (uint8_t *)malloc(sizeof(uint8_t) * width * height * 2);
    memcpy(img, orig_img, sizeof(uint8_t) * width * height * 2);

    Scanline *scanVertical = rc->getScanVertical();
    for (int k = 0; k < rc->getScanVerticalSize(); k++) {
        Scanline sl = scanVertical[k];
        for (int i = sl.edgeCnt; i > 0; i--) {
            //            std::cout << sl.edgesY[i-1] << " -> " << sl.edgesY[i] << std::endl;
            for (int y = sl.edgesY[i]; y < sl.edgesY[i - 1]; y++) {
                //                setY(img,width,sl.edgesX[0],y,0);
                if (sl.regionsIsGreen[i - 1]) {
                    setY(img, width, sl.edgesX[0], y, 255);
                }
                if (sl.regionsIsWhite[i - 1]) {
                    setY(img, width, sl.edgesX[0], y, 0);
                }
            }
        }
    }

    Scanline *scanHorizontal = rc->getScanHorizontal();
    for (int k = 0; k < rc->getScanHorizontalSize(); k++) {
        Scanline sl = scanHorizontal[k];
        for (int i = sl.edgeCnt; i > 0; i--) {
            //            std::cout << sl.edgesY[i-1] << " -> " << sl.edgesY[i] << std::endl;
            for (int x = sl.edgesX[i]; x < sl.edgesX[i - 1]; x++) {
                //                setY(img,width,sl.edgesX[0],y,0);
                if (sl.regionsIsGreen[i - 1]) {
                    setY(img, width, x, sl.edgesY[0], 255);
                }
                if (sl.regionsIsWhite[i - 1]) {
                    setY(img, width, x, sl.edgesY[0], 0);
                }
            }
        }
    }

    saveAsPng(img, width, height, name + "_scanlines.png");
    free(img);
}

void drawFieldBorder(const string &name, const uint8_t *const orig_img, std::shared_ptr<FieldBorderDetector> fd, RegionClassifier *rc, int width, int height) {
    uint8_t *img = (uint8_t *)malloc(sizeof(uint8_t) * width * height * 2);
    memcpy(img, orig_img, sizeof(uint8_t) * width * height * 2);
    const std::vector<int>& fieldBorder = fd->getConvexFieldBorder();
    for (int x = 0; x < width; x++) {
        int y = fieldBorder[x];
        if (y < 0 || y >= height)
            continue;
        setY(img, width, x, y, 255);
    }

    saveAsPng(img, width, height, name);
    free(img);
}


void drawLine(uint8_t *img, int width, int height, int x1, int y1, int x2, int y2, const int yValue) {
    float dx = x2 - x1;
    float dy = y2 - y1;
    if (dy != 0) {
        for (int y = 0; y < height; y++) {
            float n = (y - y1) / dy;
            if (n < 0 || n > 1)
                continue;
            int x = (int)(x1 + n * dx);
            if (x < 0 || x >= width)
                continue;
            setY(img, width, x, y, yValue);
        }
    }
    if (dx != 0) {
        for (int x = 0; x < width; x++) {
            float n = (x - x1) / dx;
            if (n < 0 || n > 1)
                continue;
            int y = (int)(y1 + n * dy);
            if (y < 0 || y >= height)
                continue;
            setY(img, width, x, y, yValue);
        }
    }
}

void drawHorizon(uint8_t *img, int width, int height, CamPose &camPose) {

    auto horizon = LocalizationUtils::getHorizon(camPose);

    if (!horizon)
        return;

    drawLine(img, width, height, horizon->px1, horizon->py1, horizon->px2, horizon->py2, 255);
}

void drawLineEdge(uint8_t *img, const LineEdge &ls, int c, LineDetector *ld, int width, int height) {
    for (float d = 0; d <= 1; d += 0.001) {
        int px = (int)round(ls.px1 * (1 - d) + ls.px2 * d);
        int py = (int)round(ls.py1 * (1 - d) + ls.py2 * d);
        if (px < 0 || py < 0 || px >= width || py >= height)
            continue;
        setY(img, width, px, py, c);
    }
}

void drawLines(const string &name, const uint8_t *const orig_img, LineDetector *ld, int width, int height) {
    uint8_t *img = (uint8_t *)malloc(sizeof(uint8_t) * width * height * 2);
    memcpy(img, orig_img, sizeof(uint8_t) * width * height * 2);
    std::vector<LineGroup> linesList = ld->getLineGroups();
    for (const LineGroup &it : linesList) {
        drawLineEdge(img, it.lines[0], 0, ld, width, height);
        drawLineEdge(img, it.lines[1], 0, ld, width, height);
    }

    saveAsPng(img, width, height, name + "_linedetector.png");
    free(img);
}

void drawHypothesis(const string &name, const uint8_t *const orig_img, HypothesesGenerator *hg, int width, int height) {
    uint8_t *img = (uint8_t *)malloc(sizeof(uint8_t) * width * height * 2);
    memcpy(img, orig_img, sizeof(uint8_t) * width * height * 2);
    for (const ObjectHypothesis &it : hg->getHypotheses()) {
        int startX = it.x - it.r, endX = it.x + it.r;
        int startY = it.y - it.r, endY = it.y + it.r;
        int p = (int)(255 * it.rating);
        if (startX < 0 || startY < 0 || width < endX || height < endY)
            continue;

        for (int i = startX; i < endX; i++) {
            setY(img, width, i, startY, p);
            setY(img, width, i, endY, p);
        }
        for (int i = startY; i < endY; i++) {
            setY(img, width, startX, i, p);
            setY(img, width, endX, i, p);
        }
    }

    saveAsPng(img, width, height, name + "_hypothesisGen.png");
    free(img);
}

void drawAllHypothesis(const string &name, const uint8_t *const orig_img, const std::vector<ObjectHypothesis>& hg, int width, int height) {
    uint8_t *img = (uint8_t *)malloc(sizeof(uint8_t) * width * height * 2);
    int cnt = 0;
    for (const ObjectHypothesis &it : hg) {
        int r = it.r > 0 ? it.r : 5;
        memcpy(img, orig_img, sizeof(uint8_t) * width * height * 2);
        int startX = it.x - r, endX = it.x + r;
        int startY = it.y - r, endY = it.y + r;
        // int p=std::min(255,std::max(0,(int)(it.rating/2.4f))); // 5Points
        // int p=std::min(255,std::max(0,(int)(it.rating/3.1))); // Integral
        // int p=std::min(255,std::max(0,(int)(it.rating/1.5))); // inner-outer
        // int p=std::min(255,std::max(0,(int)(it.rating/7.65))); // cross
        int p = std::min(255, std::max(0, (int)(it.rating / 2.4f)));  // crossPoints
        if (startX < 0 || startY < 0 || width < endX || height < endY)
            continue;

        for (int i = startX; i < endX; i++) {
            setY(img, width, i, startY, p);
            setY(img, width, i, endY, p);
        }
        for (int i = startY; i < endY; i++) {
            setY(img, width, startX, i, p);
            setY(img, width, endX, i, p);
        }
        saveAsPng(img, width, height, name + "_hypo_" + std::to_string(cnt) + "_" + std::to_string(p) + ".png");
        cnt++;
    }

    free(img);
}

void drawRatingHypo(const string &name, const uint8_t *const orig_img, HypothesesGenerator *hg, int width, int height) {
    uint8_t *img = (uint8_t *)malloc(sizeof(uint8_t) * width * height * 2);
    memcpy(img, orig_img, sizeof(uint8_t) * width * height * 2);

    // Hypo Blocks
    //    int iWidth = (int)width*0.5;
    //    int iHeight = (int)height*0.5;
    //    for (int y=0;y<iHeight;y++){
    //        for (int x=0;x<iWidth;x++){
    //            int cy=std::max(0,std::min((int)(128+0.1*hg->getRatingImg()[x+y*iWidth]),255));
    //            setY(img,width,x*2,y*2,cy);
    //            setY(img,width,x*2+1,y*2,cy);
    //            setY(img,width,x*2,y*2+1,cy);
    //            setY(img,width,x*2+1,y*2+1,cy);
    //        }
    //    }

    //--------Hypo Blur
    int scale = 4;
    int iWidth = (int)width / scale;
    int iHeight = (int)height / scale;
    for (int y = 0; y < iHeight; y++) {
        for (int x = 0; x < iWidth; x++) {
            int cy = std::max(0, std::min((int)(0.1 * hg->getRatingImg()[x + y * iWidth]), 255));
            for (int ny = y * scale; ny < y * scale + scale; ny++) {
                for (int nx = x * scale; nx < x * scale + scale; nx++) {
                    setY(img, width, nx, ny, cy);
                }
            }
        }
    }

    std::string newName = name + "_ratingIMG.png";
    printf("XXX %s\n", newName.c_str());
    saveAsPng(img, width, height, name + "_ratingIMG.png", Yuv422Image::CONVERT_TO_GREY);
    free(img);
}

void drawBall(const string &name, const uint8_t *const orig_img, std::optional<ObjectHypothesis> ball, int width, int height) {
    if (ball) {
        uint8_t *img = (uint8_t *)malloc(sizeof(uint8_t) * width * height * 2);
        memcpy(img, orig_img, sizeof(uint8_t) * width * height * 2);

        int bx = ball->x;
        int by = ball->y;
        int r = ball->r;
        for (float a = 0; a < M_PI * 2; a += 0.01) {
            int nx = bx + std::sin(a) * r;
            int ny = by + std::cos(a) * r;
            if (nx < 0 || ny < 0 || nx >= width || ny >= height)
                continue;
            setY(img, width, nx, ny, 255);
        }

        saveAsPng(img, width, height, name + "_balldetector.png");
        free(img);
    }
}

void drawCircle(const string &name, const uint8_t *const orig_img, RansacEllipseFitter *raf, RegionClassifier *rc, int width, int height) {
    uint8_t *img = (uint8_t *)malloc(sizeof(uint8_t) * width * height * 2);
    memcpy(img, orig_img, sizeof(uint8_t) * width * height * 2);
    if (raf->getEllipse().found) {
        float stepWidth = M_PI * 2 / 64;
        float lastX = 0;
        float lastY = 0;
        for (double a = 0; a <= M_PI * 2 + 0.00001; a += stepWidth) {
            float x = sinf(a);
            float y = cosf(a);
            point_2d point(x * raf->getEllipse().ta, y * raf->getEllipse().tb);
            raf->transformPoInv(point, raf->getEllipse().trans, raf->getEllipse().translation);
            float px = point.x;
            float py = point.y;
            if (a > 0) {
                for (float d = 0; d <= 1; d += 0.01) {
                    int nx = (int)round(px * (1 - d) + lastX * d);
                    int ny = (int)round(py * (1 - d) + lastY * d);
                    if (nx < 0 || ny < 0 || nx >= width || ny >= height)
                        continue;
                    setY(img, width, nx, ny, 0);
                }
            }
            lastX = px;
            lastY = py;
        }
    }

    saveAsPng(img, width, height, name + "_ellipsefitter.png");
    free(img);
}

void drawObstacleInputParameter(const string &name, const uint8_t *const orig_img, LowerCamObstacleDetection *od, int width, int height) {
    uint8_t *img = (uint8_t *)malloc(sizeof(uint8_t) * width * height * 2);
    memcpy(img, orig_img, sizeof(uint8_t) * width * height * 2);
    od->drawInputParameter(img);
    saveAsPng(img, width, height, name + "_obstacledetector_input.png");
    free(img);
}

void drawObstacleResult(const string &name, const uint8_t *const orig_img, LowerCamObstacleDetection *od, int width, int height) {
    uint8_t *img = (uint8_t *)malloc(sizeof(uint8_t) * width * height * 2);
    memcpy(img, orig_img, sizeof(uint8_t) * width * height * 2);
    od->drawResult(img);
    saveAsPng(img, width, height, name + "_obstacledetector_result.png");
    free(img);
}

void drawScaledImage(const string &name, const uint8_t *const orig_img, std::shared_ptr<ImagePreprocessor> imgPrep, int width, int height) {
    uint8_t *img = (uint8_t *)malloc(sizeof(uint8_t) * width * height * 2);
    memcpy(img, orig_img, sizeof(uint8_t) * width * height * 2);
    imgPrep->drawScaledImage(img);
    saveAsPng(img, width, height, name + "_scaled_image_result.png");
    free(img);
}

void drawBallHypothesesImagePatch(const string &name, const uint8_t *const orig_img, std::shared_ptr<UpperCamBallHypothesesGenerator> hyp, int width, int height) {
    uint8_t *img = (uint8_t *)malloc(sizeof(uint8_t) * width * height * 2);
    memcpy(img, orig_img, sizeof(uint8_t) * width * height * 2);
    hyp->drawPatch(img, 2, 2);
    saveAsPng(img, width, height, name + "_patch_result.png");
    free(img);
}

void drawGoals(const string &name, const uint8_t *const orig_img, std::shared_ptr<UpperCamGoalPostDetector> gpDetector, std::shared_ptr<FieldBorderDetector> fd, int width, int height) {
    uint8_t *img = (uint8_t *)malloc(sizeof(uint8_t) * width * height * 2);
    memcpy(img, orig_img, sizeof(uint8_t) * width * height * 2);

    const std::vector<int>& fieldBorder = fd->getConvexFieldBorder();
    for (int x = 0; x < width; x++) {
        int y = fieldBorder[x];
        if (y < 0 || y >= height)
            continue;
        setY(img, width, x, y, 255);
    }

    for (const ObjectHypothesis &it : gpDetector->getHypotheses()) {
        printf("GoalPost prob: %.2f (%d, %d)\n", it.prob, it.x, it.y);

        int startX = std::clamp(it.x - it.r, 0, width-1), endX = std::clamp(it.x + it.r, 0, width-1);
        int startY = 0, endY = std::clamp(it.y, 0, height-1);
        int p = 255*((it.prob - 0.9)*10);
        if (startX < 0 || startY < 0 || width < endX || height < endY)
            continue;

        for (int i = startX; i < endX; i++) {
            setY(img, width, i, startY, p);
            setY(img, width, i, endY, p);
        }
        for (int i = startY; i < endY; i++) {
            setY(img, width, startX, i, p);
            setY(img, width, endX, i, p);
        }
    }

    saveAsPng(img, width, height, name + "_goalposts.png");
    free(img);
}

void drawCenterCirclePoints(const string &name, const uint8_t *const orig_img, std::shared_ptr<UpperCamCenterCirclePointDetector> ccpDetector, std::shared_ptr<FieldBorderDetector> fd, int width, int height) {
    uint8_t *img = (uint8_t *)malloc(sizeof(uint8_t) * width * height * 2);
    memcpy(img, orig_img, sizeof(uint8_t) * width * height * 2);

    const std::vector<int>& fieldBorder = fd->getConvexFieldBorder();
    for (int x = 0; x < width; x++) {
        int y = fieldBorder[x];
        if (y < 0 || y >= height)
            continue;
        setY(img, width, x, y, 255);
    }

    for (const ObjectHypothesis &it : ccpDetector->getHypotheses()) {
        printf("CenterCirclePost prob: %.2f (%d, %d)\n", it.prob, it.x, it.y);

        int startX = std::clamp(it.x - it.r, 0, width-1), endX = std::clamp(it.x + it.r, 0, width-1);
        int startY = std::clamp(it.y-it.r, 0, height-1), endY = std::clamp(it.y+it.r, 0, height-1);
        int p = 255*((it.prob - 0.9)*10);
        if (startX < 0 || startY < 0 || width < endX || height < endY)
            continue;

        for (int i = startX; i < endX; i++) {
            setY(img, width, i, startY, p);
            setY(img, width, i, endY, p);
        }
        for (int i = startY; i < endY; i++) {
            setY(img, width, startX, i, p);
            setY(img, width, endX, i, p);
        }
    }

    saveAsPng(img, width, height, name + "_centercirclepoints.png");
    free(img);
}

void drawRobotDetectorInputParameter(const string &name, const uint8_t *const orig_img, HTWKVision &vision, uint32_t width, uint32_t height) {
    uint8_t *img = (uint8_t *)malloc(sizeof(uint8_t) * width * height * 2);
    memcpy(img, orig_img, sizeof(uint8_t) * width * height * 2);
    vision.ucRobotDetector->drawInputParameter(img);
    saveAsPng(img, width, height, name + "_robotdetector_input.png");
    free(img);
}

void drawRobotDetectorResult(const string &name, const uint8_t *const orig_img, HTWKVision &vision,
                             uint32_t width, uint32_t height) {
    uint8_t *img = (uint8_t *)malloc(sizeof(uint8_t) * width * height * 2);
    memcpy(img, orig_img, sizeof(uint8_t) * width * height * 2);

    for (const auto rbb : vision.ucRobotDetector->getBoundingBoxes()) {
        const auto &bb = rbb.bb;
        printf("%.2f: %.2f %.2f %.2f %.2f -- %.2f %.2f\n", bb.prob, bb.a.x, bb.a.y, bb.b.x, bb.b.y, rbb.dist, rbb.angle);

        int x1 = (int)bb.a.x;
        int y1 = (int)bb.a.y;
        int x2 = (int)bb.b.x;
        int y2 = (int)bb.b.y;


        drawLine(img, width, height, x1, y1, x2, y1, 255);
        drawLine(img, width, height, x1, y2, x2, y2, 255);

        drawLine(img, width, height, x1, y1, x1, y2, 255);
        drawLine(img, width, height, x2, y1, x2, y2, 255);
    }

    saveAsPng(img, width, height, name + "_robotdetector_result.png");
    free(img);
}

void handleProgramArguments(bpo::variables_map &varmap, int argc, char **argv) {
    bpo::options_description options_all("");
    auto tmp = options_all.add_options();
    tmp = tmp("help,h", "Print this help");
    tmp = tmp("dir,d", bpo::value<std::string>(), "Process a directory of images");
    tmp = tmp("whitelist", bpo::value<std::string>(), "Path to whitelist file for directory parsing.");
    tmp = tmp("image,i", bpo::value<std::string>(), "The image that should be processed");
    tmp = tmp("cycles,c", bpo::value<int>()->default_value(1), "How often process a image with the vision.");
    tmp = tmp("resultImages,r", bpo::value<std::string>()->default_value(""), "Path for result images.");
    tmp = tmp("writeTimeFile,w", bpo::value<bool>()->default_value(false), "Write a time.csv file");

    bpo::store(bpo::parse_command_line(argc, argv, options_all), varmap);  // parse and store

    bpo::notify(varmap);  // update the varmap

    if (varmap.count("help")) {
        std::cout << options_all;
        exit(0);
    }
    if (varmap.count("image") == 0 && varmap.count("dir") == 0) {
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

void writeDebugFiles(const string &name, uint32_t width, uint32_t height, uint8_t *imageYUV422, HTWKVision &vision, CamPose& camPose) {
    //    copyOriginal(name, width, height, imageYUV422);
    //    drawFieldColor(name, imageYUV422, vision.fieldColorDetector, width, height);
    //    drawLineSegments(name, imageYUV422, vision.regionClassifier, width, height);
        drawScanLines(name, imageYUV422, vision.regionClassifier, width, height);
    //    drawHorizon(imageYUV422, width, height, camPose);
    //drawFieldBorder(name, imageYUV422, vision.fieldBorderDetector, vision.regionClassifier, width, height);
    //    drawLines(name, imageYUV422, vision.lineDetector, width, height);
    //    drawRatingHypo(name,imageYUV422,vision.hypothesesGenerator, width, height);
    //     drawAllHypothesis(name,imageYUV422,vision.ucBallHypGenerator->getHypotheses(), width, height);
    //drawGoals(name,imageYUV422,vision.ucGoalPostDetector, vision.fieldBorderDetector, width,height);
    //drawCenterCirclePoints(name,imageYUV422,vision.ucCenterCirclePointDetector, vision.fieldBorderDetector, width,height);
    //    drawHypothesis(name,imageYUV422,vision.hypothesesGenerator, width, height);
    //    drawHypothesis(name,imageYUV422,vision.robotAreaDetector, width, height);
    // drawBall(name, imageYUV422, vision.getBall(), width, height);
    //    drawCircle(name, imageYUV422, vision.ellipseFitter, vision.regionClassifier, width, height);
    //drawObstacleInputParameter(name, imageYUV422, vision.obstacleDetectionLowCam, width, height);
    //drawObstacleResult(name, imageYUV422, vision.obstacleDetectionLowCam, width, height);
    //drawScaledImage(name,imageYUV422,vision.ucImagePreprocessor,width,height);
    //drawBallHypothesesImagePatch(name,imageYUV422,vision.ucBallHypGenerator,width,height);
    //drawRobotDetectorResult(name, imageYUV422, vision, width, height);
    //drawRobotDetectorInputParameter(name, imageYUV422, vision, width, height);
}

void processOneFile(const std::string &filename, int processingCount, const std::string &debugPath, ThreadPool* pool) {
    PngMetadata metadata;
    uint8_t *imageYUV422 = loadFile(filename, metadata);

    high_resolution_clock::time_point t1 = high_resolution_clock::now();

    HtwkVisionConfig visionConfig;
    visionConfig.isUpperCam = (filename.find("_U.") != std::string::npos);
    HTWKVision vision(visionConfig, pool);
    CamPose cam_pose = getCamPoseFromMetadata(metadata);
    for (int i = 0; i < processingCount; i++) {
        vision.proceed(imageYUV422, cam_pose);
    }
    //TODO time measuerment missing
    // vision.printProfilingResults(true);

    high_resolution_clock::time_point t2 = high_resolution_clock::now();
    duration<double> time_span = duration_cast<duration<double>>(t2 - t1);
    std::cout << "time: " << time_span.count() * 1000 << "ms\n";

    if (!debugPath.empty()) {
        int start = filename.rfind('/') + 1;
        writeDebugFiles(debugPath + "/" + filename.substr(start, filename.size() - start - 4), STD_WIDTH, STD_HEIGHT, imageYUV422, vision, cam_pose);
    }

    free(imageYUV422);
    exit(0);
}

struct ipr {
    uint8_t *img;
    PngMetadata metadata;
    std::string name;
};

void processDirectory(const std::string &path, int processingCount, const std::string &debugPath, const bool writeTimeFile, ThreadPool* pool) {
    std::deque<ipr> images;

    /* We don't want to see all the loading and converting of pngs. We cache the raw data in memory */
    const bf::recursive_directory_iterator end;

    bf::path startPath(path);
    for (auto it = bf::recursive_directory_iterator(startPath); it != end; ++it) {
        if (bf::is_regular_file(*it) && it->path().extension() == ".png") {
            std::cout << "Load " << it->path() << std::endl;
            std::string filename = it->path().string();
            ipr img;
            img.name = filename;
            img.img = loadFile(filename, img.metadata);

            images.push_back(img);
        }
    }

    std::cout << std::endl << std::endl << "Processing " << images.size() << " images." << std::endl;
    high_resolution_clock::time_point t1 = high_resolution_clock::now();

    HtwkVisionConfig upperConfig;
    HtwkVisionConfig lowerConfig;
    lowerConfig.isUpperCam = false;
    HTWKVision upperVision(upperConfig, pool);
    HTWKVision lowerVision(lowerConfig, pool);
    std::string statsFileUpper;
    std::string statsFileLower;
    statsFileUpper.append(path + "/timeUpper.csv");
    statsFileLower.append(path + "/timeLower.csv");

    for (const ipr &img : images) {
        boost::filesystem::path p(img.name);
        std::string imgName = p.filename().string();
        bool isUpper = (imgName.find("_U.") != std::string::npos);
        std::cout << imgName << std::endl;

        CamPose cam_pose = getCamPoseFromMetadata(img.metadata);
        if (isUpper) {
            for (int i = 0; i < processingCount; i++) {
                upperVision.proceed(img.img, cam_pose);
            }
            if (writeTimeFile)
                ;
            // upperVision.writeProfilingFile(statsFileUpper,imgName);
            // TODO time measuerment missing
        } else {
            for (int i = 0; i < processingCount; i++) {
                lowerVision.proceed(img.img, cam_pose);
            }
            if (writeTimeFile)
                ;
            // TODO time measuerment missing
            // lowerVision.writeProfilingFile(statsFileLower,imgName);
        }

        if (!debugPath.empty()) {
            if (isUpper) {
                writeDebugFiles(debugPath + "/" + imgName, STD_WIDTH, STD_HEIGHT, img.img, upperVision, cam_pose);
            } else {
                writeDebugFiles(debugPath + "/" + imgName, STD_WIDTH, STD_HEIGHT, img.img, lowerVision, cam_pose);
            }
        }
    }
    if (!writeTimeFile) {
        printf("\n\nUpper Camera timing:\n");
        // TODO time measuerment missing
        // upperVision.printProfilingResults(true);
        printf("\nLower Camera timing:\n");
        // lowerVision.printProfilingResults(true);
    }

    high_resolution_clock::time_point t2 = high_resolution_clock::now();
    duration<double> time_span = duration_cast<duration<double>>(t2 - t1);
    std::cout << "time: " << time_span.count() * 1000 << "ms\n";

    for (const ipr &img : images) {
        free(img.img);
    }
    exit(0);
}

void processDirectoryWithWhitelist(const std::string &path, const std::set<string> &whitelist, const std::string &debugPath, ThreadPool* pool) {
    const bf::recursive_directory_iterator end;

    HtwkVisionConfig upperConfig;
    HtwkVisionConfig lowerConfig;
    lowerConfig.isUpperCam = false;

    HTWKVision upperVision(upperConfig, pool);
    HTWKVision lowerVision(lowerConfig, pool);

    uint8_t *img;

    bf::path startPath(path);
    for (auto it = bf::recursive_directory_iterator(startPath); it != end; ++it) {
        std::string file = it->path().string();
        std::string filename = it->path().filename().string();
        if (!bf::is_regular_file(*it) || it->path().extension() != ".png" || whitelist.count(filename) == 0) {
            continue;
        }

        PngMetadata metadata;
        std::cout << "Load " << filename << std::endl;
        img = loadFile(file, metadata);

        bool isUpper = (filename.find("_U.") != std::string::npos);
        HTWKVision &vision = isUpper ? upperVision : lowerVision;
        CamPose cam_pose = getCamPoseFromMetadata(metadata);
        vision.proceed(img, cam_pose);

        if (!debugPath.empty()) {
            writeDebugFiles(debugPath + "/" + filename, STD_WIDTH, STD_HEIGHT, img, upperVision, cam_pose);
        }

        free(img);
    }
}

std::set<std::string> readWhitelist(std::string whitelistFile) {
    std::set<std::string> whitelist;

    ifstream file(whitelistFile);
    string line;
    while (getline(file, line)) {
        whitelist.insert(line);
    }

    return whitelist;
}

int main(int argc, char **argv) {
    bpo::variables_map varmap;
    handleProgramArguments(varmap, argc, argv);

    const int processingCount = max(1, varmap["cycles"].as<int>());
    const bool writeTime = varmap["writeTimeFile"].as<bool>();
    string writeDebugFiles = varmap["resultImages"].as<string>();

    std::set<std::string> whitelist;

    if (varmap.count("whitelist")) {
        whitelist = readWhitelist(varmap["whitelist"].as<std::string>());
    }
    ThreadPool pool("Vision", 1);

    if (varmap.count("image")) {
        std::string filename = varmap["image"].as<std::string>();
        processOneFile(filename, processingCount, writeDebugFiles, &pool);
    } else {
        std::string path = varmap["dir"].as<std::string>();
        if (whitelist.empty()) {
            processDirectory(path, processingCount, writeDebugFiles, writeTime, &pool);
        } else {
            processDirectoryWithWhitelist(path, whitelist, writeDebugFiles, &pool);
        }
    }
}
