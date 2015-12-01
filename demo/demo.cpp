#include "htwk_vision.h"

#include <algorithm>
#include <chrono>
#include <deque>
#include <iostream>
#include <stdint.h>
#include <string>
#include <vector>

#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>

#include <lodepng/lodepng.h>
#include <valgrind/callgrind.h>

using namespace htwk;
using namespace std::chrono;
namespace bpo = boost::program_options;
namespace bf  = boost::filesystem;

#define STD_WIDTH  640
#define STD_HEIGHT 480

inline void setY(uint8_t* const img, const uint32_t width, const int32_t x, int32_t y, const uint8_t c) {
    img[(x+y*width)<<1]=c;
}

void rgbaToYuv422(uint8_t *out, const std::vector<uint8_t> &in, int width, int height) {
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            int r = in[x * 4 + y * width * 4];
            int g = in[1 + x * 4 + y * width * 4];
            int b = in[2 + x * 4 + y * width * 4];
            out[x * 2 + y * width * 2] = (uint8_t) (0.299 * r + 0.587 * g + 0.114 * b);
            if (x % 2 == 0) {
                out[1 + x * 2 + y * width * 2] =
                        (uint8_t) (-0.169 * r - 0.331 * g + 0.499 * b + 128);
            } else {
                out[1 + x * 2 + y * width * 2] =
                        (uint8_t) (0.498 * r - 0.419 * g - 0.0813 * b + 128);
            }
        }
    }
}

int clip(double d){
    if(d<0)d=0;
    if(d>255)d=255;
    return (int)d;
}

void yuv422ToRgba(std::vector<uint8_t> &out, const uint8_t *const in, int width, int height) {
    for(int py=0;py<height;py++){
        int cbLast=in[(0+py*width)*2+1]&255;
        int crLast=in[(0+py*width)*2+3]&255;
        for(int px=0;px<width;px++){
            int y=in[(px+py*width)*2]&255;
            if((px&1)==0){
                cbLast=in[(px+py*width)*2+1]&255;
            }else{
                crLast=in[(px+py*width)*2+1]&255;
            }
            int cb=cbLast;
            int cr=crLast;
            out[px * 4 + py * width * 4] = clip(y+1.402*(cr-128)+2);
            out[1 + px * 4 + py * width * 4] = clip(y-0.344*(cb-128)-0.714*(cr-128));
            out[2 + px * 4 + py * width * 4] = clip(y+1.772*(cb-128)+2);
        }
    }
}

void drawFieldColor(const uint8_t *const orig_img, FieldColorDetector *fcd, int width, int height) {
    uint8_t *img = (uint8_t*) malloc(sizeof(uint8_t) * width * height * 2);
    memcpy(img, orig_img, sizeof(uint8_t) * width * height * 2);
    for(int y=0;y<height;y++){
        for(int x=0;x<width;x++){
            int cy=fcd->getY(img,x,y);
            int cb=fcd->getCb(img,x,y);
            int cr=fcd->getCr(img,x,y);
            if(fcd->isGreen(cy,cb,cr)){
                setY(img,width,x,y,0);
            }
        }
    }
    std::vector<uint8_t> outRGBA(width * height * 4, 255);
    yuv422ToRgba(outRGBA, img, width, height);
    lodepng::encode("fieldcolordetector.png", outRGBA, width, height);
    free(img);
}

void drawLineSegments(const uint8_t * const orig_img, RegionClassifier *rc, int width, int height){
    std::vector<LineSegment*> * lineSegments = rc->lineSegments;
    uint8_t *img = (uint8_t*) malloc(sizeof(uint8_t) * width * height * 2);
    memcpy(img, orig_img, sizeof(uint8_t) * width * height * 2);
    for (const LineSegment *ls : *lineSegments){
        if(ls->x<0||ls->x>=width||ls->y<0||ls->y>=height)continue;
        for(float d=0;d<=5;d+=0.1){
            int px=(int)(ls->x+ls->vx*d);
            int py=(int)(ls->y+ls->vy*d);
            if(px<0||py<0||px>=width||py>=height)continue;
            setY(img,width,px,py,255);
        }
        setY(img,width,ls->x,ls->y,0);
    }
    std::vector<uint8_t> outRGBA(width * height * 4, 255);
    yuv422ToRgba(outRGBA, img, width, height);
    lodepng::encode("regionclassifier.png", outRGBA, width, height);
    free(img);
}

void drawFieldBorder(const uint8_t * const orig_img, FieldDetector *fd, RegionClassifier *rc, int width, int height) {
    uint8_t *img = (uint8_t*) malloc(sizeof(uint8_t) * width * height * 2);
    memcpy(img, orig_img, sizeof(uint8_t) * width * height * 2);
    const int *fieldBorder = fd->getConvexFieldBorder();
    for(int x=0;x<width;x++){
        int y=fieldBorder[x];
        if(y<0||y>=height)continue;
        setY(img,width,x,y,255);
    }
    std::vector<uint8_t> outRGBA(width * height * 4, 255);
    yuv422ToRgba(outRGBA, img, width, height);
    lodepng::encode("fielddetector.png", outRGBA, width, height);
    free(img);
}

void drawLineEdge(uint8_t *img, const LineEdge &ls, int c, LineDetector *ld, int width, int height){
    for(float d=0;d<=1;d+=0.001){
        int px=(int)round(ls.px1*(1-d)+ls.px2*d);
        int py=(int)round(ls.py1*(1-d)+ls.py2*d);
        if(px<0||py<0||px>=width||py>=height)continue;
        setY(img,width,px,py,c);
    }
}


void drawRobots(const uint8_t * const orig_img, std::vector<RobotClassifierResult> robots, int width, int height){
    uint8_t *img = (uint8_t*) malloc(sizeof(uint8_t) * width * height * 2);
    memcpy(img, orig_img, sizeof(uint8_t) * width * height * 2);
    for(const RobotClassifierResult &it:robots){
    	printf("prob: %f\n",it.detectionProbability);
    	int p=(int)(255*it.detectionProbability);
    	for(int px=it.rect.xLeft;px<=it.rect.xRight;px++){
    		setY(img,width,px,it.rect.yTop,p);
    		setY(img,width,px,it.rect.yBottom,p);
    	}
    	for(int py=it.rect.yTop;py<=it.rect.yBottom;py++){
			setY(img,width,it.rect.xLeft,py,p);
			setY(img,width,it.rect.xRight,py,p);
		}
    }
    std::vector<uint8_t> outRGBA(width * height * 4, 255);
    yuv422ToRgba(outRGBA, img, width, height);
    lodepng::encode("robotdetector.png", outRGBA, width, height);
    free(img);
}

void drawLines(const uint8_t * const orig_img, LineDetector *ld, int width, int height){
    uint8_t *img = (uint8_t*) malloc(sizeof(uint8_t) * width * height * 2);
    memcpy(img, orig_img, sizeof(uint8_t) * width * height * 2);
    std::vector<LineGroup> linesList=ld->getLineGroups();
    for(const LineGroup &it:linesList){
        drawLineEdge(img,it.lines[0],0, ld, width, height);
        drawLineEdge(img,it.lines[1],0, ld, width, height);
    }
    std::vector<uint8_t> outRGBA(width * height * 4, 255);
    yuv422ToRgba(outRGBA, img, width, height);
    lodepng::encode("linedetector.png", outRGBA, width, height);
    free(img);
}

void drawGoalPosts(const uint8_t * const orig_img, GoalDetector *gd, int width, int height){
    uint8_t *img = (uint8_t*) malloc(sizeof(uint8_t) * width * height * 2);
    memcpy(img, orig_img, sizeof(uint8_t) * width * height * 2);
    for(const GoalPost &gp : gd->getGoalPosts()){
        std::cout << "GoalPost: " << gp.probability << std::endl;

        for(int ny=gp.upperPoint.y;ny<=gp.basePoint.y;ny++){
            double f=(double)(ny-gp.upperPoint.y)/(gp.basePoint.y-gp.upperPoint.y);
            int nx=(int)(gp.upperPoint.x*(1-f)+gp.basePoint.x*f);
            if(nx<0||nx>=width)continue;
            setY(img, width, nx, ny, 255);
        }

        for(int dx=-8;dx<=8;dx++){
            int px=gp.basePoint.x+dx;
            if(px<0||px>=width||gp.basePoint.y<0||gp.basePoint.y>=height)break;
            setY(img,width,px,(int)gp.basePoint.y,255);
        }
    }
    std::vector<uint8_t> outRGBA(width * height * 4, 255);
    yuv422ToRgba(outRGBA, img, width, height);
    lodepng::encode("goaldetector.png", outRGBA, width, height);
    free(img);
}

void drawBall(const uint8_t * const orig_img, BallDetector *bd, int width, int height){
    uint8_t *img = (uint8_t*) malloc(sizeof(uint8_t) * width * height * 2);
    memcpy(img, orig_img, sizeof(uint8_t) * width * height * 2);
    if(bd->isBallFound()){
        int bx=bd->getBallX();
        int by=bd->getBallY();
        int r=bd->getBallRadius();
        for(float a=0;a<M_PI*2;a+=0.01){
            int nx=bx+sin(a)*r;
            int ny=by+cos(a)*r;
            if(nx<0||ny<0||nx>=width||ny>=height)continue;
            setY(img,width,nx,ny,255);
        }
    }
    std::vector<uint8_t> outRGBA(width * height * 4, 255);
    yuv422ToRgba(outRGBA, img, width, height);
    lodepng::encode("balldetector.png", outRGBA, width, height);
    free(img);
}

void drawCircle(const uint8_t * const orig_img, RansacEllipseFitter *raf, RegionClassifier *rc, int width, int height){
    uint8_t *img = (uint8_t*) malloc(sizeof(uint8_t) * width * height * 2);
    memcpy(img, orig_img, sizeof(uint8_t) * width * height * 2);
    if(raf->getEllipse().found){
        float stepWidth=M_PI*2/64;
        float lastX=0;
        float lastY=0;
        for(double a=0;a<=M_PI*2+0.00001;a+=stepWidth){
            float x=sinf(a);
            float y=cosf(a);
            point_2d point;
            point.x=x*raf->getEllipse().ta;
            point.y=y*raf->getEllipse().tb;
            raf->transformPoInv(point, raf->getEllipse().trans, raf->getEllipse().translation);
            float px=point.x;
            float py=point.y;
            if(a>0){
                for(float d=0;d<=1;d+=0.01){
                    int nx=(int)round(px*(1-d)+lastX*d);
                    int ny=(int)round(py*(1-d)+lastY*d);
                    if(nx<0||ny<0||nx>=width||ny>=height)continue;
                    setY(img,width,nx,ny,0);
                }
            }
            lastX=px;
            lastY=py;
        }
    }
    std::vector<uint8_t> outRGBA(width * height * 4, 255);
    yuv422ToRgba(outRGBA, img, width, height);
    lodepng::encode("ellipsefitter.png", outRGBA, width, height);
    free(img);
}

void drawFeet(const uint8_t * const orig_img, FeetDetector *fd, int width, int height){
    uint8_t *img = (uint8_t*) malloc(sizeof(uint8_t) * width * height * 2);
    memcpy(img, orig_img, sizeof(uint8_t) * width * height * 2);
    int px=fd->getBase().x;
    if(px>=0&&px<width){
        for(int y=0;y<height;y++){
            setY(img,width,px,y,255);
        }
    }
    int py=fd->getBase().y;
    if(py>=0&&py<height){
        for(int x=0;x<width;x++){
            setY(img,width,x,py,255);
        }
    }
    std::vector<uint8_t> outRGBA(width * height * 4, 255);
    yuv422ToRgba(outRGBA, img, width, height);
    lodepng::encode("feetdetector.png", outRGBA, width, height);
    free(img);
}

void handleProgramArguments(bpo::variables_map& varmap, int argc, char** argv) {
    bpo::options_description options_all("");
    options_all.add_options()
            ("help,h",                                                    "Print this help")
            ("dir,d",           bpo::value<std::string>(),                "Process a directory of images")
            ("image,i",         bpo::value<std::string>(),                "The image that should be processed")
            ("cycles,c",        bpo::value<int>()->default_value(1),      "How often process a image with the vision.")
            ("resultImages,r",  bpo::value<bool>()->default_value(false), "Write the result images.")
            ("upperCam,u",      bpo::value<bool>()->default_value(false), "Whether the image is from the upper cam.")
            ;

    bpo::store(bpo::parse_command_line(argc, argv, options_all), varmap); //parse and store

    bpo::notify(varmap); // update the varmap

    if(varmap.count("help")) { std::cout << options_all; exit(0); }
    if(varmap.count("image") == 0 && varmap.count("dir") == 0) { std::cout << options_all; exit(0); }
}

uint8_t* loadFile(std::string& filename, uint32_t& width, uint32_t& height) {
    std::vector<uint8_t> imageRGBA;
    uint32_t error = lodepng::decode(imageRGBA, width, height, filename);

    //if there's an error, display it
    if(error) {
        std::cout << "decoder error " << error << ": " << lodepng_error_text(error) << std::endl;
        exit(1);
    }

    uint8_t *imageYUV422 = NULL;
    if(posix_memalign((void**)&imageYUV422, 16, sizeof(uint8_t) * 2 * width * height) != 0) {
        std::cout << "error allocating aligned memory! reason: " << strerror(errno) << std::endl;
        exit(1);
    }


    if(imageYUV422 == NULL) {
        std::cout << "Couldn't allocate yuv memory. Exit now." << std::endl;
        exit(1);
    }

    if(width == STD_WIDTH && height == STD_HEIGHT)
        rgbaToYuv422(imageYUV422, imageRGBA, width, height);

    return imageYUV422;
}

void processOneFile(std::string& filename, int processingCount, bool writeDebugFiles, bool isUpperCam) {
    uint32_t width, height;
    uint8_t* imageYUV422 = loadFile(filename, width, height);

    high_resolution_clock::time_point t1 = high_resolution_clock::now();

    HTWKVision vision(width, height);
    for (int i=0;i<processingCount;i++)
        vision.proceed(imageYUV422, isUpperCam, true);

    high_resolution_clock::time_point t2 = high_resolution_clock::now();
    duration<double> time_span = duration_cast<duration<double>>(t2 - t1);
    std::cout << "time: " << time_span.count() * 1000 << "ms\n";

    if(writeDebugFiles) {
        std::vector<uint8_t> outRGBA(width * height * 4, 255);
        yuv422ToRgba(outRGBA, imageYUV422, width, height);

        lodepng::encode("image.png", outRGBA, width, height);

        drawFieldColor(imageYUV422, vision.fieldColorDetector, width, height);
        drawLineSegments(imageYUV422, vision.regionClassifier, width, height);
        drawFieldBorder(imageYUV422, vision.fieldDetector, vision.regionClassifier, width, height);
        drawLines(imageYUV422, vision.lineDetector, width, height);
        drawGoalPosts(imageYUV422, vision.goalDetector, width, height);
        drawBall(imageYUV422, vision.ballDetector, width, height);
        drawCircle(imageYUV422, vision.ellipseFitter, vision.regionClassifier, width, height);
        drawFeet(imageYUV422, vision.feetDetector, width, height);
        drawRobots(imageYUV422, vision.getRobotClassifierResult(), width, height);
    }

    free(imageYUV422);
}

void processDirectory(bf::path startPath, int processingCount, bool writeDebugFile, bool isUpperCam) {
    std::deque<uint8_t*> images;

    /* We don't want to see all the loading and converting of pngs. We cache the raw data in memory */
    const bf::recursive_directory_iterator end;

    uint32_t width, height;

    for(auto it = bf::recursive_directory_iterator(startPath); it != end; ++it) {
        if(bf::is_regular_file(*it) && it->path().extension() == ".png") {
            std::cout << "Load " << it->path() << std::endl;
            std::string filename = it->path().string();
            uint8_t* yuv422Image = loadFile(filename, width, height);

            if(width != STD_WIDTH && height != STD_HEIGHT) {
                std::cout << "Ignoring file. Has wrong dimensions" << std::endl;
                free(yuv422Image);
            } else {
                images.push_back(yuv422Image);
            }
        }
    }


    std::cout << std::endl << std::endl << "Processing " << images.size() << " images." << std::endl;
    high_resolution_clock::time_point t1 = high_resolution_clock::now();
//    ProfilerStart("vision.prof");

//    CALLGRIND_START_INSTRUMENTATION;
//    CALLGRIND_TOGGLE_COLLECT;

    HTWKVision vision(STD_WIDTH, STD_HEIGHT);
    for (int i=0;i<processingCount;i++) {
        if(i % 10 == 0) {
            std::cout << "Current count: " << i << std::endl;
        }

        std::for_each(images.begin(), images.end(), [&vision, &isUpperCam](uint8_t* img) {
            vision.proceed(img, isUpperCam, true);
        });
    }

//    CALLGRIND_TOGGLE_COLLECT;
//    CALLGRIND_STOP_INSTRUMENTATION;

//    ProfilerStop();
    high_resolution_clock::time_point t2 = high_resolution_clock::now();
    duration<double> time_span = duration_cast<duration<double>>(t2 - t1);
    std::cout << "time: " << time_span.count() * 1000 << "ms\n";

    std::for_each(images.begin(), images.end(), [](uint8_t* e) { free(e); });
}

int main(int argc, char **argv) {
    bpo::variables_map varmap;
    handleProgramArguments(varmap, argc, argv);

    const int  processingCount  = varmap["cycles"].as<int>();
    const bool writeDebugFiles =  varmap["resultImages"].as<bool>();
    const bool isUpperCam =  varmap["upperCam"].as<bool>();

    if(varmap.count("image")) {
        std::string filename = varmap["image"].as<std::string>();
        processOneFile(filename, processingCount, writeDebugFiles, isUpperCam);
    } else {
        bf::path startPath(varmap["dir"].as<std::string>());
        processDirectory(startPath, processingCount, writeDebugFiles, isUpperCam);
    }
}
