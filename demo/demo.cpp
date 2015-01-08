#include <chrono>
#include <iostream>
#include <stdint.h>
#include <vector>

#include <htwk_vision.h>
#include <lodepng/lodepng.h>

using namespace htwk;
using namespace std::chrono;

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
                fcd->setY(img,x,y,0);
            }
        }
    }
    std::vector<uint8_t> outRGBA(width * height * 4, 255);
    yuv422ToRgba(outRGBA, img, width, height);
    lodepng::encode("fieldcolordetector.png", outRGBA, width, height);
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
            rc->setY(img,px,py,255);
        }
        rc->setY(img,ls->x,ls->y,0);
    }
    std::vector<uint8_t> outRGBA(width * height * 4, 255);
    yuv422ToRgba(outRGBA, img, width, height);
    lodepng::encode("regionclassifier.png", outRGBA, width, height);
}

void drawFieldBorder(const uint8_t * const orig_img, FieldDetector *fd, RegionClassifier *rc, int width, int height) {
    uint8_t *img = (uint8_t*) malloc(sizeof(uint8_t) * width * height * 2);
    memcpy(img, orig_img, sizeof(uint8_t) * width * height * 2);
    const int *fieldBorder = fd->getConvexFieldBorder();
    for(int x=0;x<width;x++){
        int y=fieldBorder[x];
        if(y<0||y>=height)continue;
        rc->setY(img,x,y,255);
    }
    std::vector<uint8_t> outRGBA(width * height * 4, 255);
    yuv422ToRgba(outRGBA, img, width, height);
    lodepng::encode("fielddetector.png", outRGBA, width, height);
}

void drawLineEdge(uint8_t *img, const LineEdge &ls, int c, LineDetector *ld, int width, int height){
    for(float d=0;d<=1;d+=0.001){
        int px=(int)round(ls.px1*(1-d)+ls.px2*d);
        int py=(int)round(ls.py1*(1-d)+ls.py2*d);
        if(px<0||py<0||px>=width||py>=height)continue;
        ld->setY(img,px,py,c);
    }
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
}

void drawGoalPosts(const uint8_t * const orig_img, GoalDetector *gd, int width, int height){
    uint8_t *img = (uint8_t*) malloc(sizeof(uint8_t) * width * height * 2);
    memcpy(img, orig_img, sizeof(uint8_t) * width * height * 2);
    for(const GoalPost &gp : gd->getGoalPosts()){
        for(int ny=gp.height[0];ny<gp.height[1];ny++){
            int nx=(int)(2*((gp.lineL[0]+gp.lineR[0])*(ny-gp.height[0])/2+(gp.lineL[1]+gp.lineR[1])/2));
            if(nx<0||nx>=width||ny<0||ny>=height)break;
            gd->setY(img,nx,ny,255);
        }
        for(int dx=-8;dx<=8;dx++){
            int px=gp.x+dx;
            if(px<0||px>=width||gp.y<0||gp.y>=height)break;
            gd->setY(img,px,(int)gp.y,255);
        }
    }
    std::vector<uint8_t> outRGBA(width * height * 4, 255);
    yuv422ToRgba(outRGBA, img, width, height);
    lodepng::encode("goaldetector.png", outRGBA, width, height);
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
            bd->setY(img,nx,ny,255);
        }
    }
    std::vector<uint8_t> outRGBA(width * height * 4, 255);
    yuv422ToRgba(outRGBA, img, width, height);
    lodepng::encode("balldetector.png", outRGBA, width, height);
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
                    rc->setY(img,nx,ny,0);
                }
            }
            lastX=px;
            lastY=py;
        }
    }
    std::vector<uint8_t> outRGBA(width * height * 4, 255);
    yuv422ToRgba(outRGBA, img, width, height);
    lodepng::encode("ellipsefitter.png", outRGBA, width, height);
}

void drawFeet(const uint8_t * const orig_img, FeetDetector *fd, int width, int height){
    uint8_t *img = (uint8_t*) malloc(sizeof(uint8_t) * width * height * 2);
    memcpy(img, orig_img, sizeof(uint8_t) * width * height * 2);
    int px=fd->getBase().x;
    if(px>=0&&px<width){
        for(int y=0;y<height;y++){
            fd->setY(img,px,y,255);
        }
    }
    int py=fd->getBase().y;
    if(py>=0&&py<height){
        for(int x=0;x<width;x++){
            fd->setY(img,x,py,255);
        }
    }
    std::vector<uint8_t> outRGBA(width * height * 4, 255);
    yuv422ToRgba(outRGBA, img, width, height);
    lodepng::encode("feetdetector.png", outRGBA, width, height);
}

int main(int argc, char **argv) {
  uint32_t width, height;
  std::vector<uint8_t> imageRGBA;
  uint32_t error = lodepng::decode(imageRGBA, width, height, argv[1]);

  //if there's an error, display it
  if(error) {
    std::cout << "decoder error " << error << ": " << lodepng_error_text(error) << std::endl;
    return error;
  }
  uint8_t *imageYUV422 = (uint8_t *) malloc(sizeof(uint8_t) * 2 * width * height);
  rgbaToYuv422(imageYUV422, imageRGBA, width, height);
  HTWKVision vision(width, height);
  high_resolution_clock::time_point t1 = high_resolution_clock::now();
//  for (int i=0;i<1000;i++)
  vision.proceed(imageYUV422, false, true);
  high_resolution_clock::time_point t2 = high_resolution_clock::now();
  duration<double> time_span = duration_cast<duration<double>>(t2 - t1);
  std::cout << "time: " << time_span.count() * 1000 << "ms\n";
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
}
