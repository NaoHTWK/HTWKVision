#include "hypotheses_generator_blur.h"

#include <algorithm>

#include <projectionutil.h>

#include <cassert>

//#include <htwkoptimizer.h>
//float get(const std::string& what) {
//    return htwk::tools::optimizer::Optimizer::get(what);
//}

namespace htwk {


const float HypothesesGeneratorBlur::MIN_OBJECT_RADIUS_NORM = 8.16f;

HypothesesGeneratorBlur::HypothesesGeneratorBlur(int width, int height, int8_t *lutCb, int8_t *lutCr)
    : BaseDetector(width, height, lutCb, lutCr)
    , rWidth(IntegralImage::iWidth/RATING_SCALE)
    , rHeight(IntegralImage::iHeight/RATING_SCALE)
    , numBlockX(IntegralImage::iWidth/blockSize)
    , numBlockY(IntegralImage::iHeight/blockSize)
    , blockObjectRadius(numBlockX*numBlockY)
{
    adaptiveBlurImg=new int[rWidth*rHeight];
    ratingImg=new int[rWidth*rHeight];

    for(float a=0;a<M_PI-0.0001;a+=M_PI/8){
        vec.emplace_back(sinf(a),cosf(a));
    }

    assert(vec.size() == numAngles);
}

HypothesesGeneratorBlur::~HypothesesGeneratorBlur() {
    delete [] adaptiveBlurImg;
    delete [] ratingImg;
}

void HypothesesGeneratorBlur::proceed(uint8_t* img, const int * const fieldBorder, float camPitch, float camRoll, IntegralImage* _integralImg) {
    rollRad=camRoll;
    pitchRad=camPitch;
    integralImg = _integralImg;
//    createIntegralImage(img.y,img.cr,integralImg,iWidth,iHeight);

    // based on the current pitch and roll!
    calculateBlockRadii();

    hypoList.clear();
    hypoList=searchObjectHypotheses(fieldBorder);
    improveHypothesesRatings(hypoList);
    improvePositionAccuracy(hypoList);
    selectBestHypotheses(hypoList);
}

/**
 * improves position from 2d points with subpixel accuracy using the ratingImg and scales the points back to original resolution
 */
void HypothesesGeneratorBlur::improvePositionAccuracy(std::vector<ObjectHypothesis>& selectedList){
    for(ObjectHypothesis& hyp : selectedList){
        point_2d pos=getSubpixelPosition2(ratingImg, hyp.x/IntegralImage::INTEGRAL_SCALE/RATING_SCALE,
                                                     hyp.y/IntegralImage::INTEGRAL_SCALE/RATING_SCALE, rWidth, rHeight);
        hyp.x=(int)(pos.x*IntegralImage::INTEGRAL_SCALE*RATING_SCALE);
        hyp.y=(int)(pos.y*IntegralImage::INTEGRAL_SCALE*RATING_SCALE+hyp.r*0.25f);
//        hyp.y=(int)(pos.y*IntegralImage::INTEGRAL_SCALE*RATING_SCALE+hyp.r*1.024234f); //get("improvePositionAccuracy"));
    }
}

/**
 * searches object hypotheses:
 * - blurring the input channel with box blur (integral image optimized) using predicted object size
 * - using a 3 point triangle pattern for simplified Wavelet-Filter
 * - sort hypotheses by rating-value
 * - select 30 best hypotheses for further processing (higher number is statistically not needed)
 */
std::vector<ObjectHypothesis> HypothesesGeneratorBlur::searchObjectHypotheses(const int* const border){
    std::vector<ObjectHypothesis> maxList;

    for(int by=0;by<numBlockY;by++){
        for(int bx=0;bx<numBlockX;bx++){
            int blockAddr=bx+by*numBlockX;
            int px=bx*blockSize;
            int py=by*blockSize;
            float objectRadius=blockObjectRadius[blockAddr];
            calculateBlockBlur(objectRadius,px,py);
        }
    }
    for(int by=0;by<numBlockY;by++){
        for(int bx=0;bx<numBlockX;bx++){
            int blockAddr=bx+by*numBlockX;
            int px=bx*blockSize;
            int py=by*blockSize;
            float objectRadius=blockObjectRadius[blockAddr];
            calculateBlockRating(objectRadius,px,py,maxList,border);
        }
    }

    std::sort(maxList.begin(), maxList.end(), [](const ObjectHypothesis& a, const ObjectHypothesis& b){return a.rating > b.rating;});

    const int maxElementsInList = 30;
    if(maxList.size() > maxElementsInList)
        maxList.resize(maxElementsInList);

    return maxList;
}

/**
 * sorts hypotheses by rating value and selects best non-overlapping ones
 * @param maxList
 */
void HypothesesGeneratorBlur::selectBestHypotheses(std::vector<ObjectHypothesis>& maxList) {
    std::vector<ObjectHypothesis> tmp(maxList);
    maxList.clear();

    std::sort(tmp.begin(), tmp.end(), [](const ObjectHypothesis& a, const ObjectHypothesis& b){return a.rating > b.rating;});

    for(ObjectHypothesis& hyp : tmp){
        if(!containsObject(maxList, hyp)){
            maxList.push_back(hyp);
            if(maxList.size()>=MAX_NUM_HYPOTHESES){
                break;
            }
        }
    }
}

/**
 * improves the rating values for the best hypotheses with better circle pattern
 */
void HypothesesGeneratorBlur::improveHypothesesRatings(std::vector<ObjectHypothesis>& selectedList){
    for(ObjectHypothesis& hyp : selectedList){
        int blockAddr=hyp.x/BLOCK_SIZE+hyp.y/BLOCK_SIZE*numBlockX;
        float objectRadius=blockObjectRadius[blockAddr];
//        int r=std::max(2,(int)(objectRadius*1.251029f /*get("OBJECT_RADIUS_SCALE1")*//IntegralImage::INTEGRAL_SCALE/RATING_SCALE));
        int r=std::max(2,(int)(objectRadius*OBJECT_RADIUS_SCALE /*get("OBJECT_RADIUS_SCALE1")*//IntegralImage::INTEGRAL_SCALE/RATING_SCALE));
        int nx=hyp.x/IntegralImage::INTEGRAL_SCALE/RATING_SCALE;
        int ny=hyp.y/IntegralImage::INTEGRAL_SCALE/RATING_SCALE;
        if(nx<0||ny<0||nx>=rWidth||ny>=rHeight)
            continue;
        int rm=ratingImg[nx+ny*rWidth];
        int maxValue=0;
        for(int idx=0;idx<numAngles;idx++){
            float vx=r*vec[idx].x;
            float vy=r*vec[idx].y;
            int px1=(int)(nx+vx+0.5f);
            int py1=(int)(ny+vy+0.5f);
            if(px1<0||py1<0||px1>=rWidth||py1>=rHeight)continue;
            int value1=ratingImg[px1+py1*rWidth];
            int px2=(int)(nx-vx+0.5f);
            int py2=(int)(ny-vy+0.5f);
            if(px2<0||py2<0||px2>=rWidth||py2>=rHeight)continue;
            int value2=ratingImg[px2+py2*rWidth];
            int value=std::min(value1,value2);
            if(value>maxValue){
                maxValue=value;
            }
        }
        hyp.rating=rm-maxValue;
    }
}

/**
 * tests if a new hypotheses overlaps with any of the hypotheses in maxList
 */
bool HypothesesGeneratorBlur::containsObject(const std::vector<ObjectHypothesis>& maxList, const ObjectHypothesis& hyp){
//    int maxDistSq=(int)(hyp.r*hyp.r*0.416255f*0.416255f); //get("OBJECT_RADIUS_SCALE2")*get("OBJECT_RADIUS_SCALE2"));
    int maxDistSq=(int)(hyp.r*hyp.r*OBJECT_RADIUS_SCALE*OBJECT_RADIUS_SCALE); //get("OBJECT_RADIUS_SCALE2")*get("OBJECT_RADIUS_SCALE2"));
    for(const ObjectHypothesis& h : maxList){
        int dx=h.x-hyp.x;
        int dy=h.y-hyp.y;
        float distSq=dx*dx+dy*dy;
        if(distSq<maxDistSq){
            return true;
        }
    }
    return false;
}

/**
 * calculates the rating values for one block by using a 3 point triangle pattern (r1,r2,r3) for simplified Wavelet-Filter
 * rating=3*rCenter-Math.max(r1,Math.max(r2,r3));
 */
void HypothesesGeneratorBlur::calculateBlockRating(const float objectRadius, const int px, const int py, std::vector<ObjectHypothesis>& maxList, const int *border){
//    int r=std::max(1, (int)(objectRadius*get("calculateBlockRating_r")/IntegralImage::INTEGRAL_SCALE/RATING_SCALE));
//    int rDown=std::max(1, (int)(objectRadius*get("calculateBlockRating_rDown")/IntegralImage::INTEGRAL_SCALE/RATING_SCALE));
//    int r=std::max(1, (int)(objectRadius*0.529607f/IntegralImage::INTEGRAL_SCALE/RATING_SCALE));
//    int rDown=std::max(1, (int)(objectRadius*2.933128f/IntegralImage::INTEGRAL_SCALE/RATING_SCALE));
    int r=std::max(1, (int)(objectRadius*1.0f/IntegralImage::INTEGRAL_SCALE/RATING_SCALE));
    int rDown=std::max(1, (int)(objectRadius*1.35f/IntegralImage::INTEGRAL_SCALE/RATING_SCALE));
    int maxRating=0;
    int maxX=0;
    int maxY=0;
    for(int y=py+RATING_SCALE/2;y<py+blockSize;y+=RATING_SCALE){
        for(int x=px+RATING_SCALE/2;x<px+blockSize;x+=RATING_SCALE){
            int nx=x/RATING_SCALE;
            int ny=y/RATING_SCALE;
            if(nx<0||ny<0||nx>=rWidth||ny>=rHeight)
                continue;
            int px1=std::max(0,nx-r);
            int py1=std::max(0,ny-r);
            int px2=std::min(rWidth-1,nx+r);
            int py2=std::max(0,ny-r);
            int px3=std::max(0,nx);
            int py3=std::min(rHeight-1,ny+rDown);
            int rCenter=adaptiveBlurImg[nx+ny*rWidth];
            int r1=adaptiveBlurImg[px1+py1*rWidth];
            int r2=adaptiveBlurImg[px2+py2*rWidth];
            int r3=adaptiveBlurImg[px3+py3*rWidth];
            int rating=3*rCenter-std::max(r1,std::max(r2,r3));
            ratingImg[nx+ny*rWidth]=rating;
            if(rating>maxRating){
                int tx=x*IntegralImage::INTEGRAL_SCALE;
                int ty=y*IntegralImage::INTEGRAL_SCALE;
                if(border[tx]-borderCompensation<ty){
                    maxRating=rating;
                    maxX=tx;
                    maxY=ty;
                };
            }
        }
    }
    if(maxRating>0){
        const float ballRadiusCorrection=0.91f; //get("ballRadiusCorrection");
        maxList.emplace_back(maxX,maxY,std::max(1, (int)(objectRadius*ballRadiusCorrection)),maxRating);
    }
}

/**
 * adaptive box blur filter for one image block with given object radius as kernel size
  */
void HypothesesGeneratorBlur::calculateBlockBlur(const float objectRadius, const int px, const int py){
//    int r=std::max((int)(objectRadius*get("calculateBlockBlur")/IntegralImage::INTEGRAL_SCALE), 1);
//    int r=std::max((int)(objectRadius*0.507689f/IntegralImage::INTEGRAL_SCALE), 1);
    int r=std::max((int)(objectRadius*0.6f/IntegralImage::INTEGRAL_SCALE), 1);
    for(int y=py+RATING_SCALE/2;y<py+blockSize;y+=RATING_SCALE){
        for(int x=px+RATING_SCALE/2;x<px+blockSize;x+=RATING_SCALE){
            if(x<0||y<0||x>=IntegralImage::iWidth||y>=IntegralImage::iHeight)
                continue;
            int px1=std::max(0,x-r);
            int py1=std::max(0,y-r);
            int px2=std::min(IntegralImage::iWidth-1,x+r);
            int py2=std::min(IntegralImage::iHeight-1,y+r);
            int iInner=integralImg->getIntegralValue(px1,py1,px2,py2);
            int areaInner=getArea(px1,py1,px2,py2);
            int rating=iInner/areaInner;
            adaptiveBlurImg[x/RATING_SCALE+y/RATING_SCALE*rWidth]=rating;
        }
    }
}

/**
 * calculates the object radii for every block in the image
 */
void HypothesesGeneratorBlur::calculateBlockRadii(){
    for(int by=0;by<numBlockY;by++){
        for(int bx=0;bx<numBlockX;bx++){
            int blockAddr=bx+by*numBlockX;
            int px=bx*blockSize;
            int py=by*blockSize;
            float objectSize=ProjectionUtil::getObjectRadius(IntegralImage::INTEGRAL_SCALE*(px+blockSize/2),
                                                             IntegralImage::INTEGRAL_SCALE*(py+blockSize/2),
                                                             pitchRad,rollRad,BALL_SIZE);
            blockObjectRadius[blockAddr]=std::max(MIN_OBJECT_RADIUS_NORM, objectSize);
        }
    }
}

/**
 * creates integral image
 */
void HypothesesGeneratorBlur::createIntegralImage(int* dataY, int* dataCr, int* iImage, int iWidth, int iHeight){
    iImage[0]=(dataY[0]+3*dataCr[0]);
    for(int x=1;x<iWidth;x++){
        iImage[x]=iImage[x-1]+(dataY[x*IntegralImage::INTEGRAL_SCALE]+3*dataCr[x*IntegralImage::INTEGRAL_SCALE]);
    }
    for(int y=1;y<iHeight;y++){
        int sum=0;
        for(int x=0;x<iWidth;x++){
            int addr=x+y*iWidth;
            int addrData=x*IntegralImage::INTEGRAL_SCALE+y*IntegralImage::INTEGRAL_SCALE*IntegralImage::INTEGRAL_SCALE*iWidth;
            sum+=(dataY[addrData]+3*dataCr[addrData]);
            iImage[addr]=sum+iImage[addr-iWidth];
        }
    }
}

/**
 * calculates the subpixel position of a given integer position on a rating grid
 */
point_2d HypothesesGeneratorBlur::getSubpixelPosition2(const int* data, int x, int y, const int width, const int height){
    if(x<1)x=1;
    if(y<1)y=1;
    if(x>=width-1)x=width-2;
    if(y>=height-1)y=height-2;
    float dx = (data[x+1+(y)*width] - data[x-1+(y)*width]) / 2.f;
    float dy = (data[x+(y+1)*width] - data[x+(y-1)*width]) / 2.f;
    float dxx = (data[x+1+(y)*width] + data[x-1+(y)*width] - 2 * data[x+(y)*width]);
    float dyy = (data[x+(y+1)*width] + data[x+(y-1)*width] - 2 * data[x+(y)*width]);
    float dxy = (data[x+1+(y+1)*width] - data[x+1+(y-1)*width] - data[x-1+(y+1)*width] + data[x-1+(y-1)*width]) / 4.f;
    float d=dxx*dyy - dxy*dxy;

    if(std::abs(d)<0.0001f)
        return point_2d(x,y);

    float det = 1.f/d;
    float vx=(dyy*dx - dxy*dy) * det;
    float vy=(dxx*dy - dxy*dx) * det;
    if(vx*vx+vy*vy<1*1){
        return point_2d(x-vx+0.5f,y-vy+0.5f);
    }else{
        return point_2d(x+0.5f,y+0.5f);
    }
}

} // htwk
