#include "hypotheses_generator_scanlines.h"

#include <cstring>
#include <emmintrin.h>
#include <xmmintrin.h>

#include <projectionutil.h>

#include <cassert>

namespace htwk {

const float HypothesesGeneratorScanlines::MIN_OBJECT_RADIUS_NORM = 8.16f;
const int HypothesesGeneratorScanlines::blockSize=BLOCK_SIZE/IntegralImage::INTEGRAL_SCALE;

HypothesesGeneratorScanlines::HypothesesGeneratorScanlines(int width, int height, int8_t *lutCb, int8_t *lutCr, const HtwkVisionConfig &config)
    : BaseDetector(width, height, lutCb, lutCr)
    , MAX_NUM_HYPOTHESES(config.hypothesisGeneratorMaxHypothesisCount)
    , rWidth(IntegralImage::iWidth/RATING_SCALE)
    , rHeight(IntegralImage::iHeight/RATING_SCALE)
    , numBlockX(IntegralImage::iWidth/blockSize)
    , numBlockY(IntegralImage::iHeight/blockSize)
    , blockObjectRadius(numBlockX*numBlockY)
    , lineSpace(2)
    , svMax(rWidth/lineSpace-1)
    , shMax(rHeight/lineSpace-1)
{
    if(posix_memalign((void**)&ratingImg, 16, sizeof(*ratingImg)*rWidth*rHeight) != 0) {
        fprintf(stderr, "HypothesesGeneratorScanlines: Error allocating aligned memory! reason: %s\n", strerror(errno));
        exit(1);
    }

    histCnt=0;
    histData=std::array<int,256>();
    scale=IntegralImage::INTEGRAL_SCALE*RATING_SCALE;
    scanVertical = new RatingScanline[svMax];
    scanHorizontal = new RatingScanline[shMax];

    for(float a=0;a<M_PI-0.0001;a+=M_PI/8){
        vec.emplace_back(sinf(a),cosf(a));
    }

    assert(vec.size() == numAngles);
}

HypothesesGeneratorScanlines::~HypothesesGeneratorScanlines() {
    free(ratingImg);
    delete [] scanVertical;
    delete [] scanHorizontal;
}

void HypothesesGeneratorScanlines::proceed(uint8_t* img, const int * const fieldBorder, float camPitch, float camRoll, IntegralImage* _integralImg) {
    rollRad=camRoll;
    pitchRad=camPitch;
    integralImg = _integralImg;

    calculateBlockRadii();

    createRatingImg(fieldBorder);
    hypoList.clear();
    scanRatingImg(fieldBorder);

    const Line horizon = ProjectionUtil::getHorizon(camPitch, camRoll);
    analyseScanlinesHorizontal(horizon);
    analyseScanlinesVertical(horizon);

    std::sort(hypoList.begin(), hypoList.end(), [](const ObjectHypothesis& a, const ObjectHypothesis& b){return a.rating > b.rating;});
    const int maxElementsInList = 60;
    if(hypoList.size() > maxElementsInList)
        hypoList.resize(maxElementsInList);
    for (ObjectHypothesis& hyp : hypoList){
        hyp.r=ProjectionUtil::getObjectRadius(hyp.x,hyp.y,pitchRad,rollRad,BALL_SIZE);
    }

    improveHypothesesRatings(hypoList);
    improvePositionAccuracy(hypoList);
    selectBestHypotheses(hypoList);
}

/**
 * improves position from 2d points with subpixel accuracy using the ratingImg and scales the points back to original resolution
 */
void HypothesesGeneratorScanlines::improvePositionAccuracy(std::vector<ObjectHypothesis>& selectedList){
    for(ObjectHypothesis& hyp : selectedList){
        point_2d pos=getSubpixelPosition2(ratingImg, hyp.x/IntegralImage::INTEGRAL_SCALE/RATING_SCALE,
                                                     hyp.y/IntegralImage::INTEGRAL_SCALE/RATING_SCALE, rWidth, rHeight);
        hyp.x=(int)(pos.x*IntegralImage::INTEGRAL_SCALE*RATING_SCALE);
        hyp.y=(int)(pos.y*IntegralImage::INTEGRAL_SCALE*RATING_SCALE+hyp.r*0.25f);
    }
}

/**
 * searches object hypotheses:
 * - blurring the input channel with box blur (integral image optimized) using predicted object size
 * - using a 3 point triangle pattern for simplified Wavelet-Filter
 * - sort hypotheses by rating-value
 * - select 30 best hypotheses for further processing (higher number is statistically not needed)
 */
void HypothesesGeneratorScanlines::createRatingImg(const int* const border){
    std::fill(histData.begin(),histData.end(),0);
    histCnt=0;

    for(int by=0;by<numBlockY;by++){
        for(int bx=0;bx<numBlockX;bx++){
            int blockAddr=bx+by*numBlockX;
            int px=bx*blockSize;
            int py=by*blockSize;
            float objectRadius=blockObjectRadius[blockAddr];
            calculateBlockBlur(objectRadius,px,py,border);
        }
    }

    calcOtsuThreshold();
}

/**
 * sorts hypotheses by rating value and selects best non-overlapping ones
 * @param maxList
 */
void HypothesesGeneratorScanlines::selectBestHypotheses(std::vector<ObjectHypothesis>& maxList) {
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
void HypothesesGeneratorScanlines::improveHypothesesRatings(std::vector<ObjectHypothesis>& selectedList){
    for(ObjectHypothesis& hyp : selectedList){
        int blockAddr=hyp.x/BLOCK_SIZE+hyp.y/BLOCK_SIZE*numBlockX;
        float objectRadius=blockObjectRadius[blockAddr];
        int r=std::max(2,(int)(objectRadius*OBJECT_RADIUS_SCALE/IntegralImage::INTEGRAL_SCALE/RATING_SCALE));
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
bool HypothesesGeneratorScanlines::containsObject(const std::vector<ObjectHypothesis>& maxList, const ObjectHypothesis& hyp){
    int maxDistSq=(int)(hyp.r*hyp.r*OBJECT_RADIUS_SCALE*OBJECT_RADIUS_SCALE);
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

void HypothesesGeneratorScanlines::calculateBlockBlurBorder(const float objectRadius, const int px, const int py){
    int r=(int)(objectRadius*0.6f/IntegralImage::INTEGRAL_SCALE);

    for(int y=py+RATING_SCALE/2;y<py+blockSize;y+=RATING_SCALE){
        for(int x=px+RATING_SCALE/2;x<px+blockSize;x+=RATING_SCALE){
            int px1=std::max(0,x-r);
            int py1=std::max(0,y-r);
            int px2=std::min(IntegralImage::iWidth-1,x+r);
            int py2=std::min(IntegralImage::iHeight-1,y+r);
            int iInner=integralImg->getIntegralValue(px1,py1,px2,py2);
            int areaInner=getArea(px1,py1,px2,py2);
            int rating=3*(iInner/areaInner);
            ratingImg[x/RATING_SCALE+y/RATING_SCALE*rWidth]=rating;
        }
    }
}

#define _mm_shuffle2_epi32(a, b, imm) \
    _mm_castps_si128(_mm_shuffle_ps(_mm_castsi128_ps(a), _mm_castsi128_ps(b), (imm)))

void HypothesesGeneratorScanlines::calculateBlockBlurSSERatingScale2(const float objectRadius, const int px, const int py){
    int r=(int)(objectRadius*0.6f/IntegralImage::INTEGRAL_SCALE);

    int iWidth = IntegralImage::iWidth;
    const int* iImg = integralImg->getIntegralImg();

    for(int y=py+RATING_SCALE/2;y<py+blockSize;y+=RATING_SCALE){
        int py1=std::max(0,y-r);
        int py2=std::min(IntegralImage::iHeight-1,y+r);
        __m128 areaInner = _mm_set1_ps(3.f/(2.f*r*(py2-py1)));
        for(int x=px+RATING_SCALE/2;x<px+blockSize;x+=8){
            int px1=x-r;
            int px2=x+r;
            int px3=x+4-r;
            int px4=x+4+r;
            __m128i p11 = _mm_loadu_si128((__m128i*)&iImg[px1+py1*iWidth]);
            __m128i p31 = _mm_loadu_si128((__m128i*)&iImg[px3+py1*iWidth]);
            __m128i p131 = _mm_shuffle2_epi32(p11,p31,_MM_SHUFFLE(2,0,2,0));
            __m128i p12 = _mm_loadu_si128((__m128i*)&iImg[px1+py2*iWidth]);
            __m128i p32 = _mm_loadu_si128((__m128i*)&iImg[px3+py2*iWidth]);
            __m128i p132 = _mm_shuffle2_epi32(p12,p32,_MM_SHUFFLE(2,0,2,0));
            __m128i p21 = _mm_loadu_si128((__m128i*)&iImg[px2+py1*iWidth]);
            __m128i p41 = _mm_loadu_si128((__m128i*)&iImg[px4+py1*iWidth]);
            __m128i p241 = _mm_shuffle2_epi32(p21,p41,_MM_SHUFFLE(2,0,2,0));
            __m128i p22 = _mm_loadu_si128((__m128i*)&iImg[px2+py2*iWidth]);
            __m128i p42 = _mm_loadu_si128((__m128i*)&iImg[px4+py2*iWidth]);
            __m128i p242 = _mm_shuffle2_epi32(p22,p42,_MM_SHUFFLE(2,0,2,0));
            __m128 inner = _mm_cvtepi32_ps(_mm_sub_epi32(_mm_sub_epi32(p242,p132),_mm_sub_epi32(p241,p131)));
            __m128i rating = _mm_cvtps_epi32(_mm_mul_ps(inner,areaInner));
            _mm_store_si128((__m128i*)&ratingImg[x/RATING_SCALE+y/RATING_SCALE*rWidth],rating);
        }
    }
}

void HypothesesGeneratorScanlines::calculateBlockBlurSSERatingScale4(const float objectRadius, const int px, const int py){
    int r=(int)(objectRadius*0.6f/IntegralImage::INTEGRAL_SCALE);

    int iWidth = IntegralImage::iWidth;
    const int* iImg = integralImg->getIntegralImg();

    for(int y=py+RATING_SCALE/2;y<py+blockSize;y+=RATING_SCALE){
        int py1=std::max(0,y-r);
        int py2=std::min(IntegralImage::iHeight-1,y+r);
        __m128 areaInner = _mm_set1_ps(3.f/(2.f*r*(py2-py1)));
        for(int x=px+RATING_SCALE/2;x<px+blockSize;x+=2*RATING_SCALE){
            int px1=x-r;
            int px2=x+r;
            int px3=x+2*RATING_SCALE-r;
            int px4=x+2*RATING_SCALE+r;
            __m128i p11 = _mm_loadu_si128((__m128i*)&iImg[px1+py1*iWidth]);
            __m128i p31 = _mm_loadu_si128((__m128i*)&iImg[px3+py1*iWidth]);
            __m128i p131 = _mm_shuffle2_epi32(p11,p31,_MM_SHUFFLE(2,0,2,0));
            __m128i p12 = _mm_loadu_si128((__m128i*)&iImg[px1+py2*iWidth]);
            __m128i p32 = _mm_loadu_si128((__m128i*)&iImg[px3+py2*iWidth]);
            __m128i p132 = _mm_shuffle2_epi32(p12,p32,_MM_SHUFFLE(2,0,2,0));
            __m128i p21 = _mm_loadu_si128((__m128i*)&iImg[px2+py1*iWidth]);
            __m128i p41 = _mm_loadu_si128((__m128i*)&iImg[px4+py1*iWidth]);
            __m128i p241 = _mm_shuffle2_epi32(p21,p41,_MM_SHUFFLE(2,0,2,0));
            __m128i p22 = _mm_loadu_si128((__m128i*)&iImg[px2+py2*iWidth]);
            __m128i p42 = _mm_loadu_si128((__m128i*)&iImg[px4+py2*iWidth]);
            __m128i p242 = _mm_shuffle2_epi32(p22,p42,_MM_SHUFFLE(2,0,2,0));
            __m128 inner = _mm_cvtepi32_ps(_mm_sub_epi32(_mm_sub_epi32(p242,p132),_mm_sub_epi32(p241,p131)));
            __m128i rating = _mm_cvtps_epi32(_mm_mul_ps(inner,areaInner));
            _mm_storeu_si128((__m128i*)&ratingImg[x/RATING_SCALE+y/RATING_SCALE*rWidth],rating);
        }
    }
}


/**
 * adaptive box blur filter for one image block with given object radius as kernel size
  */
void HypothesesGeneratorScanlines::calculateBlockBlur(const float objectRadius, const int px, const int py, const int* const border){
    int r=(int)(objectRadius*0.6f/IntegralImage::INTEGRAL_SCALE);

    if(RATING_SCALE == 2) {
        if(px+RATING_SCALE/2-r>=0 && px+blockSize+2*RATING_SCALE+r<IntegralImage::iWidth) {
            calculateBlockBlurSSERatingScale2(objectRadius, px, py);
        } else {
            calculateBlockBlurBorder(objectRadius, px, py);
        }
    } else if(RATING_SCALE == 4) {
        if(px+RATING_SCALE/2-r>=0 && px+blockSize+2*RATING_SCALE+r<IntegralImage::iWidth) {
            calculateBlockBlurSSERatingScale4(objectRadius, px, py);
        } else {
            calculateBlockBlurBorder(objectRadius, px, py);
        }
    } else {
        int r=(int)(objectRadius*0.6f/IntegralImage::INTEGRAL_SCALE);
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
                int rating=3*(iInner/areaInner);
                ratingImg[x/RATING_SCALE+y/RATING_SCALE*rWidth]=rating;
                if (border[x*IntegralImage::INTEGRAL_SCALE] < y*IntegralImage::INTEGRAL_SCALE){
                    int h=clamp(0,rating/12,255);
                    histData[h]++;
                    histCnt++;
                }
            }
        }
    }

    // Fill histogram for binarization threshold estimation.
    int mx = px + blockSize / 2;
    int my = py + blockSize / 2;
    if (border[mx*IntegralImage::INTEGRAL_SCALE] < my*IntegralImage::INTEGRAL_SCALE){
        int h=clamp(0,ratingImg[mx/RATING_SCALE+my/RATING_SCALE*rWidth]/12,255);
        histData[h]++;
        histCnt++;
    }
}

void HypothesesGeneratorScanlines::calcOtsuThreshold(){
    int total=histCnt;
    float sum = 0;
    for (int t=0;t<256;t++) sum += t * histData[t];

    float sumB = 0;
    int wB = 0;
    int wF = 0;

    float varMax = 0;
    threshold = 0;

    for(int t=0;t<256;t++) {
        wB += histData[t];               // Weight Background
        if(wB == 0)
            continue;

        wF = total - wB;                 // Weight Foreground
        if(wF == 0)
            break;

        sumB += (float)(t*histData[t]);

        float mB = sumB / wB;            // Mean Background
        float mF = (sum - sumB) / wF;    // Mean Foreground

        // Calculate Between Class Variance
        float varBetween = (float)wB*(float)wF*(mB-mF)*(mB-mF);

        // Check if new maximum found
        if (varBetween > varMax) {
            varMax = varBetween;
            threshold = t;
        }
    }
    threshold*=12;
}

/**
 * calculates the object radii for every block in the image
 */
void HypothesesGeneratorScanlines::calculateBlockRadii(){
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
void HypothesesGeneratorScanlines::createIntegralImage(int* dataY, int* dataCr, int* iImage, int iWidth, int iHeight){
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
point_2d HypothesesGeneratorScanlines::getSubpixelPosition2(const int* data, int x, int y, const int width, const int height){
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

void HypothesesGeneratorScanlines::scanRatingImg(const int* border){
    int x=0;
    for (int i=0;i<svMax;i++){
        x+=lineSpace;
        int startY=(int)(border[x*scale]/scale);
        scanVertical[i].edgesX[0]=x;
        scanVertical[i].edgesY[0]=startY;
        scanVertical[i].regionsIsObj[0]=false;
        int cnt=1;
        for (int y=startY+1;y<rHeight;y++){
            if (ratingImg[x+y*rWidth]<threshold){
                scanVertical[i].regionsIsObj[cnt]=false;
                for (y++;y<rHeight-lineSpace;y++){
                    if (threshold < ratingImg[x+y*rWidth])
                        break;
                }
            }else{
                scanVertical[i].regionsIsObj[cnt]=true;
                for (y++;y<rHeight-lineSpace;y++){
                    if (ratingImg[x+y*rWidth] < threshold)
                        break;
                }

            }
            y--;
            scanVertical[i].edgesX[cnt]=x;
            scanVertical[i].edgesY[cnt]=y;
            cnt++;
            if (cnt == maxEdges)
                break;
        }
        scanVertical[i].edgeCnt=cnt;
    }

    int y=0;
    for (int i=0;i<shMax;i++){
        y+=lineSpace;
        int StartX=lineSpace;
        for (int x=lineSpace;x<rWidth;x++){
            if (y > border[x*scale]/scale){
                StartX=x;
                break;
            }
        }
        scanHorizontal[i].edgesX[0]=StartX;
        scanHorizontal[i].regionsIsObj[0]=false;
        scanHorizontal[i].edgesY[0]=y;
        int cnt=1;

        for (int x=StartX;x<rWidth;x++){
            if (y < border[x*scale]/scale) continue;
            if (ratingImg[x+y*rWidth]<threshold){
                scanHorizontal[i].regionsIsObj[cnt]=false;
                for (x++;x<rWidth-lineSpace;x++){
                    if (threshold < ratingImg[x+y*rWidth])
                        break;
                    if (y < border[x*scale]/scale)
                        break;
                }
            }else{
                scanHorizontal[i].regionsIsObj[cnt]=true;
                for (x++;x<rWidth-lineSpace;x++){
                    if (ratingImg[x+y*rWidth] < threshold)
                        break;
                    if (y < border[x*scale]/scale)
                        break;
                }
            }
            x--;
            scanHorizontal[i].edgesX[cnt]=x;
            scanHorizontal[i].edgesY[cnt]=y;
            cnt++;
            if (cnt == maxEdges)
                break;
        }
        scanHorizontal[i].edgeCnt=cnt;
    }
}

void HypothesesGeneratorScanlines::analyseScanlinesHorizontal(const htwk::Line& horizon){
    for (int i=0;i<shMax;i++){
        for(int j=0;j<scanHorizontal[i].edgeCnt;j++){
            if(!scanHorizontal[i].regionsIsObj[j]) continue;
            int left=scanHorizontal[i].edgesX[j-1];
            int right=scanHorizontal[i].edgesX[j];
            int radius=((right-left+1)/2);
            int top=std::max(0,(scanHorizontal[i].edgesY[j]-radius));
            int bottom=std::min(rHeight-1,(scanHorizontal[i].edgesY[j]+radius));

            if (radius==0 || radius > rWidth*0.2f) continue;
            float centerX=left+radius;
            float centerY=scanHorizontal[i].edgesY[j];

            if(!PRUNE_HYPOTHESIS_OVER_HORIZON ||
                    ProjectionUtil::belowHorizon(htwk::point_2d{centerX, centerY}*
                                                 RATING_SCALE*IntegralImage::INTEGRAL_SCALE, horizon)) {
    //            int rating=calcRatingCross(left,right,top,bottom,centerX,centerY);
                int rating=calcPoints(left,right,top,bottom,centerX,centerY);
    //            int rating=calcRatingIntegral(left,right,top,bottom,right-left);
    //            int rating=calcCrossPoints(left,right,top,bottom,centerX,centerY);
                hypoList.emplace_back(centerX*scale,centerY*scale,radius,rating);
            }
        }
    }
}

void HypothesesGeneratorScanlines::analyseScanlinesVertical(const htwk::Line& horizon){
    for (int i=0;i<svMax;i++){
        for(int j=0;j<scanVertical[i].edgeCnt;j++){
            if(!scanVertical[i].regionsIsObj[j]) continue;

            int top=scanVertical[i].edgesY[j-1];
            int bottom=scanVertical[i].edgesY[j];
            int radius=((bottom-top+1)/2);
            int left=std::max(0,scanVertical[i].edgesX[j]-radius);
            int right=std::min(rWidth-1,scanVertical[i].edgesX[j]+radius);

            if (radius==0 || radius > rHeight*0.2f) continue;
            float centerX=scanVertical[i].edgesX[j];
            float centerY=top+radius;

            if(!PRUNE_HYPOTHESIS_OVER_HORIZON ||
                    ProjectionUtil::belowHorizon(htwk::point_2d{centerX, centerY}*
                                                 RATING_SCALE*IntegralImage::INTEGRAL_SCALE, horizon)) {
    //            int rating=calcRatingCross(left,right,top,bottom,centerX,centerY);
                int rating=calcPoints(left,right,top,bottom,centerX,centerY);
    //            int rating=calcRatingIntegral(left,right,top,bottom,bottom-top);
    //            int rating=calcCrossPoints(left,right,top,bottom,centerX,centerY);
                hypoList.emplace_back(centerX*scale,centerY*scale,radius,rating);
            }
        }
    }
}

int HypothesesGeneratorScanlines::calcRatingIntegral(int left, int right, int top, int bottom, int size){
    if (size<3) return 0;
    int r=(size>>2)+1;
    int inner=integralImg->getIntegralValue(left+r,top+r,right-r,bottom-r);
    int outer=integralImg->getIntegralValue(left,top,right,bottom);
    int innerArea=getArea(left+r,top+r,right-r,bottom-r);
    if (innerArea <= 0 || inner <= 0) return 0;
    int outerArea=getArea(left,top,right,bottom);
    return std::max(0,3*(inner/innerArea-outer/outerArea));
//    if (size < 2) return 0;
//    int x = integralImg->getIntegralValue(left,top,right,bottom);
//    printf("integral: %d\n",x);
//    printf("%d-%d = %d -> %d\n",right,left,size,size*size);
//    printf("== %d\n",x/(size*size));
//    return (int)(x/(size*size));
}

int HypothesesGeneratorScanlines::calcRatingCross(int left, int right, int top, int bottom, int centerX, int centerY){
    int cnt=0;
    int rating=0;
    for (int x=left;x<=right;x++){
        cnt++;
        rating+=ratingImg[x+centerY*rWidth];
    }
    for (int y=top;y<=bottom;y++){
        cnt++;
        rating+=ratingImg[centerX+y*rWidth];
    }
    return cnt>0?rating/cnt:rating;
}

int HypothesesGeneratorScanlines::calcPoints(int left, int right, int top, int bottom, int centerX, int centerY){
    int rating=ratingImg[centerX+centerY*rWidth];
    int rOut=ratingImg[left+top*rWidth]
            +ratingImg[left+bottom*rWidth]
            +ratingImg[right+top*rWidth]
            +ratingImg[right+bottom*rWidth];
    rOut>>=2;

    return rating-rOut;
}

int HypothesesGeneratorScanlines::calcCrossPoints(int left, int right, int top, int bottom, int centerX, int centerY){
    int rating=calcRatingCross(left,right,top,bottom,centerX,centerY);
    int rOut=ratingImg[left+bottom*rWidth]
            +ratingImg[centerX+bottom*rWidth]
            +ratingImg[right+bottom*rWidth];
    rOut/=3;

    return std::max(0,(rating-rOut));
}



} // htwk
