#include "jersey_color_detector.h"

#include <cmath>
#include <cstring>

#include "ext_math.h"

using namespace ext_math;
using namespace std;

namespace htwk {

JerseyColorDetector::JerseyColorDetector(int _width, int _height, int8_t *lutCb, int8_t *lutCr)
    : BaseDetector(_width, _height, lutCb, lutCr)
{
    this->offsetColorAngle      = -0.5;  //offset for approx. equal colorAngle detection rates for blue and red jerseys
    this->offsetColors          =  -10;  //offset for approx. equal color detection rates for blue and red jerseys
    this->sensitivityColors     =  32;   //sensitivity of color classifier
    this->sensitivityColorAngle	=  1;    //sensitivity of angle classifier
    this->colorAngleBlue        =  2.5;  //blue color angle
    this->colorAngleRed         =  0;    //red color angle
    this->pixelCalibrationRatio =  0.7;  //how much calibration-pixels (in % of pixel sum)
    this->pixelSelectionRatio	=  0.15; //how much color selection pixels (in % of pixel sum)
    this->topRegionHeight 		=  0.3;  //vertical border for top area (in % of region-height)
    this->bottomRegionHeight 	=  0.5;  //vertical border for bottom area (in % of region-height)
    this->gridSize=16;

    memset(histCbTop,0,sizeof(histCbTop));
    memset(histCrTop,0,sizeof(histCrTop));
    memset(histYBottom,0,sizeof(histYBottom));
}

float JerseyColorDetector::isBlueJersey(const uint8_t * const img, RobotRect rect){
    memset(histCbTop,0,sizeof(histCbTop));
    memset(histCrTop,0,sizeof(histCrTop));
    memset(histYBottom,0,sizeof(histYBottom));
    RobotRect topHalf;
    topHalf.xLeft=rect.xLeft;
    topHalf.xRight=rect.xRight;
    topHalf.yTop=rect.yTop;
    topHalf.yBottom=(int)(rect.yTop*topRegionHeight+rect.yBottom*(1-topRegionHeight));

    RobotRect bottomHalf;
    bottomHalf.xLeft=rect.xLeft;
    bottomHalf.xRight=rect.xRight;
    bottomHalf.yTop=(int)(rect.yTop*bottomRegionHeight+rect.yBottom*(1-bottomRegionHeight));
    bottomHalf.yBottom=rect.yBottom;

    setHistogramY(img,bottomHalf,histYBottom);
    int thresY=getThreshold(histYBottom,pixelCalibrationRatio);


    setHistogramCb(img,topHalf,histCbTop);
    int thresCb=getThreshold(histCbTop,pixelSelectionRatio);

    setHistogramCr(img,topHalf,histCrTop);
    int thresCr=getThreshold(histCrTop,pixelSelectionRatio);

    int whiteCb=getAvgMaxValue(img,bottomHalf,thresY,1,0,0,0,1,0);
    int whiteCr=getAvgMaxValue(img,bottomHalf,thresY,1,0,0,0,0,1);
    int jerseyBlueCb=getAvgMaxValue(img,topHalf,thresCb,0,1,0,0,1,0);
    int jerseyRedCr=getAvgMaxValue(img,topHalf,thresCr,0,0,1,0,0,1);

    int calibratedJerseyBlueCb=jerseyBlueCb-whiteCb;
    int calibratedJerseyRedCr=jerseyRedCr-whiteCr;

    float probability=clamp(0.f,0.5f-(float)((offsetColors+calibratedJerseyRedCr-calibratedJerseyBlueCb)/sensitivityColors),1.f);
    return probability;
}

/**
     * gets average value of selected 'data' pixels. Selection is done by thresholding the 'base'-channel
     * @param base selection channel
     * @param data color channel
     * @param rect rectangle of interest
     * @param thres threshold for pixel selection
     * @return
     */

int JerseyColorDetector::getAvgMaxValue(const uint8_t * const img, RobotRect rect, int thres, int ay, int ab, int ar, int by, int bb, int br){
    int cnt=0;
    int avgValue=0;
    int stepSize=max(1,(rect.xRight-rect.xLeft)/gridSize);
    for(int y=rect.yTop;y<=rect.yBottom;y+=stepSize){
        for(int x=rect.xLeft;x<=rect.xRight;x+=stepSize){
            int cy=getY(img,x,y);
            int cb=getCb(img,x,y);
            int cr=getCr(img,x,y);
            if(cy*ay+cb*ab+cr*ar>=thres){
                avgValue+=cy*by+cb*bb+cr*br;
                cnt++;
            }
        }
    }
    if(cnt==0)return 0;
    return avgValue/cnt;
}

/**
     * calculates a threshold that separates 'ratio' percent of the highest pixel values in a given histogram
     * @param hist histogram of pixel values
     * @param ratio ratio for
     * @return
     */
int JerseyColorDetector::getThreshold(const int* const hist, float ratio){
    int sum=0;
    for(int i=0;i<256;i++){
        sum+=hist[i];
    }
    int cnt=0;
    int thres=255;
    for(;thres>=0;thres--){
        cnt+=hist[thres];
        if(cnt>=sum*ratio)break;
    }
    return thres;
}

/**
     * calculates histogram in a specific rectangle
     * @param data pixel values
     * @param rect rectangle of interest
     * @param hist array to write the histogram values
     */
void JerseyColorDetector::setHistogramY(const uint8_t * const img, RobotRect rect, int* hist){
    int stepSize=max(1,(rect.xRight-rect.xLeft)/gridSize);
    for(int y=rect.yTop;y<=rect.yBottom;y+=stepSize){
        for(int x=rect.xLeft;x<=rect.xRight;x+=stepSize){
            int cy=getY(img,x,y);
            hist[cy]++;
        }
    }
}

/**
 * calculates histogram in a specific rectangle
 * @param data pixel values
 * @param rect rectangle of interest
 * @param hist array to write the histogram values
 */
void JerseyColorDetector::setHistogramCb(const uint8_t * const img, RobotRect rect, int* hist){
    int stepSize=max(1,(rect.xRight-rect.xLeft)/gridSize);
    for(int y=rect.yTop;y<=rect.yBottom;y+=stepSize){
        for(int x=rect.xLeft;x<=rect.xRight;x+=stepSize){
            int cb=getCb(img,x,y);
            hist[cb]++;
        }
    }
}

/**
 * calculates histogram in a specific rectangle
 * @param data pixel values
 * @param rect rectangle of interest
 * @param hist array to write the histogram values
 */
void JerseyColorDetector::setHistogramCr(const uint8_t * const img, RobotRect rect, int* hist){
    int stepSize=max(1,(rect.xRight-rect.xLeft)/gridSize);
    for(int y=rect.yTop;y<=rect.yBottom;y+=stepSize){
        for(int x=rect.xLeft;x<=rect.xRight;x+=stepSize){
            int cr=getCr(img,x,y);
            hist[cr]++;
        }
    }
}

}
