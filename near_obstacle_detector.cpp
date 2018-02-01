#include "near_obstacle_detector.h"

#include "field_color_detector.h"

using namespace std;

namespace htwk {

const int NearObstacleDetector::pixelSpacing=8;	//nur jeden n-ten Pixel in x- und y-Richtung scannen
const float NearObstacleDetector::threshold=0.3;
const float NearObstacleDetector::gain=5;

/**
 * detects the yCbCr color of the playing field in the image.
 * saves two histograms with rating values for different color combinations
 */
void NearObstacleDetector::proceed(const uint8_t * const img, FieldColorDetector * const field) {

    int sumLeft=0;
    int sumCenter=0;
    int sumRight=0;
    int cntLeft=1;
    int cntCenter=1;
    int cntRight=1;
    for(int y=pixelSpacing/2;y<height;y+=pixelSpacing){
        for(int x=pixelSpacing/2;x<width;x+=pixelSpacing){
            int cy=getY(img,x,y);
            int cb=getCb(img,x,y);
            int cr=getCr(img,x,y);
            bool isGreen=field->maybeGreen(cy,cb,cr);
            if(x*2+y<600){
                cntLeft++;
                if(!isGreen){
                    sumLeft++;
                }
            }
            if((width-x)*2+y<600){
                cntRight++;
                if(!isGreen){
                    sumRight++;
                }
            }
            if(y<300&&abs(x-width/2)<200){
                cntCenter++;
                if(!isGreen){
                    sumCenter++;
                }
            }
        }
    }

    obstacleL += float(sumLeft)/cntLeft;
    obstacleC += float(sumCenter)/cntCenter;
    obstacleR += float(sumRight)/cntRight;
}

float NearObstacleDetector::getProb(float s){
    return max(0.f,min(1.f,(s-threshold)*gain));
}

float NearObstacleDetector::getObstacleLeft(){
    return getProb(obstacleL);
}
float NearObstacleDetector::getObstacleCenter(){
    return getProb(obstacleC);
}
float NearObstacleDetector::getObstacleRight(){
    return getProb(obstacleR);
}

}
