#include "robot_area_detector.h"

#include <chrono>
#include <cstring>
#include <iostream>

#include <projectionutil.h>

using namespace std::chrono;
namespace htwk {

RobotAreaDetector::RobotAreaDetector(int _width, int _height, int8_t *lutCb, int8_t *lutCr, int scanlineCnt)
    : BaseDetector(_width, _height, lutCb, lutCr)
{
    this->lutCb=lutCb;
    this->lutCr=lutCr;

    this->scanlineCnt=scanlineCnt;

}

void RobotAreaDetector::proceed(Scanline *scanVertical, const int * const fieldborder, float camPitch, float camRoll){

    pitchRad=camPitch;
    rollRad=camRoll;

    hypoList.clear();
    searchRobotHypotheses(scanVertical,fieldborder);

}

void RobotAreaDetector::searchRobotHypotheses(Scanline *scanVertical, const int * const fieldborder){

    //search for unknown regions
    for(int j=0;j<scanlineCnt;j++){
        Scanline sl=scanVertical[j];
        for(int i=0;i<sl.edgeCnt;i++){
            int px=sl.edgesX[i];
            int py=sl.edgesY[i];
            if(i==0) continue;
            if(sl.regionsIsGreen[i]&&sl.regionsIsGreen[i-1]){
                continue;
            }
            if(sl.regionsIsGreen[i]&&sl.regionsIsWhite[i-1]){
                continue;
            }
            if(py-minBorderHeight<fieldborder[px]) break;
            int unknownCnt=0;
            int greenCnt=0;
            int greenStartCnt=0;
            int whiteCnt=0;
            int segmentCnt=0;
            for(int k=i;k<sl.edgeCnt-1;k++){
                int px2=sl.edgesX[k];
                int py2=sl.edgesY[k];
                if(py2-minBorderHeight<fieldborder[px2]) break;
                int py3=sl.edgesY[k+1];
                segmentCnt++;
                if(sl.regionsIsWhite[k]){
                    whiteCnt+=py2-py3;
                    continue;
                }
                if(sl.regionsIsGreen[k]){
                    if(segmentCnt<4){
                        greenStartCnt+=py2-py3;
                    }
                    greenCnt+=py2-py3;
                    continue;
                }
                unknownCnt+=py2-py3;
            }
            if(greenCnt+whiteCnt+unknownCnt==0) continue;

            float greenRatio=(float)greenCnt/(greenCnt+whiteCnt+unknownCnt);
            int borderHeight=py-fieldborder[px];
//            int wEst=(int)(14.885+0.4413*borderHeight);//estimated Robot width (fitting by zunzun.com)
            int objectSize=ProjectionUtil::getObjectRadius(px,py,pitchRad,rollRad,FEET_SOLO_SIZE);
            int wEst=objectSize;
            if(greenRatio<minGreenRatio&&borderHeight>minBorderHeight){
//                {
//                    py-=objectSize;
//                    if (px>=0 && py>=0 && px<width && py<height)
//                        hypoList.push_back(ObjectHypothesis(px,py,objectSize,0));
//                }
                {
                    int npy=py-wEst;
                    if(px>=0&&px<width&&npy>=0&&npy<height)
                        hypoList.emplace_back(px,npy,wEst,0);
                }
                {
                    int npx=px-wEst*1/4;
                    int npy=py-wEst;
                    if(npx>=0&&npx<width&&npy>=0&&npy<height)
                        hypoList.emplace_back(npx,npy,wEst,0);
                }
                {
                    int npx=px+wEst*1/4;
                    int npy=py-wEst;
                    if(npx>=0&&npx<width&&npy>=0&&npy<height)
                        hypoList.emplace_back(npx,npy,wEst,0);
                }
            }
            if(greenStartCnt<8){
                break;
            }
        }
    }
}
}
