#include "goal_detector.h"

#include <cstring>
#include <iostream>

using namespace ext_math;
using namespace std;

namespace htwk {

const int GoalDetector::maxGoalWidth=320;
const int GoalDetector::minGoalWidth=8;
const int GoalDetector::minGoalHeight=70;
const int GoalDetector::scanlineCnt=15;
const int GoalDetector::q=2;
const int GoalDetector::minGoalPostToBorderDist=4;
const bool GoalDetector::detectDarkGoals=false;

GoalDetector::GoalDetector(int width, int height, int8_t *lutCb, int8_t *lutCr)
    : BaseDetector(width, height, lutCb, lutCr)
{
    goalColor = color(255, 128, 128);
    goalClassifier = new Classifier("./data/net_WhiteGoal_15FeaturesMirror_do0.5_hidden8_bs3000_bi64_iter1000_l1.0E-6.net");
}

void GoalDetector::proceed(uint8_t *img, const int * const fieldborder, color green, color white){
    goalPosts = std::vector<GoalPost>();
    bestProbability = 0;

//    std::vector<int> edgeTable;
//    getEdgeTable(img, fieldborder, scanlineCnt, edgeTable);
//
//    int widthTable = width / q;
//
//    int acc=0;
//    int lastSum=0;
//    int lastX=0;
//    int lastAcc=0;
//    int avgX=0;
//    int avgXCnt=0;
//    int thres=40;
//    int accThres=200;
//    int goalpostAccThres=800;
//    for(int x=1;x<widthTable-1;x++){
//        int sumMax=0;
//        for(int dx=-3;dx<=3;dx++){
//            int sum=0;
//            for(int y=0;y<scanlineCnt;y++){
//                int px=x+dx*(y-scanlineCnt)/6;
//                if(px<0||px>=width)continue;
//                int edge=edgeTable[px+y*widthTable];
//                sum+=edge;
//            }
//            if(abs(sum)>abs(sumMax)){
//                sumMax=sum;
//            }
//        }
//
//        int sum=sumMax;
//        if(abs(sum)>thres&&lastSum*sum>0){
//            acc+=sum;
//            avgX+=2*x*sum;
//            avgXCnt+=sum;
//        }else{
//            if(abs(acc)>accThres){
//                int currX=avgX/avgXCnt;
//                if(lastAcc<0&&acc>0&&acc-lastAcc>goalpostAccThres){
//                    analyseGoalPost(img,lastX,currX,fieldborder,green);
//                }
//                lastAcc=acc;
//                lastX=currX;
//            }
//            acc=0;
//            avgX=0;
//            avgXCnt=0;
//        }
//        lastSum=sum;
//    }
}

void GoalDetector::analyseGoalPost(uint8_t *img, int xLeft, int xRight, const int * const fieldborder, color green){
    int goalWidth=xRight-xLeft-2;
    if(goalWidth>maxGoalWidth)return;
    int seedX=(xRight+xLeft)/2;
    int seedY=std::min(height-1,std::max(0,fieldborder[xRight]-minGoalHeight/2+minGoalPostToBorderDist*2));
    int whiteY=getY(img,seedX,seedY);
    int whiteCb=getCb(img,seedX,seedY);
    int whiteCr=getCr(img,seedX,seedY);

    int baseSearchHeight=20;
    int maxSteps=40;
    int maxBrightnessChange=20;
    int searchStepsBottom=9;
    int searchStepsInit=searchStepsBottom*2;
    int searchStepsTop=7;
    int windowSteps=9;
    int windowSize=(int)(std::max(5,static_cast<int>(goalWidth*0.7f)));

    //Point scanPoint=new Point((xRight+xLeft)/2,seedY);
    point_2d scanPoint;
    scanPoint.x=(xRight+xLeft)/2;
    scanPoint.y=seedY;
    findMaxWhite(img,scanPoint,windowSize,windowSteps,searchStepsInit);
    double sxy=0;
    double syy=0;
    double xm=0;
    double ym=0;
    double n=0;
    double maxSum=0;
    double maxSumSq=0;
    int lowerY=seedY;
    int upperY=seedY;

    while(true){
        int maxWhite=findMaxWhite(img,scanPoint,windowSize,windowSteps,searchStepsBottom);
        if(scanPoint.x<0||scanPoint.x>=width)return;
        maxSum+=maxWhite;
        maxSumSq+=maxWhite*maxWhite;

        sxy+=scanPoint.x*scanPoint.y;
        syy+=scanPoint.y*scanPoint.y;
        xm+=scanPoint.x;
        ym+=scanPoint.y;
        n++;
        lowerY=scanPoint.y;
        scanPoint.y+=std::max(3,goalWidth/3);
        int ny=scanPoint.y+goalWidth/4;
        if(ny>=height)break;
        int currCy=getY(img,scanPoint.x,ny);
        int currCb=getCb(img,scanPoint.x,ny);
        int currCr=getCr(img,scanPoint.x,ny);

        int diffGreen=abs(currCy-green.cy)+4*(abs(currCb-green.cb)+abs(currCr-green.cr));
        int diffWhite=abs(currCy-whiteY/n)+4*(abs(currCb-whiteCb/n)+abs(currCr-whiteCr/n));
        if(diffGreen<diffWhite)
            break;
        whiteY+=currCy;
        whiteCb+=currCb;
        whiteCr+=currCr;
    }
    whiteY/=n;
    whiteCb/=n;
    whiteCr/=n;
    scanPoint.y=seedY;
    float varDX=.0f;
    float varCy=.0f;
    float varCb=.0f;
    float varCr=.0f;
    int varCnt=0;
    for(int i=0;i<maxSteps;i++){

#warning this is temporary fix for an division by zero
        float xxx = (syy-ym*ym/n);
        if(xxx == 0.f)
            xxx = 1;
        float a1=(sxy-xm*ym/n)/xxx;
        float a0=(xm-a1*ym)/n;
        int predictX=(int)(scanPoint.y*a1+a0);
        scanPoint.x=predictX;
        if(scanPoint.x<0||scanPoint.x>=width)return;
        int maxWhite=findMaxWhite(img,scanPoint,windowSize,windowSteps,searchStepsTop);
        if(scanPoint.x<0||scanPoint.x>=width)return;
        int currCy=getY(img,scanPoint.x,scanPoint.y);
        int currCb=getCb(img,scanPoint.x,scanPoint.y);
        int currCr=getCr(img,scanPoint.x,scanPoint.y);
        varCy+=(currCy-whiteY)*(currCy-whiteY);
        varCb+=(currCb-whiteCb)*(currCb-whiteCb);
        varCr+=(currCr-whiteCr)*(currCr-whiteCr);
        varDX+=(predictX-scanPoint.x)*(predictX-scanPoint.x);
        varCnt++;
        float maxMean=maxSum/n;
        float diff=fabsf(maxWhite-maxMean);
        if(diff>maxBrightnessChange){
            break;
        }
        maxSum+=maxWhite;
        maxSumSq+=maxWhite*maxWhite;
        sxy+=scanPoint.x*scanPoint.y;
        syy+=scanPoint.y*scanPoint.y;
        xm+=scanPoint.x;
        ym+=scanPoint.y;
        n++;
        upperY=scanPoint.y;
        scanPoint.y-=std::max(3,goalWidth/4);
        int ny=scanPoint.y-goalWidth/2;
        if(ny<=0)break;
    }

    varDX=sqrtf(varDX/varCnt);
    varCy=sqrtf(varCy/varCnt);
    varCb=sqrtf(varCb/varCnt);
    varCr=sqrtf(varCr/varCnt);

#warning this is temporary fix for an division by zero
        float xxx = (syy-ym*ym/n);
        if(xxx == 0.f)
            xxx = 1;

    float a1=(sxy-xm*ym/n)/xxx;
    float a0=(xm-a1*ym)/n;

    lowerY=searchBase(img,lowerY,baseSearchHeight,a1,a0,whiteY,whiteCb,whiteCr,green);


    int baseX=(int)(lowerY*a1+a0);
    int baseY=lowerY;
    if(baseX<0||baseX>=width)return;

    GoalPost gp(point_2d(baseX, baseY));
    gp.upperPoint=point_2d((int)(upperY*a1+a0), upperY);
    gp.color=color(whiteY, whiteCb, whiteCr);
    gp.width=goalWidth;

    vec2df feature=ext_math::createVec2df(15,1);
    feature[0][0]=gp.width;
    float dx=gp.basePoint.x-gp.upperPoint.x;
    float dy=gp.basePoint.y-gp.upperPoint.y;
    float len=sqrtf(dx*dx+dy*dy);
    feature[1][0]=len;
    feature[2][0]=varDX;
    feature[3][0]=varCy;
    feature[4][0]=varCb;
    feature[5][0]=varCr;
    feature[6][0]=baseY-fieldborder[baseX];
    feature[7][0]=getGreenFeature(img,baseX,baseY,goalWidth/2,whiteY,whiteCb,whiteCr,green,0,1,5);
    feature[8][0]=getGreenFeature(img,baseX+goalWidth/2,baseY,goalWidth/2,whiteY,whiteCb,whiteCr,green,1,0,5);
    feature[9][0]=getGreenFeature(img,baseX-goalWidth/2,baseY,goalWidth/2,whiteY,whiteCb,whiteCr,green,-1,0,5);
    feature[10][0]=getGreenFeature(img,baseX+goalWidth/4,baseY,goalWidth/2,whiteY,whiteCb,whiteCr,green,1,1,5);
    feature[11][0]=getGreenFeature(img,baseX-goalWidth/4,baseY,goalWidth/2,whiteY,whiteCb,whiteCr,green,-1,1,5);
    feature[12][0]=getGreenFeature(img,baseX+goalWidth/4,baseY,goalWidth/2,whiteY,whiteCb,whiteCr,green,1,-1,5);
    feature[13][0]=getGreenFeature(img,baseX-goalWidth/4,baseY,goalWidth/2,whiteY,whiteCb,whiteCr,green,-1,-1,5);
//    float maxMean=maxSum/n;
    float maxVar=(maxSumSq-maxSum*maxSum/n)/n;
    feature[14][0]=maxVar;
    gp.probability=goalClassifier->proceed(feature)[0][0];
//    if (gp.probability>0.2)
//        std::cout << "net value " << p << std::endl;
    if(gp.probability>0.5){
        goalPosts.push_back(gp);
    }

    if(gp.probability > bestProbability) {
        goalColor = gp.color;
    }
}

float GoalDetector::getGreenFeature(uint8_t *img, int baseX, int baseY, int length, int whiteY, int whiteCb, int whiteCr, color green, int vx, int vy, int steps){
    int px=baseX;
    int py=baseY;
    double vLen=sqrtf(vx*vx+vy*vy);
    double greenRatio=0;
    for(int i=1;i<=steps;i++){
        int nx=(int)(px+length*i*vx/steps/vLen);
        int ny=(int)(py+length*i*vy/steps/vLen);
        if(nx<0||nx>=width||ny<0||ny>=height)continue;
        int currCy=getY(img,nx,ny);
        int currCb=getCb(img,nx,ny);
        int currCr=getCr(img,nx,ny);
        int diffGreen=abs(currCy-green.cy)+4*(abs(currCb-green.cb)+abs(currCr-green.cr));
        int diffWhite=abs(currCy-whiteY)+4*(abs(currCb-whiteCb)+abs(currCr-whiteCr));
        if(diffGreen<diffWhite){
            greenRatio++;
        }
    }
    float feature=2*greenRatio/steps-1;
    return feature;
}

int GoalDetector::searchBase(uint8_t *img, int lowerY, int baseSearchHeight, float a1, float a0, int whiteY, int whiteCb, int whiteCr, color green){
    for(int py=lowerY;py<std::min(height-1,lowerY+baseSearchHeight);py++){
        int px=(int)(py*a1+a0);
        if(px<0||px>=width)break;
        int currCy=getY(img,px,py);
        int currCb=getCb(img,px,py);
        int currCr=getCr(img,px,py);
        int diffGreen=abs(currCy-green.cy)+4*(abs(currCb-green.cb)+abs(currCr-green.cr));
        int diffWhite=abs(currCy-whiteY)+4*(abs(currCb-whiteCb)+abs(currCr-whiteCr));
        if(diffGreen<diffWhite)
            break;
        lowerY=py;
    }
    return lowerY;
}

int GoalDetector::findMaxWhite(uint8_t *img, point_2d &base, int windowSize, int windowSteps, int searchSteps){
    int sumCy=0;
    //int gain=4;
    for(int j=-windowSteps/2;j<-windowSteps/2+windowSteps;j++){
        int nx=base.x+(j-searchSteps/2)*windowSize/(windowSteps-1);

        if(nx<0)nx=0;
        if(nx>=width)nx=width-1;
        int cy=getY(img,nx,base.y);
        if(detectDarkGoals){
            cy=128+(128-getY(img,nx,base.y));
        }
        sumCy+=cy;
    }
    int maxSumCy=sumCy;
    int bestI=-searchSteps/2;
    for(int i=-searchSteps/2+1;i<-searchSteps/2+searchSteps;i++){
        int nxStart=base.x+(i-windowSteps/2-1)*windowSize/(windowSteps-1);
        int nxEnd=base.x+(i-windowSteps/2+windowSteps-1)*windowSize/(windowSteps-1);
        if(nxStart<0)nxStart=0;
        if(nxStart>=width)nxStart=width-1;
        if(nxEnd<0)nxEnd=0;
        if(nxEnd>=width)nxEnd=width-1;
        int cyStart=getY(img,nxStart,base.y);
        int cyEnd=getY(img,nxEnd,base.y);
        if(detectDarkGoals){
            cyStart=128+(128-getY(img,nxStart,base.y));
            cyEnd=128+(128-getY(img,nxEnd,base.y));
        }
        sumCy+=cyEnd-cyStart;
        if(sumCy>maxSumCy){
            maxSumCy=sumCy;
            bestI=i;
        }
    }
    for(int j=-windowSteps/2;j<-windowSteps/2+windowSteps;j++){
        int nx=base.x+(bestI+j)*windowSize/(windowSteps-1);
        if(nx<0||nx>=width)continue;
    }
    base.x+=bestI*windowSize/(windowSteps-1);
    return maxSumCy/windowSteps;
}

void GoalDetector::getEdgeTable(uint8_t *img, const int * const fieldborder, int scanlineCnt, std::vector<int>& hist){
    int edgeThreshold=1;
    int widthHist=width/q;
    hist.resize(scanlineCnt*widthHist);
    for(int x=4;x<width-2;x+=q){
        for(int y=0;y<scanlineCnt;y++){
            int py=std::min(height-1,std::max(0,std::max(fieldborder[x],minGoalHeight-minGoalPostToBorderDist*2)-y*minGoalHeight/scanlineCnt+minGoalPostToBorderDist*2));
            int f=getY(img,x-2,py)-getY(img,x+2,py);
            if(detectDarkGoals){
                f=f/2+2*(getY(img,x+2,py)-getY(img,x-2,py));
            }
            if(abs(f)>edgeThreshold){
                hist[x/q+y*widthHist]=f;
            }
        }
    }
}

std::vector<GoalPost> GoalDetector::getGoalPosts(){
    return goalPosts;
}

}  // namespace htwk

