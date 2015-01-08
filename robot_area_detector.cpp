#include <robot_area_detector.h>

#include <ext_math.h>

using namespace ext_math;
using namespace std;

namespace htwk {

RobotAreaDetector::RobotAreaDetector(int width, int height, int *lutCb, int *lutCr, int scanlineCnt) : width(width), height(height) {
    this->lutCb=lutCb;
    this->lutCr=lutCr;

    minRobotRegions=3;
    minRobotHeight=16;
    minRobotWidth=20;
    minRobotArea=800;
    this->scanlineCnt=scanlineCnt;
    robotAreaYTop=(int*)malloc(sizeof(int)*(scanlineCnt));
    robotAreaYBottom=(int*)malloc(sizeof(int)*(scanlineCnt));
    robotAreaYTmp=(int*)malloc(sizeof(int)*(scanlineCnt));
    this->robotAreas=new vector<Rect>();
}


RobotAreaDetector::~RobotAreaDetector(){
}



void RobotAreaDetector::proceed(const uint8_t * const img, Scanline *scanVertical, const int * const fieldborder, FieldColorDetector *field, const Ball &ball, const vector<GoalPost> &goalPosts){

    robotAreas->clear();
    int BORDER_DISTANCE=5;

    for(int i=0;i<scanlineCnt;i++){
        robotAreaYTop[i]=height-1;
        robotAreaYBottom[i]=0;
    }

    //search for unknown regions
    for(int j=0;j<scanlineCnt;j++){
        Scanline sl=scanVertical[j];
        robotAreaYTop[j]=fieldborder[sl.edgesX[0]];
        for(int i=0;i<sl.edgeCnt;i++){
            int px=sl.edgesX[i];
            int py=sl.edgesY[i];

            if(sl.regionsIsWhite[i]||sl.regionsIsGreen[i]){
                continue;
            }
            if(py-BORDER_DISTANCE<fieldborder[px])break;
            int unknownCnt=0;
            int greenCnt=0;
            int greenStartCnt=0;
            int whiteCnt=0;
            int segmentCnt=0;
            for(int k=i;k<sl.edgeCnt-1;k++){
                int px2=sl.edgesX[k];
                int py2=sl.edgesY[k];
                if(py2-BORDER_DISTANCE<fieldborder[px2])break;
                int py3=sl.edgesY[k+1];
                segmentCnt++;
                if(sl.regionsIsWhite[k]){
                    whiteCnt+=py2-py3;
                    continue;
                }
                if(sl.regionsIsGreen[k]){
                    if(segmentCnt<5){
                        greenStartCnt+=py2-py3;
                    }
                    greenCnt+=py2-py3;
                    continue;
                }
                unknownCnt+=py2-py3;
            }
            if(greenCnt+whiteCnt+unknownCnt==0)continue;
            double unknownRatio=(double)unknownCnt/(greenCnt+whiteCnt+unknownCnt);
            int borderHeight=py-fieldborder[px];

            if(greenStartCnt<borderHeight*0.1&&unknownRatio>0.6&&segmentCnt>2){
                robotAreaYBottom[j]=max(robotAreaYBottom[j],py);
            }
        }

    }

    //filter outliners:

    //dilate
    morphMin(robotAreaYTop);
    morphMax(robotAreaYBottom);

    //erode
    morphMax(robotAreaYTop);
    morphMin(robotAreaYBottom);

    //search for rectangles with maximal area
    searchRobotAreas(scanVertical);

    vector<Rect> areas;
    for(Rect &r : *robotAreas){
        int robotRight=searchRobotBorder(fieldborder,img,field,ball,r,(r.xLeft+r.xRight*2)/3,2);
        int robotLeft=searchRobotBorder(fieldborder,img,field,ball,r,(r.xLeft*2+r.xRight)/3,-2);
        r.xLeft=robotLeft;
        r.xRight=robotRight;
        r.yTop=(int)(max(0.f,r.yTop-1.4f*(robotRight-robotLeft)));
        for(Rect &r2 : areas){
            if(r.xRight-r.xLeft<r2.xRight-r2.xLeft){
                if(r.xLeft<r2.xRight&&r.xRight>r2.xLeft){
                    goto next;
                }
            }else{
                if(r.xLeft<r2.xRight&&r.xRight>r2.xLeft){
                    r2.xLeft=r.xLeft;
                    r2.xRight=r.xRight;
                    r2.yBottom=r.yBottom;
                    r2.yTop=r.yTop;
                    goto next;
                }
            }
        }
        if(r.xRight-r.xLeft>minRobotWidth){
            areas.push_back(r);
        }
next:;
    }
    robotAreas->clear();
    for(const Rect &r : areas){
        robotAreas->push_back(r);
    }
}



bool RobotAreaDetector::isInGoal(vector<GoalPost> goalPosts, Rect r){
    for(const GoalPost &gp : goalPosts){
        if(gp.x>r.xLeft&&gp.x<r.xRight){
            return true;
        }
    }
    return false;
}


int RobotAreaDetector::searchRobotBorder(const int * const fieldborder,
                          const uint8_t * const img, FieldColorDetector *field,
                          const Ball &ball, Rect r, int xStart, int dx) const {
    int xData[7];
    for(int i=0;i<7;i++){
        double f=i/6.;
        int py=(int)(r.yTop*f+r.yBottom*(1-f));
        int noGreenCnt=0;
        int px=xStart;
        int lastX=xStart;
        for(;px<width&&px>0;px+=dx){
            if(fieldborder[px]>py)break;
            if(isBall(px,py,ball)){
                break;
            }
            int cr=getCr(img,px,py);
            int cb=getCb(img,px,py);
            int cy=getY(img,px,py);
            if(field->isGreen(cy,cb,cr)){
                noGreenCnt++;
                if(noGreenCnt>5)break;
            }else{
                lastX=px;
                noGreenCnt=0;
            }
        }
        xData[i]=lastX;
    }
    isort(xData,7);
    int nx=xData[3];
    return nx;
}


bool RobotAreaDetector::isBall(int x, int y, const Ball &ball){
    if(!ball.found)return false;
    int dx=x-ball.x;
    int dy=y-ball.y;
    int dist=dx*dx+dy*dy;
    return dist<(ball.radius+4)*(ball.radius+4);
}


void RobotAreaDetector::searchRobotAreas(Scanline *scanVertical){
    int maxArea=0;
    int bestXLeft=0;
    int bestXRight=0;
    int bestYTop=0;
    int bestYBottom=0;
    for(int j=0;j<scanlineCnt;j++){
        int robotRegionHeight=robotAreaYBottom[j]-robotAreaYTop[j];
        if(robotRegionHeight<minRobotHeight){
            if(maxArea>minRobotArea){
                Rect r;
                r.xLeft=bestXLeft-7;
                r.xRight=bestXRight+7;
                r.yTop=bestYTop;
                r.yBottom=bestYBottom;
                robotAreas->push_back(r);
            }
            maxArea=0;
            bestXLeft=0;
            bestXRight=0;
            bestYTop=0;
            bestYBottom=0;
            continue;
        }
        Scanline sl=scanVertical[j];
        int xLeft=sl.edgesX[0];
        int xRight=sl.edgesX[0];
        int yTop=robotAreaYTop[j];
        int yBottom=robotAreaYBottom[j];
        for(int k=j;k<scanlineCnt;k++){
            Scanline sl2=scanVertical[k];
            xRight=sl2.edgesX[0];
            yTop=max(yTop,robotAreaYTop[k]);
            yBottom=min(yBottom,robotAreaYBottom[k]);
            bestYBottom=max(bestYBottom,robotAreaYBottom[k]);
            if(yBottom-yTop>=minRobotHeight){
                int area=((xRight-xLeft)+32)*(yBottom-yTop);
                if(area>maxArea){
                    maxArea=area;
                    bestXLeft=xLeft;
                    bestXRight=xRight;
                    bestYTop=yTop;
                }
            }else{
                break;
            }
        }
    }
}


void RobotAreaDetector::morphMax(int *data){
    for(int j=0;j<scanlineCnt;j++){
        robotAreaYTmp[j]=data[j];
    }
    for(int j=0;j<scanlineCnt;j++){
        int y=robotAreaYTmp[j];
        for(int dj=-1;dj<=1;dj++){
            int nj=j+dj;
            if(nj<0||nj>=scanlineCnt)continue;
            y=max(y,robotAreaYTmp[nj]);
        }
        data[j]=y;
    }
}


void RobotAreaDetector::morphMin(int *data){
    for(int j=0;j<scanlineCnt;j++){
        robotAreaYTmp[j]=data[j];
    }
    for(int j=0;j<scanlineCnt;j++){
        int y=robotAreaYTmp[j];
        for(int dj=-1;dj<=1;dj++){
            int nj=j+dj;
            if(nj<0||nj>=scanlineCnt)continue;
            y=min(y,robotAreaYTmp[nj]);
        }
        data[j]=y;
    }
}


vector<Rect>* RobotAreaDetector::getRobotAreas(){
    return robotAreas;
}

}  // namespace htwk
