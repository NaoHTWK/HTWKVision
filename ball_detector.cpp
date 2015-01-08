#include <ball_detector.h>

#include <cstring>

#include <ext_math.h>

using namespace ext_math;
using namespace std;

namespace htwk {

circle newCircle(float x,float y,float r,float q){
    circle c;
    c.x=x;
    c.y=y;
    c.r=r;
    c.q=q;
    return c;
}

Ball newBall(float x,float y,float radius,bool found){
    Ball b;
    b.x=x;
    b.y=y;
    b.radius=radius;
    b.found=found;
    return b;
}

const float BallDetector::maxBallRadius=60;

const int BallDetector::diffWhiteCr=15;
const int BallDetector::diffGreenCr=30;
const int BallDetector::minLenBF=40;
const int BallDetector::minLenBW=30;
const int BallDetector::minColorRating=20;
const int BallDetector::minCy=32;
const int BallDetector::maxCy=128;
const int BallDetector::vecCr=40;
const int BallDetector::vecCb=-40;

BallDetector::BallDetector(int width, int height, int *lutCb, int *lutCr) :
    ballX(0),
    ballY(0),
    ballRadius(0),
    ballRating(0.0),
    found(false)
{
    this->lutCb=lutCb;
    this->lutCr=lutCr;
    this->width=width;
    this->height=height;

    ballClassifier=new Classifier("../data/net_1to2F+Mirror+Artefact_IRAN_do0.5_hidden16_bs45000_bi64_iter1000_l2.0E-5.net");
    ball.cb = 180;
    ball.cr = 100;
    ball.cy = 180;

    vecLen=(int)sqrtf(vecCr*vecCr+vecCb*vecCb);
}

/*
 * detects the ball (if visible) and outputs its position
 */
void BallDetector::proceed(uint8_t *img, const int * const fieldborder, color green, color white, color goal){
    found=false;
    //grob schätzen, wo der Ball liegen könnte
    point_2d guess={0,0};
    int colorRating=getInitialGuess(img,guess,fieldborder,4,4,white);
    if(colorRating<minColorRating)return;
    //Farbunterschied zu Boden und Linien hoch genug?
    ball=getBallColor(img,guess);
    if(ball.cr>green.cr+diffGreenCr&&ball.cr>white.cr+diffWhiteCr){
        point_2d points1[NUM_STAR_SCANLINES];
        if(!raytraceCircle(img, guess, points1, ball, green, white, goal))
            return;
        point_2d innerPos=getInnerPoint(points1, guess);

        updateBallColor(img,ball,points1, NUM_STAR_SCANLINES);

        point_2d points2[NUM_STAR_SCANLINES];
        if(!raytraceCircle(img, innerPos, points2, ball, green, white, goal))return;

        point_2d innerPos2=getInnerPoint(points2, innerPos);
        if(!raytraceCircle(img, innerPos2, points1, ball, green, white, goal))return;

        float diameterGuess=guessDiameter(points2);
//

        point_2d allPoints[NUM_STAR_SCANLINE_CANDIDATES];
        memcpy(allPoints,points1,sizeof(point_2d)*NUM_STAR_SCANLINES);
        memcpy(allPoints+NUM_STAR_SCANLINES,points2,sizeof(point_2d)*NUM_STAR_SCANLINES);
        circle circle=ransacCircle(allPoints, 0.2f+diameterGuess*0.05f);

        if(circle.q<4)return;
        if(circle.r>maxBallRadius)return;

        if(circle.y-circle.r*2<0&&colorRating<minColorRating*2){
            return;
        }

        updateBallColor(img,ball,allPoints, NUM_STAR_SCANLINE_CANDIDATES);

        float dBFb=green.cr-ball.cr;
        float dBFr=ball.cb-green.cb;
        float lenBF=sqrtf(dBFb*dBFb+dBFr*dBFr);

        if(lenBF>0){
            dBFb/=lenBF*max(float(minLenBF),lenBF*0.5f);
            dBFr/=lenBF*max(float(minLenBF),lenBF*0.5f);
        }
        float dBF=(green.cb+ball.cb)*0.5*dBFr-(green.cr+ball.cr)*0.5*dBFb;

        float dBWb=white.cr-ball.cr;
        float dBWr=ball.cb-white.cb;
        float lenBW=sqrtf(dBWb*dBWb+dBWr*dBWr);

        if(lenBW>0){
            dBWb/=lenBW*max(float(minLenBW),lenBW);
            dBWr/=lenBW*max(float(minLenBW),lenBW);
        }
        float dBW=white.cb*dBWr-white.cr*dBWb;

        int bx=(int)(circle.x);
        int by=(int)(circle.y);
        int r=7;
        vec2df feature=createVec2df((r*2+1)*(r*2+1),1);
        int idx=0;
        for(int dy=-r;dy<=r;dy++){
            for(int dx=-r;dx<=r;dx++){
                double f=max(1.f,2*circle.r*0.7f/r);
                int px=(int)round(bx+dx*f);
                int py=(int)round(by+dy*f);
                if(px<0||px>=width||py<0||py>=height-1){
                    feature[idx++][0]=0;
                    continue;
                }
                int bCb=getCb(img,px,py);
                int bCr=getCr(img,px,py);
                float vBF=bCb*dBFr-bCr*dBFb-dBF;
                float vBW=bCb*dBWr-bCr*dBWb-dBW;
                feature[idx++][0]=(max(0.f,vBF)+max(0.f,vBW))*0.5f;
            }
        }
        float p=ballClassifier->proceed(feature)[0][0];

        if(p>0.5f){
            found=true;
            ballX=(int)circle.x;
            ballY=(int)circle.y;
            ballRadius=(int)circle.r;
            ballRating=p;

        }
    }
}

void BallDetector::updateBallColor(uint8_t *img, color ball, point_2d *points, int n){
    int ballCy=0;
    int ballCb=0;
    int ballCr=0;
    int cnt=0;
    for(int i=0;i<n-8;i++){
        if(points[i].x==-1||points[i+8].x==-1)continue;
        int px1=(2*points[i].x+points[i+8].x)/3;
        int py1=(2*points[i].y+points[i+8].y)/3;
        int px2=(points[i].x+2*points[i+8].x)/3;
        int py2=(points[i].y+2*points[i+8].y)/3;
        ballCy+=getY(img,px1,py1);
        ballCy+=getY(img,px2,py2);
        ballCb+=getCb(img,px1,py1);
        ballCb+=getCb(img,px2,py2);
        ballCr+=getCr(img,px1,py1);
        ballCr+=getCr(img,px2,py2);
        cnt+=2;
    }
    if(cnt>0){
        ball.cy=ballCy/cnt;
        ball.cb=ballCb/cnt;
        ball.cr=ballCr/cnt;
    }
}

float BallDetector::guessDiameter(point_2d points[NUM_STAR_SCANLINES]){
    int sum=0;
    for(int i=0;i<8;i++){
        int dx=points[i].x-points[i+8].x;
        int dy=points[i].y-points[i+8].y;
        int dist=dx*dx+dy*dy;
        sum+=dist;
    }
    return sqrt(sum>>3);
}

circle BallDetector::ransacCircle(point_2d points[NUM_STAR_SCANLINE_CANDIDATES], float maxDistEdge){
    float max=0;
    circle bestCircle=newCircle(0,0,0,0);
    for(int i=0;i<NUM_STAR_SCANLINE_CANDIDATES;i++){
        //modell:
        int p1=i;
        if(points[p1].x==-1)continue;
        int p2=(i+NUM_STAR_SCANLINES/4)%(NUM_STAR_SCANLINES*2);
        if(points[p2].x==-1)continue;
        int p3=(i+NUM_STAR_SCANLINES/2)%(NUM_STAR_SCANLINES*2);
        if(points[p3].x==-1)continue;
        circle c=getCircle(points[p1].x,points[p1].y,points[p2].x,points[p2].y,points[p3].x,points[p3].y);

        //bewerten:
        float sum=0;
        for(int j=0;j<NUM_STAR_SCANLINE_CANDIDATES;j++){
            float distx=(c.x-points[j].x);
            float disty=(c.y-points[j].y);
            float r=sqrt(distx*distx+disty*disty);
            float distR=fabsf(r-c.r);
            if(distR<maxDistEdge)sum++;
        }
        if(sum>max){
            max=sum;
            bestCircle=c;
        }
    }
    bestCircle.q=max;
    return bestCircle;
}

point_2d BallDetector::getInnerPoint(point_2d points[NUM_STAR_SCANLINES], point_2d guess){
    float maxDist=0;
    int maxIdx=0;
    for(int i=0;i<NUM_STAR_SCANLINES;i++){
        int dx=guess.x-points[i].x;
        int dy=guess.y-points[i].y;
        int dist=dx*dx+dy*dy;
        if(dist>maxDist){
            maxDist=dist;
            maxIdx=i;
        }
    }
    return newPoint2D((guess.x+points[maxIdx].x)/2,(guess.y+points[maxIdx].y)/2);
}

color BallDetector::getBallColor(uint8_t *img, point_2d guess){
    int ballCy=getY(img,(int)guess.x,(int)guess.y);
    int ballCb=getCb(img,(int)guess.x,(int)guess.y);
    int ballCr=getCr(img,(int)guess.x,(int)guess.y);
    color ball={ballCy,ballCb,ballCr};
    return ball;
}


/*
 * scans the edge of the ball by using some scan-lines. it detects the ball border by the color change from red to green (field), white (lines) or yellow (goal)
 */
bool BallDetector::raytraceCircle(uint8_t *img, point_2d pos, point_2d points[NUM_STAR_SCANLINES], color ball, color green, color white, color goal){
    int rayCnt=0;
    for(int dx=-2;dx<=2;dx++){
        for(int dy=-2;dy<=2;dy++){
            if(abs(dx)==2||abs(dy)==2){
                int bx=pos.x;
                int by=pos.y;
                int vx=dx;
                int vy=dy;
                int dist=0;
                bool borderFound=false;
                for(dist=0;dist<maxBallRadius;dist++){
                    bx+=vx;
                    by+=vy;
                    if(bx<2||bx>=width-1){
                        vx=-vx;
                        continue;
                    }
                    if(by<2||by>=height-1){
                        vy=-vy;
                        continue;
                    }
                    int cy=getY(img,bx,by);
                    int cb=getCb(img,bx,by);
                    int cr=getCr(img,bx,by);

                    int diffGreen=abs(cy-green.cy)+4*(abs(cb-green.cb)+abs(cr-green.cr));
                    int diffOrange=abs(cy-ball.cy)+4*(abs(cb-ball.cb)+abs(cr-ball.cr));
                    if(diffOrange<diffGreen){
                        int diffWhite=abs(cy-white.cy)+4*(abs(cb-white.cb)+abs(cr-white.cr));
                        if(diffOrange<diffWhite){
                            int diffGoal=abs(cy-goal.cy)+4*(abs(cb-goal.cb)+abs(cr-goal.cr));
                            if(diffOrange<diffGoal){
                                continue;
                            }
                        }
                    }
                    borderFound=true;
                    break;
                }
                if(borderFound){
                    points[rayCnt]=newPoint2D(bx,by);
                }else{
                    points[rayCnt]=newPoint2D(-1,-1);
                }
                rayCnt++;
            }
        }
    }
    return true;
}

/*
 * finds an initial seed point for the ball detection.
 * the point is the one with the highest Cr-Value in a small neighborhood, but below the field border
 */
int BallDetector::getInitialGuess(uint8_t *img, point_2d &maxPos, const int * const fieldborder, const int dy, const int dx, color white){
    int maxVal=0;
    int startY=getYStartCoord(fieldborder);
    for(int y=startY;y<height-1;y+=dy){
        int lastcv=0;
        for(int x=1;x<width;x+=dx){

            int cy=getY(img,x,y);
            int cb=getCb(img,x,y)-128;
            int cr=getCr(img,x,y)-128;
            int cv1=max(0,min(vecLen*vecLen,cr*vecCr+cb*vecCb))>>4;
            if(cv1==0){
                lastcv=0;
                continue;
            }
            int cv2=max(0,64*vecLen-abs(cr*vecCb-cb*vecCr))>>4;
            int cv3=((minCy+min(minCy,cy))*(minCy+max(0,min(minCy,maxCy-cy))))>>4;
            int cv=cv1*cv2*cv3;
            int sum=cv+lastcv;

            if(sum>=maxVal){
                if(y>=min(min(fieldborder[max(0,x-16)],fieldborder[min(width-1,x+16)]),fieldborder[x])){
                    maxVal=sum;
                    maxPos.x=x;
                    maxPos.y=y;
                }
            }
            lastcv=cv;
        }
    }
    return maxVal/2/vecLen/vecLen/minCy;
}

//where to scan for the ball (maximal y coordinate..top y-coordinate in the fieldborder)
int BallDetector::getYStartCoord(const int * const fieldborder) const {
    int startY=height-2;
    for(int x=8;x<width-8;x+=16){
        startY=min(startY,fieldborder[x]);
    }
    return startY;
}

//calculates a circle with center point and radius from 3 given points
circle BallDetector::getCircle(float x1,float y1,float x2,float y2,float x3,float y3){
    float f2 = x3*x3-x3*x2-x1*x3+x1*x2+y3*y3-y3*y2-y1*y3+y1*y2;
    float g2 = x3*y1-x3*y2+x1*y2-x1*y3+x2*y3-x2*y1;
    float m=0;
    if(g2!=0) m = (f2/g2);

    float c = (m*y2)-x2-x1-(m*y1);
    float d = (m*x1)-y1-y2-(x2*m);
    float e = (x1*x2)+(y1*y2)-(m*x1*y2)+(m*x2*y1);

    float x = (c/2);
    float y = (d/2);
    float s = (((x)*(x))+((y)*(y))-e);
    float r = pow(s,.5f);
    return newCircle(-x,-y,r,0);
}

int BallDetector::getBallRadius() const {
    if(ballRadius>width/4)
        return width/4;
    return ballRadius;
}

bool BallDetector::isBall(int x, int y) const {
    if(!found)
        return false;
    int dx=x-ballX;
    int dy=y-ballY;
    int dist=dx*dx+dy*dy;
    return dist<ballRadius*ballRadius;
}

Ball BallDetector::getBall(){
    return newBall(ballX,ballY,ballRadius,found);
}

}  // namespace htwk
