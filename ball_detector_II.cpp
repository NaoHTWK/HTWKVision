#include <cstring>

#include <ball_detector_II.h>
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

BallDetectorII::BallDetectorII(int width, int height, int8_t *lutCb, int8_t *lutCr) :
    BaseDetector(width, height, lutCb, lutCr),
    ballX(0),
    ballY(0),
    ballRadius(0),
    ballRating(0.0),
    found(false)
{
    iWidth=width/INTEGRAL_SCALE;
    iHeight=height/INTEGRAL_SCALE;
    blockSize=BLOCK_SIZE/INTEGRAL_SCALE;
    numBlockX=iWidth/blockSize;
    numBlockY=iHeight/blockSize;
    integralImg=new int[iWidth*iHeight];
    ratingImg=new int[iWidth*iHeight];
    blockMeanValues=new int[numBlockX*numBlockY];
    blockMaxX=new int[numBlockX*numBlockY];
    blockMaxY=new int[numBlockX*numBlockY];
    isBlockUsable=new int[numBlockX*numBlockY];
    blockObjectRadius=new int[numBlockX*numBlockY];
    //ballHypotheses=new LinkedList<ObjectHypothesis>();
    //ballClassifier=new Classifier("./data/net_1+3F_GOALL2015_do0.5_hidden16_bs45000_bi64_iter1000_l2.0E-5.net");
}

/*
 * detects the ball (if visible) and outputs its position
 */
void BallDetectorII::proceed(uint8_t *img, const int * const fieldborder, const FieldColorDetector *const fieldColorDetector){
    createIntegralImage(img);
    setBlockValues(img,fieldColorDetector);
    for(int by=0;by<numBlockY;by++){
        for(int bx=0;bx<numBlockX;bx++){
            int blockAddr=bx+by*numBlockX;
            if(isBlockUsable[blockAddr]==0){
                continue;
            }
            int px=bx*blockSize;
            int py=by*blockSize;
            for(int y=py;y<py+blockSize;y++){
                for(int x=px;x<px+blockSize;x++){
                    for(int dy=0;dy<INTEGRAL_SCALE;dy++){
                        for(int dx=0;dx<INTEGRAL_SCALE;dx++){
                            int nx=x*INTEGRAL_SCALE+dx;
                            int ny=y*INTEGRAL_SCALE+dy;
                            setY(img,nx,ny,0);
                        }
                    }
                }
            }

        }
    }
//    float p=ballClassifier->proceed(feature)[0][0];
//    if(p>0.5f){
//        found=true;
//        ballX=(int)circle.x;
//        ballY=(int)circle.y;
//        ballRadius=(int)circle.r;
//        ballRating=p;

//    }
}


void BallDetectorII::setBlockValues(uint8_t *img, const FieldColorDetector *const fieldColorDetector) const{
    memset(isBlockUsable,0,sizeof(&isBlockUsable));//need to dereference?
    for(int by=0;by<numBlockY;by++){
        for(int bx=0;bx<numBlockX;bx++){
            int blockAddr=bx+by*numBlockX;
            int px=bx*blockSize;
            int py=by*blockSize;
            int objectSize=10;//(int)(getBallRadius(INTEGRAL_SCALE*(px+blockSize/2),INTEGRAL_SCALE*(py+blockSize/2)));
            blockObjectRadius[blockAddr]=objectSize;
            int iV=getIntegralValue(px,py,px+blockSize-1,py+blockSize-1)/blockSize/blockSize;
            blockMeanValues[blockAddr]=iV;
            if(objectSize>=MIN_OBJECT_RADIUS){
                px+=blockSize/2;
                py+=blockSize/2;
                px*=INTEGRAL_SCALE;
                py*=INTEGRAL_SCALE;
                int cyReal=getY(img,px,py);
                int cbReal=getCb(img,px,py);
                int crReal=getCr(img,px,py);
                if(!fieldColorDetector->isGreen(cyReal,cbReal,crReal)){
                    isBlockUsable[blockAddr]=255;
                }
            }
        }
    }
    int r=1;
    int mean=0;
    int cnt=0;
    for(int by=0;by<numBlockY;by++){
        for(int bx=0;bx<numBlockX;bx++){
            int blockAddr=bx+by*numBlockX;
            if(isBlockUsable[blockAddr]>0||blockObjectRadius[blockAddr]<MIN_OBJECT_RADIUS)
                continue;
            int c=blockMeanValues[blockAddr];
            int maxDiff=0;
            for(int dy=-r;dy<=r;dy++){
                for(int dx=-r;dx<=r;dx++){
                    if(dx==0&&dy==0)continue;
                    int px=bx+dx;
                    int py=by+dy;
                    if(px<0||px>=numBlockX||py<0||py>=numBlockY)continue;
                    int nc=blockMeanValues[px+py*numBlockX];
                    int diff=c-nc;
                    if(diff>maxDiff){
                        maxDiff=diff;
                    }
                }
            }
            mean+=maxDiff;
            cnt++;
        }
    }
    if(cnt>0){
        mean/=cnt;
    }
    for(int by=0;by<numBlockY;by++){
        for(int bx=0;bx<numBlockX;bx++){
            int blockAddr=bx+by*numBlockX;
            if(isBlockUsable[blockAddr]>0||blockObjectRadius[blockAddr]<MIN_OBJECT_RADIUS)
                continue;
            int c=blockMeanValues[blockAddr];
            int maxDiff=0;
            for(int dy=-r;dy<=r;dy++){
                for(int dx=-r;dx<=r;dx++){
                    if(dx==0&&dy==0)continue;
                    int px=bx+dx;
                    int py=by+dy;
                    if(px<0||px>=numBlockX||py<0||py>=numBlockY)continue;
                    int nc=blockMeanValues[px+py*numBlockX];
                    int diff=c-nc;
                    if(diff>maxDiff){
                        maxDiff=diff;
                    }
                }
            }
            isBlockUsable[blockAddr]=maxDiff>mean?255:0;
        }
    }
}

int BallDetectorII::getArea(int px1, int py1, int px2, int py2) const{
    return (px2-px1)*(py2-py1);
}
int BallDetectorII::getIntegralValue(int x1, int y1, int x2, int y2) const{
    return (integralImg[x2+y2*iWidth]-integralImg[x1+y2*iWidth]-integralImg[x2+y1*iWidth]+integralImg[x1+y1*iWidth]);
}

void BallDetectorII::createIntegralImage(uint8_t *img) const {
    integralImg[0]=getY(img,0,0);
    for(int x=1;x<iWidth;x++){
        integralImg[x]=integralImg[x-1]+(getY(img,x*INTEGRAL_SCALE,0))*128;
    }
    for(int y=1;y<iHeight;y++){
        int sum=0;
        for(int x=0;x<iWidth;x++){
            int addr=x+y*iWidth;
            sum+=(getY(img,x*INTEGRAL_SCALE,y*INTEGRAL_SCALE))*128;
            integralImg[addr]=sum+integralImg[addr-iWidth];
        }
    }
}

int BallDetectorII::getBallRadius() const {
    if(ballRadius>width/4)
        return width/4;
    return ballRadius;
}

bool BallDetectorII::isBall(int x, int y) const {
    if(!found)
        return false;
    int dx=x-ballX;
    int dy=y-ballY;
    int dist=dx*dx+dy*dy;
    return dist<ballRadius*ballRadius;
}

Ball BallDetectorII::getBall(){
    return newBall(ballX,ballY,ballRadius,found);
}

}  // namespace htwk
