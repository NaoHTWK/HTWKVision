#include "hypotheses_generator_blocks.h"

#include <Eigen/Dense>
#include <projectionutil.h>

namespace htwk {

HypothesesGeneratorBlocks::HypothesesGeneratorBlocks(int width, int height, int8_t *lutCb, int8_t *lutCr)
    : BaseDetector(width, height, lutCb, lutCr)
    , numBlockX(IntegralImage::iWidth/blockSize)
    , numBlockY(IntegralImage::iHeight/blockSize)
{
    ratingImg=new int[IntegralImage::iWidth*IntegralImage::iHeight];
    blockMeanValues=new int[numBlockX*numBlockY];
    blockMaxX=new int[numBlockX*numBlockY];
    blockMaxY=new int[numBlockX*numBlockY];
    isBlockUsable.resize(numBlockX*numBlockY,false);
    blockObjectRadius=new int[numBlockX*numBlockY];
}

HypothesesGeneratorBlocks::~HypothesesGeneratorBlocks()
{
    delete [] ratingImg;
    delete [] blockMeanValues;
    delete [] blockMaxX;
    delete [] blockMaxY;
    delete [] blockObjectRadius;
}

void HypothesesGeneratorBlocks::proceed(uint8_t *img, const int * const fieldborder, float camPitch, float camRoll, IntegralImage *integralImg){
    rollRad=camRoll;
    pitchRad=camPitch;
    integral=integralImg;
    setBlockValues(img,fieldborder);
    hypoList.clear();
    searchHypotheses(SCALE_MEDIUM,fieldborder);
    selectBestHypotheses();
}

void HypothesesGeneratorBlocks::setBlockValues(uint8_t *img, const int * const border){
    isBlockUsable.assign(numBlockX*numBlockY,false);

    std::vector<block> blocks;
    for(int by=0;by<numBlockY;by++){
        for(int bx=0;bx<numBlockX;bx++){
            int blockAddr=bx+by*numBlockX;
            int px=bx*blockSize;
            int py=by*blockSize;
            float objectSize=ProjectionUtil::getObjectRadius(IntegralImage::INTEGRAL_SCALE*(px+blockSize/2),
                                                             IntegralImage::INTEGRAL_SCALE*(py+blockSize/2),
                                                             pitchRad,rollRad,BALL_SIZE);
            blockObjectRadius[blockAddr]=std::max(MIN_OBJECT_RADIUS_NORM,objectSize);

            int mi=255;
            int ma=0;
            for(int y=py;y<py+blockSize;y+=3){
                for(int x=px;x<px+blockSize;x+=3){
                    int nx=x*IntegralImage::INTEGRAL_SCALE;
                    int ny=y*IntegralImage::INTEGRAL_SCALE;
                    int cy=getY(img,nx,ny);
                    mi=std::min(cy,mi);
                    ma=std::max(cy,ma);
                }
            }
            int diffMinMax=ma-mi;
            if(border[px*IntegralImage::INTEGRAL_SCALE]-borderCompensation>py*IntegralImage::INTEGRAL_SCALE)continue;
            if(objectSize>=MIN_OBJECT_RADIUS){
                block b;
                b.x=bx;
                b.y=by;
                b.diff=diffMinMax;
                blocks.push_back(b);
            }
        }
    }

    std::sort(blocks.begin(), blocks.end(), [](const block& a, const block& b){return a.diff > b.diff;});

    int maxBlocks=50;
    int numBlocks=0;
    for(block b:blocks){
        int bx=b.x;
        int by=b.y;
        int r=1;
        for(int dy=-r;dy<=r;dy++){
            for(int dx=-r;dx<=r;dx++){
                if(dx*dx+dy*dy>1)continue;
                int nx=bx+dx;
                int ny=by+dy;
                if(nx<0||ny<0||nx>=numBlockX||ny>=numBlockY)continue;
                int blockAddr=nx+ny*numBlockX;
                isBlockUsable[blockAddr]=true;
            }
        }
        if(numBlocks++>=maxBlocks){
            break;
        }
    }
}

void HypothesesGeneratorBlocks::searchHypotheses(float objectScale, const int * const border){
    memset(ratingImg,0,sizeof(int)*IntegralImage::iWidth*IntegralImage::iHeight);
    getRatingGrid(objectScale,border);
    findMaxima(objectScale,border);
}

void HypothesesGeneratorBlocks::getRatingGrid(float objectScale, const int * const border) const{
    for(int by=0;by<numBlockY;by++){
        for(int bx=0;bx<numBlockX;bx++){
            int blockAddr=bx+by*numBlockX;
            if(!isBlockUsable[blockAddr]){
                continue;
            }
            int px=bx*blockSize;
            int py=by*blockSize;
            if(border[px*IntegralImage::INTEGRAL_SCALE]-borderCompensation>py*IntegralImage::INTEGRAL_SCALE) continue;
            float objectRadius=blockObjectRadius[blockAddr]*objectScale;
            int r=(int)(objectRadius/IntegralImage::INTEGRAL_SCALE);
            float outerAreaRadius=1.41f;//Optimizer.getParam("OuterArea",1.5,2.5);
            int r2=(int)((objectRadius*outerAreaRadius/IntegralImage::INTEGRAL_SCALE));
            int maxRating=0;
            int maxX=0;
            int maxY=0;
            int stepSize=2;
            if(r>=10)stepSize=2;
            if(r>=20)stepSize=4;
            for(int y=py;y<py+blockSize;y+=stepSize){
                for(int x=px;x<px+blockSize;x+=stepSize){
                    getRating(y, maxY, stepSize, maxRating, r, r2, maxX, x);
                }
            }
            blockMaxX[blockAddr]=maxX;
            blockMaxY[blockAddr]=maxY;
        }
    }
}

void HypothesesGeneratorBlocks::findMaxima(float objectScale, const int * const border){
    int steps=16;
    float stepWidth=M_PI*2/steps;
    for(int by=0;by<numBlockY;by++){
        for(int bx=0;bx<numBlockX;bx++){
            int blockAddr=bx+by*numBlockX;
            if(!isBlockUsable[blockAddr]){
                continue;
            }
            int px=bx*blockSize;
            int py=by*blockSize;
//            for(int y=py;y<py+blockSize;y++){
//                for(int x=px;x<px+blockSize;x++){
//                    int rating=ratingImg[x+y*iWidth];
//                    int ratingVis=(int)max(0.,min(128+0.03*rating,255.));
//                    for(int dy=0;dy<INTEGRAL_SCALE;dy++){
//                        for(int dx=0;dx<INTEGRAL_SCALE;dx++){
//                            int nx=x*INTEGRAL_SCALE+dx;
//                            int ny=y*INTEGRAL_SCALE+dy;
//                            setY(img,nx,ny,ratingVis);
//                        }
//                    }
//                }
//            }
            if(border[px*IntegralImage::INTEGRAL_SCALE]-borderCompensation>py*IntegralImage::INTEGRAL_SCALE)continue;
            float objectRadius=blockObjectRadius[blockAddr]*objectScale;
            int r=(int)(objectRadius/IntegralImage::INTEGRAL_SCALE);
            int maxX=blockMaxX[blockAddr];
            int maxY=blockMaxY[blockAddr];
            if(ratingImg[maxX+maxY*IntegralImage::iWidth]<MIN_RATING)continue;

            int maxRating=-10000000;
            int minRating=10000000;
            for(float a=0;a<M_PI*2-0.001;a+=stepWidth){
                int qx=(int)std::round(maxX+std::sin(a)*r);
                int qy=(int)std::round(maxY+std::cos(a)*r);
                if(qx<0||qy<0||qx>=IntegralImage::iWidth||qy>=IntegralImage::iHeight)continue;
                int rating=ratingImg[qx+qy*IntegralImage::iWidth];
                if(rating>maxRating){
                    maxRating=rating;
                }
                if(rating<minRating){
                    minRating=rating;
                }
            }
            int ratingMax=ratingImg[maxX+maxY*IntegralImage::iWidth]-maxRating;
            if(ratingMax>0){
                maxX=maxX*IntegralImage::INTEGRAL_SCALE+IntegralImage::INTEGRAL_SCALE/2-1;
                maxY=maxY*IntegralImage::INTEGRAL_SCALE+IntegralImage::INTEGRAL_SCALE/2-1;
                float ballRadiusCorrection=0.91f;//Optimizer.getParam("BallRadiusCorrection",0.8,1.2);
                hypoList.emplace_back(maxX,maxY,(int)(r*ballRadiusCorrection*IntegralImage::INTEGRAL_SCALE),ratingMax);
            }
        }
    }
}

void HypothesesGeneratorBlocks::selectBestHypotheses(){
    std::vector<ObjectHypothesis> tmp;
    tmp.swap(hypoList);

    std::sort(tmp.begin(), tmp.end(), [](const ObjectHypothesis& a, const ObjectHypothesis& b){return a.rating > b.rating;});
    for(const ObjectHypothesis &hyp:tmp){
        if(!containsObject(hyp)){
            hypoList.push_back(hyp);
            if(hypoList.size()>=MAX_NUM_HYPOTHESES){
                break;
            }
        }
    }
}

bool HypothesesGeneratorBlocks::containsObject(const ObjectHypothesis &hyp) const{
    int maxDistSq=(int)(hyp.r*hyp.r*1.5f*1.5f);
    for(const ObjectHypothesis &h:hypoList){
        int dx=h.x-hyp.x;
        int dy=h.y-hyp.y;
        float distSq=dx*dx+dy*dy;
        if(distSq<maxDistSq){
            return true;
        }
    }
    return false;
}

void HypothesesGeneratorBlocks::getRating(int y, int &maxY, int stepSize, int &maxRating, int r, int r2, int &maxX, int x) const{
    int px1=std::max(x-r,0);
    int py1=std::max(y-r,0);
    int px2=std::min(x+r,IntegralImage::iWidth-1);
    int py2=std::min(y+r,IntegralImage::iHeight-1);

    int px3=std::max(x-r2,0);
    int py3=std::max(y-r2,0);
    int px4=std::min(x+r2,IntegralImage::iWidth-1);
    int py4=std::min(y+r2,IntegralImage::iHeight-1);

    int iInner=integral->getIntegralValue(px1,py1,px2,py2)*128;
    int iAll=integral->getIntegralValue(px3,py3,px4,py4)*128;
    int areaInner=getArea(px1,py1,px2,py2);
    int areaAll=getArea(px3,py3,px4,py4);
    if(areaInner==0)areaInner=1;
    if(areaAll==0)areaAll=1;
    int rating=iInner/areaInner-iAll/areaAll;
    if(rating>maxRating){
        maxRating=rating;
        maxX=x+stepSize/2;
        maxY=y+stepSize/2;
    }
    ratingImg[x+y*IntegralImage::iWidth]=rating;
    if(stepSize==2){
        ratingImg[x+1+y*IntegralImage::iWidth]=rating;
        ratingImg[x+(y+1)*IntegralImage::iWidth]=rating;
        ratingImg[x+1+(y+1)*IntegralImage::iWidth]=rating;
    } else if(stepSize==4){
        for(int dy=0;dy<4;dy++){
            for(int dx=0;dx<4;dx++){
                ratingImg[(x+dx)+(y+dy)*IntegralImage::iWidth]=rating;
            }
        }
    }
}
}//namespace htwk
