#include "ball_feature_extractor.h"

#include <cmath>
#include <cstring>

namespace htwk {

#define fast_round(x) (((int)(x+100.5f))-100)

BallFeatureExtractor::BallFeatureExtractor(const int _width, const int _height, const int8_t* _lutCb, const int8_t* _lutCr)
    : BaseDetector(_width, _height, _lutCb, _lutCr)
{
}

void BallFeatureExtractor::getFeature(const ObjectHypothesis &p, const uint8_t *img, const int _featureSize, float* dest)
{
    const int featureSize=_featureSize;
    const float scale=p.r*FEATURE_SCALE/(featureSize/2);

    int cyValues[featureSize*featureSize];
    int cnt[featureSize*featureSize];
    memset(cyValues,0,sizeof(int)*featureSize*featureSize);
    memset(cnt,0,sizeof(int)*featureSize*featureSize);

    for(int dy=-featureSize;dy<featureSize;dy++){
        for(int dx=-featureSize;dx<featureSize;dx++){
            float vx=dx;
            float vy=dy;
            int x=p.x+fast_round(vx*scale*0.5f);
            int y=p.y+fast_round(vy*scale*0.5f);
            if(x<0||y<0||x>=width||y>=height){
                continue;
            }
            int addr=((dx+featureSize)/2)+((dy+featureSize)/2)*featureSize;
            cnt[addr]++;
            cyValues[addr]+=getY(img,x,y);
        }
    }

    postprocessFeature(featureSize, cnt, cyValues, dest);
}

void BallFeatureExtractor::getModifiedFeature(const ObjectHypothesis &p, const uint8_t *img, const int featureSize, float* dest, const bool mirrored, const float rotation)
{
    const float scale=p.r*FEATURE_SCALE/(featureSize/2);

    int cyValues[featureSize*featureSize];
    int cnt[featureSize*featureSize];
    memset(cyValues,0,sizeof(int)*featureSize*featureSize);
    memset(cnt,0,sizeof(int)*featureSize*featureSize);

    for(int dy=-featureSize;dy<featureSize;dy++){
        for(int dx=-featureSize;dx<featureSize;dx++){
            float vx=dx*cosf(rotation)-dy*sinf(rotation);
            float vy=dx*sinf(rotation)+dy*cosf(rotation);
            int x=p.x+fast_round((mirrored ? -1 : 1)*vx*scale*0.5f);
            int y=p.y+fast_round(vy*scale*0.5f);
            if(x<0||y<0||x>=width||y>=height){
                continue;
            }
            int addr=((dx+featureSize)/2)+((dy+featureSize)/2)*featureSize;
            cnt[addr]++;
            cyValues[addr]+=getY(img,x,y);
        }
    }

    postprocessFeature(featureSize, cnt, cyValues, dest);
}

void BallFeatureExtractor::postprocessFeature(const int featureWidth, const int* cnt, int* cyValues, float* dest)
{
    int meanCnt=0;
    int hist[HIST_SIZE];
    memset(hist,0,sizeof(int)*HIST_SIZE);
    for(int i=0;i<featureWidth*featureWidth;i++){
        if(cnt[i]<1)continue;
        cyValues[i]/=cnt[i];
        hist[cyValues[i]]++;
        meanCnt++;
    }

    float qMin=getQ(hist,HIST_SIZE,0.05f);
    float qMax=getQ(hist,HIST_SIZE,0.95f);
    float meanCy=(qMin+qMax)*0.5f;
    float var=(qMax-qMin)*0.25f;

    float varInv=32.f/(32+var);
    for(int i=0;i<featureWidth*featureWidth;i++){
        if(cnt[i]<1)
            dest[i] = 0.0f;
        else
            dest[i] = (cyValues[i]-meanCy)*varInv;
    }
}

int BallFeatureExtractor::getQ(int *hist, int size, float d)
{
    int sum=0;
    for(int i=0;i<size;i++){
        sum+=hist[i];
    }
    sum*=d;
    int sumQ=0;
    for(int i=0;i<size;i++){
        sumQ+=hist[i];
        if(sumQ>=sum){
            return i;
        }
    }
    return 0;
}

} // htwk
