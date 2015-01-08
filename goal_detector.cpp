#include <goal_detector.h>

#include <cstring>

#include <ext_math.h>

using namespace ext_math;
using namespace std;

namespace htwk {

const int GoalDetector::maxGoalWidth=320;
const int GoalDetector::minGoalWidth=8;
const int GoalDetector::minGoalHeight=90;
const int GoalDetector::scanlineCnt=16;
const int GoalDetector::q=2;
const float GoalDetector::minWidthToHeightAspect=2;
const int GoalDetector::minGoalPostToBorderDist=4;

GoalDetector::GoalDetector(int width, int height, int *lutCb, int *lutCr){
    this->lutCb=lutCb;
    this->lutCr=lutCr;
    this->width=width;
    this->height=height;

    goalColor.cy=255;
    goalColor.cb=50;
    goalColor.cr=128;
}

GoalDetector::~GoalDetector(){
}

void GoalDetector::proceed(const uint8_t * const img, const int * const fieldborder, color green, color white){
    int goalPostHeight=getGoalPostHeight(fieldborder);
    goalColor.cy=255;
    goalColor.cb=50;
    goalColor.cr=128;
    goalPosts.clear();
    int groundHeight=max(goalPostHeight,scanlineCnt*4);
    const int scanlineHeight=max(groundHeight-minGoalHeight,0);

    float stepWidth=max(1.f,(float)(groundHeight-scanlineHeight)/(scanlineCnt));
    int midHeight=(scanlineHeight+groundHeight)/2;

    int **hist=getEdgeHistogramm(img,scanlineHeight,groundHeight,stepWidth);
    int edgeThreshold=(int)(scanlineCnt*0.25f*scanlineCnt);

    const int histLen0=width/q;
    const int histLen1=(int)((groundHeight-scanlineHeight)/stepWidth);
    int histSum[histLen0];
    for(int x=0;x<histLen0;x++){
        int sum=0;
        for(int i=0;i<histLen1;i++){
            sum+=hist[x][i];
        }
        histSum[x]=sum;
    }


    int histMax[histLen0];
    memset(histMax,0,sizeof(int)*histLen0);
    getMaxSup(histMax,histSum,edgeThreshold);
    int lastGoalX=-1;

    for(int x=2;x<histLen0-2;x++){
        if(histMax[x]!=1)
            continue;
        if(lastGoalX>=0&&x-lastGoalX<maxGoalWidth/q&&x-lastGoalX>minGoalWidth/q&&histSum[lastGoalX]-histSum[x]<edgeThreshold*2){

            //Torkanten bestimmen
            int f=1;
            if(histSum[x]<0)
                f=-1;
            int r=3;
            int maxPoints[scanlineCnt];
            getMaxPoints(maxPoints,hist,histSum,x,r,f);
            bool found=true;
            pair<float,float> lineR=calcLine(scanlineHeight,stepWidth,hist,f,maxPoints,found);
            if(!found){
                lastGoalX=x;
                continue;
            }

            f=1;
            if(histSum[lastGoalX]<0)
                f=-1;
            getMaxPoints(maxPoints,hist,histSum,lastGoalX,r,f);
            pair<float,float> lineL=calcLine(scanlineHeight,stepWidth,hist,f,maxPoints,found);
            if(!found){
                lastGoalX=x;
                continue;
            }

            float bestM=(lineL.first+lineR.first)/2;
            float bestN=(lineL.second+lineR.second)/2;


            //Torfarbe bestimmen
            int histU[256];
            int histV[256];
            memset(histU,0,sizeof(histU));
            memset(histV,0,sizeof(histV));
            int sum=0;
            for(int ny=0;ny<scanlineCnt*stepWidth;ny+=4){
                int py=(int)(scanlineHeight+ny);
                int sx1=(int)(lineL.first*ny+lineL.second);
                int sx2=(int)(lineR.first*ny+lineR.second);
                for(int px=max(0,sx1+2);px<=sx2-2&&px*2<width;px++){//TODO: Thomas, temporary fix
                    int cu=getCb(img,px*2,py);
                    histU[cu]++;
                    int cv=getCr(img,px*2,py);
                    histV[cv]++;
                    sum++;
                }
            }
            int goalU=getMax(histU);
            int goalV=getMax(histV);

            int breite=(int)(lineR.second-lineL.second)*q;

            //PfostenBoden finden
            int postYCoord=findPostYCoord(img,green, white, scanlineHeight,midHeight,bestM,bestN,goalU,goalV,breite);
            const int p=clamp(0, (int)(2*(bestM*(postYCoord-scanlineHeight)+bestN)), width-1);

            float aspect=(float)(postYCoord-scanlineHeight)/breite;
            if(aspect<minWidthToHeightAspect){
                lastGoalX=x;
                continue;
            }
            if(fieldborder[p]+minGoalPostToBorderDist>postYCoord){
                lastGoalX=x;
                continue;
            }

            GoalPost goalPost=newGoalPost(p, postYCoord);
            goalColor.cy=getY(img,x+lastGoalX,midHeight);
            goalColor.cb=goalU;
            goalColor.cr=goalV;

            goalPost.lineR[0]=lineR.first;
            goalPost.lineR[1]=lineR.second;
            goalPost.lineL[0]=lineL.first;
            goalPost.lineL[1]=lineL.second;
            goalPost.width=breite;
            goalPost.height[0]=scanlineHeight;
            goalPost.height[1]=min(height-1,postYCoord);

            goalPosts.push_back(goalPost);

        }
        lastGoalX=x;
    }
    for(int i=0;i<histLen0;i++){
        free(hist[i]);
    }
    free(hist);
}

vector<GoalPost> &GoalDetector::getGoalPosts(){
    return goalPosts;
}

int GoalDetector::getGoalPostHeight(const int * const fieldborder) const {
    int min=height-2;
    for(int i=0;i<width;i++){
        if(fieldborder[i]<min){
            min=fieldborder[i];
        }
    }
    return max(0,min);
}

int GoalDetector::findPostYCoord(const uint8_t * const img, color green, color white, int scanlineHeight,
                                 int midHeight, float bestM, float bestN, int goalU, int goalV, int breite) const {
    int greenU=green.cb;
    int greenV=green.cr;
    int whiteU=white.cb;
    int whiteV=white.cr;
    int postBaseHeight=midHeight;
    int r=4;
    for(;postBaseHeight<height-2;postBaseHeight+=max(2,breite/8)){
        int nx=(int)(2*(bestM*(postBaseHeight-scanlineHeight)+bestN));
        int sum=0;
        int cu=0;
        int cv=0;
        for(int dx=-r;dx<=r;dx+=2){
            int tx=nx+dx;
            if(tx<0)continue;
            if(tx>=width)continue;
            cu+=getCb(img,tx,postBaseHeight);
            cv+=getCr(img,tx,postBaseHeight);
            sum++;
        }
        if(sum==0)continue;
        cu/=sum;
        cv/=sum;
        int diffGoalU=goalU-cu;
        int diffGoalV=goalV-cv;
        int diffGoal=diffGoalU*diffGoalU+diffGoalV*diffGoalV;
        int diffGreenU=greenU-cu;
        int diffGreenV=greenV-cv;
        int diffGreen=diffGreenU*diffGreenU+diffGreenV*diffGreenV;
        if(diffGoal>diffGreen)break;
        int diffWhiteU=whiteU-cu;
        int diffWhiteV=whiteV-cv;
        int diffWhite=diffWhiteU*diffWhiteU+diffWhiteV*diffWhiteV;
        if(diffGoal>diffWhite)break;
    }
    postBaseHeight=max(0,postBaseHeight-max(2,breite/8));
    for(;postBaseHeight<height-2;postBaseHeight++){
        int nx=(int)(2*(bestM*(postBaseHeight-scanlineHeight)+bestN));
        int sum=0;
        int cu=0;
        int cv=0;
        for(int dx=-r;dx<=r;dx+=2){
            int tx=nx+dx;
            if(tx<0)continue;
            if(tx>=width)continue;
            cu+=getCb(img,tx,postBaseHeight);
            cv+=getCr(img,tx,postBaseHeight);
            sum++;
        }
        if(sum==0)continue;
        cu/=sum;
        cv/=sum;
        int diffGoalU=goalU-cu;
        int diffGoalV=goalV-cv;
        int diffGoal=diffGoalU*diffGoalU+diffGoalV*diffGoalV;
        int diffGreenU=greenU-cu;
        int diffGreenV=greenV-cv;
        int diffGreen=diffGreenU*diffGreenU+diffGreenV*diffGreenV;
        if(diffGoal>diffGreen)break;
        int diffWhiteU=whiteU-cu;
        int diffWhiteV=whiteV-cv;
        int diffWhite=diffWhiteU*diffWhiteU+diffWhiteV*diffWhiteV;
        if(diffGoal>diffWhite)break;
    }
    return postBaseHeight;
}

pair<float,float> GoalDetector::calcLine(int scanlineHeight, float stepWidth, int **hist, int f, int* maxPoints, bool &found){
    float bestM=0;
    float bestN=0;
    int max=0;
    for(int d=scanlineCnt/2;d<scanlineCnt;d++){
        for(int py1=0;py1<scanlineCnt-d;py1++){
            int py2=py1+d;
            int px1=maxPoints[py1];
            int px2=maxPoints[py2]+(py1&1);
            if(px1<2||px2<2)continue;
            int dx=px1-px2;
            int dy=py1-py2;
            float m=(float)dx/dy;
            if(fabsf(m)>1)continue;
            float n=px1-m*py1;
            int sum=0;
            for(int ny=0;ny<scanlineCnt;ny++){
                int nx=(int)(m*ny+n+0.5);
                if(nx<2||nx>=width/q-2)continue;
                sum+=f*(hist[nx-1][ny]+2*hist[nx][ny]+hist[nx+1][ny]);
            }
            if(sum>max){
                max=sum;
                bestM=m;
                if(n<1)n=1;
                if(n>=width/q-1)n=width/q-2;
                bestN=n;
            }

        }
    }
    if(max<300){
        found=false;
        return pair<float,float>();
    }
    found=true;
    bestM/=stepWidth;
    return pair<float,float>(bestM,bestN);
}

void GoalDetector::getMaxPoints(int* const maxPoints, int ** hist, int* histSum, int x, int r, int f) const {
    for(int py=0;py<scanlineCnt;py++){
        int max=0;
        int bestPX=0;
        for(int dx=-r;dx<=r;dx++){
            int px=x+dx;
            if(px<0)continue;
            if(px>=width/q)continue;
            int value=f*hist[px][py];
            if(value>max){
                max=value;
                bestPX=px;
            }
        }
        maxPoints[py]=bestPX;
    }
}

int GoalDetector::getMax(int hist[256]) const {
    int max=0;
    int best=0;
    for(int i=0;i<256;i++){
        int value=hist[i];
        if(value>max){
            max=value;
            best=i;
        }
    }
    return best;
}

void GoalDetector::getMaxSup(int* histMax, int* hist, int threshold) const {
    const int r=minGoalWidth;
    for(int x=1;x<width/q-1;x++){
        if(abs(hist[x])<threshold)continue;
        bool ismax=true;
        int value1=(abs(hist[x-1]+2*hist[x]+hist[x+1]))*256;
        for(int dx=-r;dx<=r;dx++){
            if(dx==0)continue;
            int nx=x+dx;
            if(nx<1||nx>=width/q-1)continue;
            int f=256*(r-max(0,abs(dx)-1))/r;
            int value2=(abs(hist[nx-1]+2*hist[nx]+hist[nx+1]))*f;
            if((value2>=value1&&dx<0)||value2>value1){
                ismax=false;
                break;
            }
        }
        if(ismax){
            histMax[x]=1;
            x+=minGoalWidth/q;
        }else
            histMax[x]=0;
    }
}

int **GoalDetector::getEdgeHistogramm(const uint8_t * const img,int sy, int ey, float dy){
    size_t memsize=sizeof(int*)*(width/q);
    int **hist=(int**)calloc(1, memsize);
    for(int i=0;i<width/q;i++)
        hist[i]=(int*)calloc((int)((ey-sy)/dy),sizeof(int));
    for(int x=4;x<width-2;x+=q){
        for(int y=0;y<scanlineCnt;y++){
            int py=sy+(ey-sy)*y/(scanlineCnt-1);
            int f=getCb(img,x+2,py)-getCb(img,x-2,py);
            int edgeThreshold=4;
            if(abs(f)>edgeThreshold){
                hist[x/q][y]=f;
            }
        }
    }
    return hist;
}

}  // namespace htwk
