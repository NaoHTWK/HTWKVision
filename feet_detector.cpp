#include <feet_detector.h>

#include <cstdio>
#include <cstdlib>
#include <cstring>

#include <ext_math.h>

using namespace ext_math;
using namespace std;

namespace htwk {

FeetDetector::FeetDetector(int _width, int _height, int * _lutCb, int * _lutCr):
    patternWidth(100), /** TODO: Thomas: why does this not depend on the resolution? */
    patternHeight(30),
    minRating(0.12),
    feetX(_width/2),
    feetY(_height*15/16),
    found(false),
    avgX(feetX),
    avgY(feetY),
    avgAvailable(true),
    width(_width),
    height(_height),
    lutCb(_lutCb),
    lutCr(_lutCr) {
    if (width <= 0 || height <= 0) {
        fprintf(stderr, "Error: tried to allocate a array of length 0 or smaller file %s line %d. Exiting.", __FILE__, __LINE__);
        exit(EXIT_FAILURE);
    }
    createFootPattern();

    feetBorder=(int*)malloc(sizeof(int)*(width/q));
    if (feetBorder == NULL) {
        fprintf(stderr, "Error: malloc() returned NULL in file %s line %d. Exiting.", __FILE__, __LINE__);
        exit(EXIT_FAILURE);
    }
    feetRating=(double*)malloc(sizeof(double)*(width));
    if (feetRating == NULL) {
        fprintf(stderr, "Error: malloc() returned NULL in file %s line %d. Exiting.", __FILE__, __LINE__);
        exit(EXIT_FAILURE);
    }
}

FeetDetector::~FeetDetector(){
    free(feetBorder);
    free(feetRating);
    free(footPattern);
}

//initializes the array 'footPattern' with an approximation of the shape of the nao foot (can later be compared with shapes in the image)
inline void FeetDetector::createFootPattern(){
    footPattern=(int*)malloc(sizeof(*footPattern)*(patternWidth));
    if (footPattern == NULL) {
        fprintf(stderr, "Error: malloc() returned NULL in file %s line %d. Exiting.", __FILE__, __LINE__);
        exit(EXIT_FAILURE);
    }
    int maxHeight=0;
    for(int i=0;i<patternWidth;i++){
        footPattern[i]=(int)(1000*pow(0.01*abs(i-patternWidth/2),2.7));
        if(footPattern[i]>maxHeight){
            maxHeight=footPattern[i];
        }
    }
    for(int i=0;i<patternWidth;i++){
        footPattern[i]=footPattern[i]*patternHeight/maxHeight;
    }
}

//detects the own feets (if visible) and returns the position between the toes
void FeetDetector::proceed(const uint8_t* const img, FieldColorDetector *field, color ball, bool ballFound, bool useDetection){
    found=false;
    if(!useDetection){
        return;
    }

    int *feetBorderRaw=(int*)calloc(1, sizeof(int)*(width/q));
    if (feetBorderRaw == NULL) {
        fprintf(stderr, "Error: malloc() returned NULL in file %s line %d. Exiting.", __FILE__, __LINE__);
        exit(EXIT_FAILURE);
    }

    //detects border between robot contour and green field pixels by scanning the image vertically
    int maxGap=height/32;
    for(int x=0;x<width;x+=q){
        int ballY=height-2;
        int gapCnt=0;
        for(int y=height-2;y>height/2;y--){
//			int i=x+y*width;

            int cy=getY(img,x,y);
            int cb=getCb(img,x,y);
            int cr=getCr(img,x,y);

            if(field->isGreen(cy,cb,cr)){
                gapCnt++;
                if(gapCnt>maxGap){
                    ballY=y+gapCnt;
                    break;
                }
            }else{
                gapCnt=0;
            }
        }
        feetBorderRaw[x/q]=ballY;
    }

    //some median filtering of the detected contour
    for(int x=1;x<width/q-1;x++){
        feetBorder[x]=medianOfThree(feetBorderRaw[x-1],feetBorderRaw[x],feetBorderRaw[x+1]);
    }
    free(feetBorderRaw);

    //calculates a contour rating (determine how good the detected contour matches to the shape of the nao foot)
    double *feetRatingRaw=(double*)malloc(sizeof(double)*(width));
    for(int i=0;i<width;i++){
        feetRatingRaw[i]=1E10;
    }
    for(int i=patternWidth/2;i<width-patternWidth/2;i++){
        int sum=0;
        for(int j=-patternWidth/2;j<patternWidth/2;j++){
            int feetY=feetBorder[(i+j)/q]-feetBorder[i/q];
            int patternY=footPattern[j+patternWidth/2];
            sum+=abs(feetY-patternY);
        }
        feetRatingRaw[i]=((double)sum)/((double)patternWidth)/patternHeight;
    }

    //blur the contour rating results to suppress noise
    int blur=8;
    double scale=1./(blur*2+1);
    for(int i=0;i<width;i++){
        double sum=0;
        for(int di=-blur;di<=blur;di++){
            int ni=i+di;
            if(ni<0)ni=0;
            if(ni>=width)ni=width-1;
            sum+=feetRatingRaw[ni];
        }
        feetRating[i]=sum*scale;
    }
    free(feetRatingRaw);

    //peak detection of the rating function (in the best case, there are 2 peaks....top of the right and left nao foot)
    //all detected peek coordinates are saved into 'footPoints'
    int r=patternWidth/2;
    float headYaw=0.0;
    vector<point_2d> footPoints;
    for(int i=r*2;i<width-r*2;i++){
        if(feetRating[i]>minRating)continue;
        bool isMin=true;
        for(int di=-r;di<=r;di++){
            if(di==0)continue;
            if(feetRating[i+di]<=feetRating[i]){
                isMin=false;
                break;
            }
        }
        if(isMin){
            float angle=-(float)(i-width/2)/(width/2)*0.4+headYaw;
            int px=(int)(i+sin(angle)*patternHeight);
            int py=(int)(feetBorder[i/q]+patternHeight-patternHeight*cos(angle));
            point_2d p;
            p.x=px;
            p.y=py;
            footPoints.push_back(p);
        }
    }

    //test the distance between all peak combinations
    //and only choose two of them, if they have a plausible distance (mDist) to each other with an distance error less than 'minError'
    int mDist=160;
    float minError=50;
    int bestX=0;
    int bestY=0;
    for(const point_2d &p1 : footPoints){
        for(const point_2d &p2 : footPoints){
            int dx=(int)(p1.x-p2.x);
            int dy=(int)(p1.y-p2.y);
            float dist=sqrtf(dx*dx+dy*dy);
            int midX=(p1.x+p2.x)/2;
            int midY=(p1.y+p2.y)/2;
            float error=fabsf(dist-mDist);
            if(error<minError){
                minError=error;
                bestX=midX;
                bestY=midY;
                found=true;
            }
        }
    }
    feetX=bestX;
    feetY=bestY;
    if(found){
        avgX=avgX*.75f+feetX*.25f;
        avgY=avgY*.75f+feetY*.25f;
    }
}

bool FeetDetector::isAvailable() const {
    return found||avgAvailable;
}

bool FeetDetector::isInterpolated() const {
    return !found ;
}

//gets the point between the right and left top of the nao feet (between the toes)
point_2d FeetDetector::getBase() const {
    if(found){
        point_2d ret=newPoint2D(feetX,feetY);
        return ret;
    }else{
        point_2d ret=newPoint2D(avgX,avgY);
        return ret;
    }
}

}  // namespace htwk
