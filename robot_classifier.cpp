#include <robot_classifier.h>

#include <cstdio>
#include <cstdlib>
#include <cstring>

#include <ext_math.h>

using namespace ext_math;
using namespace std;

namespace htwk {

RobotClassifier::RobotClassifier(int width, int height, int *lutCb,
        int *lutCr) : width(width), height(height) {
    this->lutCb = lutCb;
    this->lutCr = lutCr;
    this->histolength = (256 * 2) >> HISTOGRAM_SHIFT_GRADIENT;
    robotClassifierNN=new Classifier("../data/net_Robots_Martin_1P2N+Small_do0.5_hidden4_bs3000_bi64_iter1000_l1.0E-6.net");
}


RobotClassifier::~RobotClassifier() {
}


void RobotClassifier::proceed(const uint8_t * const img, Rect r, classifierResult &result) {
    result.rect = r;
    ycbcr32_t normImage[NORMWIDTH * NORMHEIGHT];

    generateNormImage(img, r, normImage);

    result.features.color_mean_squared_deviation_Y_channel=calcMeanDeviation(normImage);
    float *gradHist=getGradientHistogramm_Y_Direction(normImage);
    result.features.entropy_Y = getGradientEntropy_Y_Direction(gradHist, histolength);
    free(gradHist);
    float **colHist=getColorHistograms(normImage);
    result.features.entropy_cl_Y_channel = getColorEntropy(colHist, 256 >> HISTOGRAM_SHIFT_COLOR);
    for(int i=0;i<3;i++)
        free(colHist[i]);
    free(colHist);
    result.features.entropy_linesum_X = getGradientEntropy_normLineSum_X_Direction(normImage);
    result.features.entropy_linesum_X_signed = getGradientEntropy_normLineSum_X_Direction_Signed(normImage);
    result.features.entropy_linesum_Y_signed = getGradientEntropy_normLineSum_Y_Direction_Signed(normImage);
    result.features.entropy_X_signed = getGradientEntropySigned_X_Direction(normImage);
    result.features.entropy_Y_signed = getGradientEntropySigned_Y_Direction(normImage);

    result.isRobot= isRobot(result);
    if(result.isRobot){
        result.teamColor=determineTeamColor(img,r);
    }

}


TeamMembership RobotClassifier::determineTeamColor(const uint8_t * const img,Rect r) const {
    int mid=0,cnt=0;
    int maxCb=0,maxCr=0;
    int stepSizeX=max(2,(r.xRight-r.xLeft)/18);
    int stepSizeY=max(2,(r.yBottom-r.yTop)/52);
    int lastSumCb=0;
    int lastSumCr=0;
    for(int dy=(r.yTop*4)/5+r.yBottom/5;dy<=r.yTop/4+(r.yBottom*3)/4;dy+=stepSizeY){
        int sumCb=0;
        int sumCr=0;
        for(int dx=(r.xLeft*3)/4+r.xRight/4;dx<=r.xLeft/4+(r.xRight*3)/4;dx+=stepSizeX){
            //use these on the robot
            sumCb+=-3*(int)getY(img,dx,dy)+16*(int)getCb(img,dx,dy)+3*(int)getCr(img,dx,dy);
            sumCr+=-1*(int)getY(img,dx,dy)+2*(int)getCb(img,dx,dy)+16*(int)getCr(img,dx,dy);
            //use these when analyzing png
//			sumCb+=-3*(int)img[(dx+dy*width)*3]+16*(int)img[(dx+dy*width)*3+1]+3*(int)img[(dx+dy*width)*3+2];
//			sumCr+=-1*(int)img[(dx+dy*width)*3]+2*(int)img[(dx+dy*width)*3+1]+16*(int)img[(dx+dy*width)*3+2];
        }
        int minSumCb=min(lastSumCb,sumCb);
        int minSumCr=min(lastSumCr,sumCr);
        mid+=minSumCb-minSumCr;
        cnt++;
        if(minSumCb>maxCb)
            maxCb=minSumCb;
        if(minSumCr>maxCr)
            maxCr=minSumCr;
        lastSumCr=sumCr;
        lastSumCb=sumCb;
    }
    if(cnt==0)
        return TeamMembership::NONE;//Rechteck zu schmal
    return ((maxCb-maxCr)-mid/cnt)>0?TeamMembership::BLUE:TeamMembership::RED;
}


float RobotClassifier::getGradientEntropy_normLineSum_Y_Direction_Signed(
        const ycbcr32_t *normImage) {
    int cnt = 0;
    const int length = (256 * 2) >> HISTOGRAM_SHIFT_GRADIENT;
    int grad_hist[length];
    memset(grad_hist, 0, sizeof(grad_hist));
    int sum_lineTwo = normSum_Y_Line(0, normImage);
    for (int x = 2; x < NORMWIDTH; x +=2) {
        cnt++;
        int sum_lineOne = sum_lineTwo;
        sum_lineTwo = normSum_Y_Line(x, normImage);
        grad_hist[(sum_lineOne-sum_lineTwo+256) >> HISTOGRAM_SHIFT_GRADIENT]++;
    }
    float *normHist=normHistogram(grad_hist, cnt, length);
    getBackTheSignedHist(normHist,length);
    float ret=processEntropy(normHist, length);
    free(normHist);
    return ret;
}


float RobotClassifier::getGradientEntropySigned_X_Direction(
        const ycbcr32_t *normImage) {
    float* hist = getBackTheSignedHist(
            getGradientHistogramm_X_Direction(normImage), histolength);
    float entropy = 0;
    for (int i = 0; i < histolength; i++) {
        if (hist[i] > 0)
            entropy += hist[i] * log2f(hist[i]);
    }
    free(hist);
    return -entropy;
}


float* RobotClassifier::getGradientHistogramm_X_Direction(
        const ycbcr32_t *normImage) {
    int hist[histolength];
    memset(hist,0,sizeof(hist));
    int cnt = 0;
    for (int y = 0; y < NORMHEIGHT; y++) {
        for (int x = 0; x < NORMWIDTH-1; x++) {
            cnt++;
            hist[(normImage[x+y*NORMWIDTH].y-normImage[x+1+y*NORMWIDTH].y+256)
                    >> HISTOGRAM_SHIFT_GRADIENT]++;
        }
    }
    return normHistogram(hist, (NORMWIDTH-1)*NORMHEIGHT, histolength);
}


float RobotClassifier::getGradientEntropySigned_Y_Direction(
        const ycbcr32_t *normImage) {
    float* hist = getBackTheSignedHist(
            getGradientHistogramm_Y_Direction(normImage), histolength);
    float entropy = 0;
    for (int i = 0; i < histolength; i++) {
        if (hist[i] > 0)
            entropy += hist[i]*log2f(hist[i]);
    }
    free(hist);
    return -entropy;
}


float RobotClassifier::getGradientEntropy_normLineSum_X_Direction_Signed(
        const ycbcr32_t *normImage) {
    int cnt = 0;
    const int length = (256 * 2) >> HISTOGRAM_SHIFT_GRADIENT;
    int grad_hist[length];
    memset(grad_hist, 0, sizeof(grad_hist));
    int sum_lineTwo = normSum_X_Line(0, normImage);
    for (int y = 2; y < NORMHEIGHT; y+=2) {
        cnt++;
        int sum_lineOne = sum_lineTwo;
        sum_lineTwo = normSum_X_Line(y, normImage);
        grad_hist[(sum_lineOne-sum_lineTwo+256) >> HISTOGRAM_SHIFT_GRADIENT]++;
    }
    float *normHist=normHistogram(grad_hist, cnt, length);
    getBackTheSignedHist(normHist,length);
    float ret=processEntropy(normHist, length);
    free(normHist);
    return ret;
}


float RobotClassifier::getGradientEntropy_normLineSum_X_Direction(
        const ycbcr32_t *normImage) {
    int cnt = 0;
    const int length = (256 * 2) >> HISTOGRAM_SHIFT_GRADIENT;
    int grad_hist[length];
    memset(grad_hist, 0, sizeof(grad_hist));
    int sum_lineTwo = normSum_X_Line(0, normImage);
    for (int y = 2; y < NORMHEIGHT; y+=2) {
        cnt++;
        int sum_lineOne = sum_lineTwo;
        sum_lineTwo = normSum_X_Line(y, normImage);
        grad_hist[(sum_lineOne-sum_lineTwo+256) >> HISTOGRAM_SHIFT_GRADIENT]++;
    }
    float *normHist=normHistogram(grad_hist, cnt, length);
    float ret=processEntropy(normHist, length);
    free(normHist);
    return ret;
}


float *RobotClassifier::getBackTheSignedHist(float* hist, int length) {
    for (int i = 0; i < length / 2; i++) {
        hist[i] = -hist[i];
    }
    return hist;
}


float RobotClassifier::processEntropy(float* grad_hist, int length) {
    float entropy = 0;
    for (int i = 0; i < length; i++) {
        if (grad_hist[i] > 0)
            entropy += grad_hist[i] * log2f(grad_hist[i]);
    }
    return -entropy;
}


int RobotClassifier::normSum_X_Line(int y,
        const ycbcr32_t *normImage) {
    int sum = 0;
    if(y>=NORMHEIGHT){
        perror("uebergebenes Argument groesser Normheight");
        exit(-1);
    }
    for (int x = 1; x < NORMWIDTH; x++) {
        sum += normImage[x + y * NORMWIDTH].y;
    }
    return (int) ((float) sum / (float) (NORMWIDTH-1));
}


int RobotClassifier::normSum_Y_Line(int x,
        const ycbcr32_t *normImage) {
    int sum = 0;
    if(x>=NORMWIDTH){
            perror("uebergebenes Argument groesser Normwidtlsdkfj");
            exit(-1);
    }
    for (int y = 1; y < NORMHEIGHT; y++) {
        sum += normImage[x + y * NORMWIDTH].y;
    }
    return (int) ((float) sum / (float) (NORMHEIGHT-1));
}


void RobotClassifier::generateNormImage(const uint8_t * const img, Rect r,
        ycbcr32_t * const normImage) const {
    int idx=0;

    for(int py=0;py<NORMHEIGHT;py++){
        for(int px=0;px<NORMWIDTH;px++){
            int x=(int)(round((NORMWIDTH-px-1)*r.xLeft/(NORMWIDTH-1)+px*r.xRight/(NORMWIDTH-1)));
            int y=(int)(round((NORMHEIGHT-py-1)*(r.yTop*2+r.yBottom)/3/(NORMHEIGHT-1)+py*r.yBottom/(NORMHEIGHT-1)));
            if(x<0||y<0||x>=width||y>=height){
                idx++;
                continue;
            }
            normImage[idx].y=getY(img,x,y);
            idx++;
        }
    }
    return;
}
/***
 * fills vector a with value val
 * @param: a: vector, max: length of vector, val: int value to fill in vector
 */

void RobotClassifier::fill(int * a, int val, int max) {
    for (int j = 0; j < max; j++) {
        a[j] = val;
    }
}

float RobotClassifier::getGradientEntropy_Y_Direction(float* hist,int histlength) {
    float entropy = 0;
    for (int i = 0; i < histlength; i++) {
        if (hist[i] > 0)
            entropy += hist[i] * log2f(hist[i]);
    }
    return -entropy;
}


float* RobotClassifier::getGradientHistogramm_Y_Direction(const ycbcr32_t *normImage){
    int hist[histolength];
    memset(hist,0,sizeof(hist));
    for (int x = 0; x < (NORMHEIGHT-1)*NORMWIDTH; x++) {
        hist[(normImage[x].y-normImage[x+NORMWIDTH].y+256)>> HISTOGRAM_SHIFT_GRADIENT]++;
    }
    return normHistogram(hist, NORMWIDTH*(NORMHEIGHT-1), histolength);
}


float* RobotClassifier::normHistogram(int* histo, int cnt, int histolength) {
    float* hist = (float*) calloc(sizeof(float), histolength);
    for (int i = 0; i < histolength ; i++) {
        hist[i] = (float)histo[i] / (float)cnt;
    }
    return hist;
}


float** RobotClassifier::normHistogram(int** histo, int cnt, int histolength) {
    float**hist = (float**) calloc(sizeof(float*), 3);
    for (int i = 0; i < 3; i++) {
        hist[i] = normHistogram(histo[i], cnt, histolength);
    }
    return hist;
}

float RobotClassifier::getColorEntropy(float** hist, int length) {
    float entropy = 0;

    for (int i = 0; i < length; i++) {
        if (hist[0][i] > 0)
            entropy += hist[0][i] * log2f(hist[0][i]);
    }
    return -entropy;
}


float** RobotClassifier::getColorHistograms(const ycbcr32_t *normImage) {
    int**histograms = (int**) calloc(sizeof(int*), 3);
    int length = 256 >> HISTOGRAM_SHIFT_COLOR;
    histograms[0] = (int*) calloc(sizeof(int), length);
    histograms[1] = (int*) calloc(sizeof(int), length);
    histograms[2] = (int*) calloc(sizeof(int), length);
    for (int x = 0; x < NORMHEIGHT*NORMWIDTH; x++) {
        histograms[0][normImage[x].y >> HISTOGRAM_SHIFT_COLOR]++;
    }
    float **ret=normHistogram(histograms, NORMHEIGHT*NORMWIDTH, length);
    for(int i=0;i<3;i++)
        free(histograms[i]);
    free(histograms);
    return ret;
}


float RobotClassifier::calcColorMean(const ycbcr32_t *img) {
    float sum = 0;
    for(uint32_t x = 0; x < NORMWIDTH*NORMHEIGHT; x++) {
        sum += img[x].y;
    }
    return sum/(NORMWIDTH*NORMHEIGHT);
}


float RobotClassifier::calcMeanDeviation(const ycbcr32_t * const img) {
    float mean = calcColorMean(img);
    float dev=0.f;
    for(uint32_t x = 0; x < NORMWIDTH*NORMHEIGHT; x++) {
        float dm=img[x].y-mean;
        dev+=dm*dm;
    }
    return dev/(NORMWIDTH*NORMHEIGHT);
}


void RobotClassifier::norm(float *m,int numFeatures) {
    float mean=0;
    for (int i = 0; i < numFeatures; i++) {
        mean+=m[i];
    }
    mean/=numFeatures;
    double var=0;
    for (int i = 0; i < numFeatures; i++) {
        m[i]-=mean;
        var+=m[i]*m[i];
    }
    var=sqrtf(var/numFeatures);
    for (int i = 0; i < numFeatures; i++){
        m[i]/=var;
    }
}


bool RobotClassifier::isRobot(classifierResult result) {
    Rect r = result.rect;
    float cwidth = r.xRight - r.xLeft;
    float cheight = r.yBottom - r.yTop;
    vec2df features=createVec2df(10,1);
    features[0][0]=result.features.entropy_Y;
    features[1][0]=result.features.entropy_Y_signed;
    features[2][0]=(result.features.entropy_X_signed - result.features.entropy_Y_signed);
    features[3][0]=result.features.entropy_cl_Y_channel;
    features[4][0]=result.features.color_mean_squared_deviation_Y_channel;
    features[5][0]=result.features.entropy_linesum_X * result.features.entropy_linesum_X_signed;
    features[6][0]=result.features.entropy_linesum_Y_signed * result.features.entropy_linesum_Y_signed;
    features[7][0]=cwidth;
    features[8][0]=cheight;
    features[9][0]=cwidth * cheight;
    float p=robotClassifierNN->proceed(features)[0][0];
    return p>0.5f;
}

}  // namespace htwk
