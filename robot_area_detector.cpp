#include <robot_area_detector.h>

#include <assert.h>
#include <x86intrin.h>

#include <chrono>
#include <cstdlib>
#include <cstring>
#include <iostream>



using namespace std;
using namespace std::chrono;
namespace htwk {

RobotAreaDetector::RobotAreaDetector(int _width, int _height, int8_t *lutCb, int8_t *lutCr, int scanlineCnt)
    : BaseDetector(_width, _height, lutCb, lutCr)
{
    this->lutCb=lutCb;
    this->lutCr=lutCr;

    this->scanlineCnt=scanlineCnt;
    this->robotAreas=new vector<RobotRect>();

    fieldMapWidth=width/fieldMapScale;
    fieldMapHeight=height/fieldMapScale;
    assert(fieldMapWidth%16 == 0);
    fieldMapWidthSSE=fieldMapWidth/16;
    fieldMapIntegral=(int*)malloc(sizeof(int)*(fieldMapWidth*fieldMapHeight));

    int rc0 = posix_memalign((void**)&yout, 16, fieldMapWidthSSE*fieldMapHeight*sizeof(*yout));
    int rc1 = posix_memalign((void**)&uout, 16, fieldMapWidthSSE*fieldMapHeight*sizeof(*uout));
    int rc2 = posix_memalign((void**)&vout, 16, fieldMapWidthSSE*fieldMapHeight*sizeof(*vout));
    int rc3 = posix_memalign((void**)&fieldMap, 16, fieldMapWidth*fieldMapHeight*sizeof(*fieldMap));

    if((rc0|rc1|rc2|rc3) != 0) {
        printf("Error allocating memory for SSE!\n");
        exit(1);
    }

}

double RobotAreaDetector::proceed(uint8_t * const img, Scanline *scanVertical, const int * const fieldborder, FieldColorDetector *field, const Ball &ball, const vector<GoalPost> &goalPosts){

    vector<RobotRect> hypotheses=searchRobotHypotheses(scanVertical,fieldborder);

    high_resolution_clock::time_point t1 = high_resolution_clock::now();
//    for(int i = 0; i < 10000; i++)
        createGreenMap(fieldborder,img,field);
    high_resolution_clock::time_point t2 = high_resolution_clock::now();
    duration<double> time_span = duration_cast<duration<double>>(t2 - t1);
//    std::cout << "createGreenMap time: " << time_span.count() * 1000 << "ms\n";

    createIntegralImage();
    hypotheses=setCarpetFeatures(hypotheses);

    robotAreas->clear();
    vector<RobotRect> best;
    while(!hypotheses.empty()){
        float maxProb=-numeric_limits<float>::infinity();
        RobotRect bestRect;
        vector<RobotRect> hypothesesLeft;
        for(RobotRect r : hypotheses) {
            if(r.detectionProbability>maxProb){
                maxProb=r.detectionProbability;
                bestRect=r;
            }else{
                hypothesesLeft.push_back(r);
            }
        }
        best.push_back(bestRect);
        hypotheses.clear();
        for(RobotRect r : hypothesesLeft) {
            float midX=(bestRect.xLeft+bestRect.xRight)*0.5f;
            if(r.xLeft>(midX+bestRect.xRight)*0.5f||r.xRight<(midX+bestRect.xLeft)*0.5f){
                hypotheses.push_back(r);
            }
        }
    }
    for(RobotRect r : best) {
        r.yTop=max(0,r.yBottom-(r.xRight-r.xLeft)*3);
        robotAreas->push_back(r);
    }

    return time_span.count()*1000;
}

vector<RobotRect> RobotAreaDetector::setCarpetFeatures(vector<RobotRect> hypotheses){
    vector<RobotRect> hypothesesNew;
    for(RobotRect r : hypotheses) {
        int xLeft=r.xLeft/fieldMapScale;
        int yTop=r.yTop/fieldMapScale;
        int xRight=r.xRight/fieldMapScale;
        int yBottom=r.yBottom/fieldMapScale;
        int yBottom2=yBottom+(yBottom-yTop)/8+4;
        int xLeft2=xLeft-(xRight-xLeft)/3;
        int xRight2=xRight+(xRight-xLeft)/3;
        float center=getIntegralValue(xLeft,yTop,xRight,yBottom);
        float left=getIntegralValue(xLeft2,yTop,xLeft,yBottom);
        float right=getIntegralValue(xRight,yTop,xRight2,yBottom);
        float bottom=min(128.f,getIntegralValue(xLeft,yBottom,xRight,yBottom2));
        r.detectionProbability=(left+right+2*bottom)/3-center;
        r.greenCenterRatio=center;
        r.greenSideRatio=left+right;
        r.greenBottomRatio=bottom;
        hypothesesNew.push_back(r);
    }
    return hypothesesNew;
}

float RobotAreaDetector::getIntegralValue(int px1, int py1, int px2, int py2){
    int x1=max(0,min(fieldMapWidth-1,min(px1,px2)-1));
    int y1=max(0,min(fieldMapHeight-1,min(py1,py2)-1));
    int x2=max(0,min(fieldMapWidth-1,max(px1,px2)));
    int y2=max(0,min(fieldMapHeight-1,max(py1,py2)));
    int area=(x2-x1)*(y2-y1);
    if(area==0)return 0;
    return (fieldMapIntegral[x2+y2*fieldMapWidth]-fieldMapIntegral[x1+y2*fieldMapWidth]-fieldMapIntegral[x2+y1*fieldMapWidth]+fieldMapIntegral[x1+y1*fieldMapWidth])/area;
}

void RobotAreaDetector::createIntegralImage(){
    fieldMapIntegral[0]=fieldMap[0];
	for(int x=1;x<fieldMapWidth;x++){
		fieldMapIntegral[x]=fieldMapIntegral[x-1]+fieldMap[x];
	}
	for(int y=1;y<fieldMapHeight;y++){
		int sum=0;
		for(int x=0;x<fieldMapWidth;x++){
			int addr=x+y*fieldMapWidth;
			sum+=fieldMap[addr];
			fieldMapIntegral[addr]=sum+fieldMapIntegral[addr-fieldMapWidth];
		}
	}
}


/*
 * See: https://software.intel.com/en-us/node/503876 YCbCr 422
 * Before:
 * Y0 Cb0  Y1 Cr0  Y2 Cb1  Y3 Cr1 Y4 Cb2 Y5 Cr2 Y6 Cb3 Y7 Cr3
 *  0   1   2   3   4   5   6   7  8   9 10  11 12  13 14  15
 *
 * After:
 * Y0 Y1 Y2 Y3 Y4 Y5  Y6  Y7 Cb0 Cb1 Cb2 Cb3 Cr0 Cr1 Cr2 Cr3
 *  0  2  4  6  8 10  12  14   1   5   9  13   3   7  11  15
 */
static const unsigned char cshuffle[]={0,2,4,6,8,10,12,14,1,5,9,13,3,7,11,15};

/*
 * Before:
 * Y0  Y1  Y2  Y3  Y4  Y5  Y6  Y7  Y8  Y9  Y10  Y11  Y12  Y13  Y14  Y15
 *  0   1   2   3   4   5   6   7   8   9   10   11   12   13   14   15
 *
 * After:
 * Y0  Y2  Y4  Y6  Y8  Y10  Y12  Y14
 *  0   2   4   6   8   10   12   14
 */
static const char yout_cshuffle[]={0,2,4,6,8,10,12,14,-1,-1,-1,-1,-1,-1,-1,-1};

/* Shuffle so the start matches to the indexes to lutCb (one is missing and everything is shifted forward) */
static const char uout_start_cshuffle[]={0,2,3,4,5,6,7,8,9,10,11,12,13,14,15,-1};

/* Shuffle so the end matches to the indexes to lutCb (duplication of last element) */
static const char uout_end_cshuffle[]={1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,15};

static const __m128i shuffle=_mm_loadu_si128((__m128i*)cshuffle);
static const __m128i yout_shuffle=_mm_loadu_si128((__m128i*)yout_cshuffle);
static const __m128i uout_start_shuffle=_mm_loadu_si128((__m128i*)uout_start_cshuffle);
static const __m128i uout_end_shuffle=_mm_loadu_si128((__m128i*)uout_end_cshuffle);

void RobotAreaDetector::createGreenMapSSEBatch(const uint8_t * const img, FieldColorDetector *field, int x, int y){

    const int BYTES_PER_ITERATION=16 /* 128 bit operations */ * 4 /* four at a time */;
    const int BYTES_PER_PIXEL = 2;
    const int BYTES_PER_LINE = BYTES_PER_PIXEL*width;

    /* Result:
     *  LSB Y00 Y01 Y02 Y03 Y04 Y05 Y06 Y07 Cb00 Cb01 Cb02 Cb03 Cr00 Cr01 Cr02 Cr03
     *  LSB Y08 Y09 Y10 Y11 Y12 Y13 Y14 Y15 Cb04 Cb05 Cb06 Cb07 Cr04 Cr05 Cr06 Cr07
     *  LSB Y16 Y17 Y18 Y19 Y20 Y21 Y22 Y23 Cb08 Cb09 Cb10 Cb11 Cr08 Cr09 Cr10 Cr11
     *  LSB Y24 Y25 Y26 Y27 Y28 Y29 Y30 Y32 Cb12 Cb13 Cb14 Cb15 Cr12 Cr13 Cr14 Cr15
     */
    __m128i *in=(__m128i*)(img+y*fieldMapScale*BYTES_PER_LINE+x*BYTES_PER_ITERATION);
    __m128i im0=_mm_shuffle_epi8(in[0],shuffle);
    __m128i im1=_mm_shuffle_epi8(in[1],shuffle);
    __m128i im2=_mm_shuffle_epi8(in[2],shuffle);
    __m128i im3=_mm_shuffle_epi8(in[3],shuffle);

    /* Result:
     *  LSB Y00 Y01 Y02 Y03 Y04 Y05 Y06 Y07 Y08 Y09 Y10 Y11 Y12 Y13 Y14 Y15
     *  LSB Y16 Y17 Y18 Y19 Y20 Y21 Y22 Y23 Y24 Y25 Y26 Y27 Y28 Y29 Y30 Y32
     */
    __m128i yout0=_mm_unpacklo_epi64(im0,im1);
    __m128i yout1=_mm_unpacklo_epi64(im2,im3);

    /* Result:
     *  LSB Y00 Y02 Y04 Y05 Y08 Y10 Y12 Y14 | 0 0 0 0 0 0 0 0
     *  LSB Y16 Y18 Y20 Y22 Y24 Y26 Y28 Y30 | 0 0 0 0 0 0 0 0
     */
    yout0=_mm_shuffle_epi8(yout0,yout_shuffle);
    yout1=_mm_shuffle_epi8(yout1,yout_shuffle);

    /* Result:
     *  LSB Y00 Y02 Y04 Y05 Y08 Y10 Y12 Y14 | Y16 Y18 Y20 Y22 Y24 Y26 Y28 Y30
     */
    yout[y*fieldMapWidthSSE+x]=_mm_unpacklo_epi64(yout0,yout1);


    /* Result:
     *  LSB Cb00 Cb01 Cb02 Cb03 Cr00 Cr01 Cr02 Cr03 Cb04 Cb05 Cb06 Cb07 Cr04 Cr05 Cr06 Cr07
     *  LSB Cb08 Cb09 Cb10 Cb11 Cr08 Cr09 Cr10 Cr11 Cb12 Cb13 Cb14 Cb15 Cr12 Cr13 Cr14 Cr15
     */
    __m128i ic0=_mm_unpackhi_epi64(im0,im1);
    __m128i ic1=_mm_unpackhi_epi64(im2,im3);

    /* Result: _MM_SHUFFLE is MSB to LSB
     *  0 Cb00 Cb01 Cb02 Cb03
     *  1 Cr00 Cr01 Cr02 Cr03
     *  2 Cb04 Cb05 Cb06 Cb07
     *  3 Cr04 Cr05 Cr06 Cr07
     *
     *  LSB Cb00 Cb01 Cb02 Cb03 Cb04 Cb05 Cb06 Cb07 Cr00 Cr01 Cr02 Cr03 Cr04 Cr05 Cr06 Cr07
     *
     *  3 Cb08 Cb09 Cb10 Cb11
     *  2 Cr08 Cr09 Cr10 Cr11
     *  1 Cb12 Cb13 Cb14 Cb15
     *  0 Cr12 Cr13 Cr14 Cr15
     *
     *  LSB Cb08 Cb09 Cb10 Cb11 Cb12 Cb13 Cb14 Cb15 Cr08 Cr09 Cr10 Cr11 Cr12 Cr13 Cr14 Cr15
     */
    __m128i is0=_mm_shuffle_epi32(ic0,_MM_SHUFFLE(3,1,2,0));
    __m128i is1=_mm_shuffle_epi32(ic1,_MM_SHUFFLE(3,1,2,0));

    /* Result: Cb00 Cb01 Cb02 Cb03 Cb04 Cb05 Cb06 Cb07 Cb08 Cb09 Cb10 Cb11 Cb12 Cb13 Cb14 Cb15 */
    uout[y*fieldMapWidthSSE+x]=_mm_unpacklo_epi64(is0,is1);

    /* Result: Cr00 Cr01 Cr02 Cr03 Cr04 Cr05 Cr06 Cr07 Cr08 Cr09 Cr10 Cr11 Cr12 Cr13 Cr14 Cr15 */
    vout[y*fieldMapWidthSSE+x]=_mm_unpackhi_epi64(is0,is1);
}

void RobotAreaDetector::createGreenMapSSECompare(Boundries& b, int x, int y){

    /* This was the contents of is green */
//    if(cyReal<minCy)return false; //68
//    if(cyReal>maxCy)return false; //30018
//    if(cbReal>maxCb)return false; //7576
//    if(cbReal<minCb)return false; //2171
//    if(crReal<minCr)return false; //353
//    if(crReal>maxCr)return false; //44705

    /*
     * We invert the logic because it's easier to implement. We added one before.
     * the results are aggregated via AND. If in the end it's 0xff it's green.
     */
    __m128i cur = yout[y*fieldMapWidthSSE+x];
    __m128i resMinY = _mm_cmpgt_epi8(cur, b.minCy);
    __m128i resMaxY = _mm_cmplt_epi8(cur, b.maxCy);
    __m128i resCy = _mm_and_si128(resMinY, resMaxY);

    cur = uout[y*fieldMapWidthSSE+x];
    __m128i resMinU = _mm_cmpgt_epi8(cur, b.minCb);
    __m128i resMaxU = _mm_cmplt_epi8(cur, b.maxCb);
    __m128i resCb = _mm_and_si128(resMinU, resMaxU);

    cur = vout[y*fieldMapWidthSSE+x];
    __m128i resMinV = _mm_cmpgt_epi8(cur, b.minCr);
    __m128i resMaxV = _mm_cmplt_epi8(cur, b.maxCr);
    __m128i resCr = _mm_and_si128(resMinV, resMaxV);

    __m128i res = _mm_and_si128(resCy, resCb);
    ((__m128i*)fieldMap)[y*fieldMapWidthSSE+x] = _mm_and_si128(res, resCr);
}


void RobotAreaDetector::createGreenMapSSE(const int * const fieldborder, const uint8_t * const img, FieldColorDetector *field){
    if(width != 640)
        return;

    const int BYTES_PER_ITERATION=16 /* 128 bit operations */ * 4 /* four at a time */;
    const int BYTES_PER_PIXEL = 2;
    const int BYTES_PER_LINE = BYTES_PER_PIXEL*width;

    /*
     * +1 to match isGreen semantics because we invert the logic.
     * We can't use class variables because they are aligned to 8 bytes. This could be a compiler bug.
     */
    Boundries b;
    b.minCy = _mm_set1_epi8((char)field->minCy+1);
    b.minCb = _mm_set1_epi8((char)field->minCb+1);
    b.minCr = _mm_set1_epi8((char)field->minCb+1);
    b.maxCy = _mm_set1_epi8((char)field->maxCy+1);
    b.maxCb = _mm_set1_epi8((char)field->maxCb+1);
    b.maxCr = _mm_set1_epi8((char)field->maxCr+1);

    /* We process the field from the end to the top. We have to carry over one Cb value to the elements before */
    for(int y = fieldMapHeight-1; y >= 0; y--) {

        /* In that line: process last two elements first. The last element need some extra shuffeling */
        createGreenMapSSEBatch(img, field, BYTES_PER_LINE/BYTES_PER_ITERATION-1, y);
        createGreenMapSSEBatch(img, field, BYTES_PER_LINE/BYTES_PER_ITERATION-2, y);

        /* Shift the first element to the most right to use it as carry over and add it later.
         * Before: LSB: Cb0 Cb1 Cb2 Cb3 Cb4 Cb5 Cb6 Cb6 Cb7 Cb8 Cb9 Cb10 Cb11 Cb12 Cb13 Cb14 Cb15
         * After:  LSB:   0   0   0   0   0   0   0   0   0   0   0    0    0    0    0    0 Cb0
          */
        __m128i tmp       = _mm_slli_si128(uout[y*fieldMapWidthSSE+BYTES_PER_LINE/BYTES_PER_ITERATION-1], 15);
        __m128i carryOver = _mm_slli_si128(uout[y*fieldMapWidthSSE+BYTES_PER_LINE/BYTES_PER_ITERATION-2], 15);

        /* Make some space for the carry over by shifting Cb0 out.
         * Before: LSB: Cb0 Cb1 Cb2 Cb3 Cb4 Cb5 Cb6 Cb6 Cb7 Cb8  Cb9 Cb10 Cb11 Cb12 Cb13 Cb14 Cb15
         * After:  LSB: Cb1 Cb2 Cb3 Cb4 Cb5 Cb6 Cb6 Cb7 Cb8 Cb9 Cb10 Cb11 Cb12 Cb13 Cb14 Cb15 0
         */
        __m128i acc = _mm_srli_si128(uout[y*fieldMapWidthSSE+BYTES_PER_LINE/BYTES_PER_ITERATION-2], 1);

        /* Add the carry over.
         * Before:  ACC: LSB: Cb1 Cb2 Cb3 Cb4 Cb5 Cb6 Cb6 Cb7 Cb8 Cb9 Cb10 Cb11 Cb12 Cb13 Cb14 Cb15 0
         *          TMP: LSB:   0   0   0   0   0   0   0   0   0   0    0    0    0    0    0    0 Cb0
         * After:   LSB: Cb1 Cb2 Cb3 Cb4 Cb5 Cb6 Cb6 Cb7 Cb8 Cb9 Cb10 Cb11 Cb12 Cb13 Cb14 Cb15 Cb0
         */
        uout[y*fieldMapWidthSSE+BYTES_PER_LINE/BYTES_PER_ITERATION-2] = _mm_or_si128(acc, tmp);

        /* Shuffle the last element according to lutCb output (duplicated last element */
        uout[y*fieldMapWidthSSE+BYTES_PER_LINE/BYTES_PER_ITERATION-1] = _mm_shuffle_epi8(uout[y*fieldMapWidthSSE+BYTES_PER_LINE/BYTES_PER_ITERATION-1], uout_end_shuffle);

        createGreenMapSSECompare(b, BYTES_PER_LINE/BYTES_PER_ITERATION-2, y);
        createGreenMapSSECompare(b, BYTES_PER_LINE/BYTES_PER_ITERATION-1, y);

        for(int x = BYTES_PER_LINE/BYTES_PER_ITERATION-3; x >= 1;x--) {
            createGreenMapSSEBatch(img, field, x, y);

            tmp = carryOver;

            /* Will be the next carry over. Everything 0 execpt Cb15 = Cb0 */
            carryOver = _mm_slli_si128(uout[y*fieldMapWidthSSE+x], 15);

            /* Make space by shifting Cb0 out */
            acc = _mm_srli_si128(uout[y*fieldMapWidthSSE+x], 1);

            /* Add the old carry over */
            uout[y*fieldMapWidthSSE+x] = _mm_or_si128(acc, tmp);
            createGreenMapSSECompare(b, x, y);
        }

        /* This is the first element which has a special ordering. */
        const int x = 0;
        createGreenMapSSEBatch(img, field, x, y);

        tmp = carryOver;

        /* See uout_start_cshuffle for ordering */
        acc = _mm_shuffle_epi8(uout[y*fieldMapWidthSSE+x], uout_start_shuffle);

        /* Add carry over from tmp. There is no new carry over. */
        uout[y*fieldMapWidthSSE+x] = _mm_or_si128(acc, tmp);
        createGreenMapSSECompare(b, x, y);
    }
}

void RobotAreaDetector::createGreenMap(const int * const fieldborder, const uint8_t * const img, FieldColorDetector *field){
    if(width == 640 && fieldMapScale == 2) {
        createGreenMapSSE(fieldborder, img, field);
        return;
    }

//    for(int py=0;py<fieldMapHeight;py++){
//        for(int px=0;px<fieldMapWidth;px++){
//            fieldMap[px+py*fieldMapWidth]=field->isGreen(img,px*fieldMapScale,py*fieldMapScale)?255:0;
//        }
//    }

    /* Naive implementation */
    int y=0;
    for(int py=0;py<fieldMapHeight;py++){
        int x=0;
        for(int px=0;px<fieldMapWidth;px++){
            if(y<fieldborder[x]){
                fieldMap[px+py*fieldMapWidth]=0;
            }else{
                fieldMap[px+py*fieldMapWidth]=field->isGreen(img,x,y)?255:0;
            }
            x+=fieldMapScale;
        }
        y+=fieldMapScale;
    }

}


vector<RobotRect> RobotAreaDetector::searchRobotHypotheses(Scanline *scanVertical, const int * const fieldborder){
    vector<RobotRect> hypotheses;

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
            int wEst=(int)(14.885+0.4413*borderHeight);//estimated Robot width (fitting by zunzun.com)
            if(greenRatio<minGreenRatio&&borderHeight>minBorderHeight){
                {
                    RobotRect rect;
                    rect.xLeft=px-wEst/2;
                    rect.yTop=fieldborder[px];
                    rect.xRight=px+wEst/2;
                    rect.yBottom=py;
                    if(rect.xLeft>=0&&rect.xRight<width&&rect.yTop>=0&&rect.yBottom<height)
                        hypotheses.push_back(rect);
                }
                {
                    RobotRect rect;
                    rect.xLeft=px-wEst*3/4;
                    rect.yTop=fieldborder[px];
                    rect.xRight=px+wEst*1/4;
                    rect.yBottom=py;
                    if(rect.xLeft>=0&&rect.xRight<width&&rect.yTop>=0&&rect.yBottom<height)
                        hypotheses.push_back(rect);
                }
                {
                    RobotRect rect;
                    rect.xLeft=px-wEst*1/4;
                    rect.yTop=fieldborder[px];
                    rect.xRight=px+wEst*3/4;
                    rect.yBottom=py;
                    if(rect.xLeft>=0&&rect.xRight<width&&rect.yTop>=0&&rect.yBottom<height)
                        hypotheses.push_back(rect);
                }
            }
            if(greenStartCnt<8){
                break;
            }
        }
    }
    return hypotheses;
}

vector<RobotRect>* RobotAreaDetector::getRobotAreas(){
    return robotAreas;
}

}
