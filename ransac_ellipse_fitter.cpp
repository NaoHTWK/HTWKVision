#include "ransac_ellipse_fitter.h"

#include <algorithm>
#include <cstring>

#include <easy/profiler.h>

#include "ellifit.h"
#include <robotoption.h>
#include <visualizer.h>

using namespace std;
using namespace NaoControl;

namespace htwk {

float RansacEllipseFitter::minRating=0.35f;
int RansacEllipseFitter::angleCnt=20;
int RansacEllipseFitter::minCurvedSegments = 20;
int RansacEllipseFitter::minIterationTries = 100;

RansacEllipseFitter::RansacEllipseFitter(const int8_t *lutCb, const int8_t *lutCr, HtwkVisionConfig &config)
    : BaseDetector(lutCb, lutCr, config) {

    if(config.activate_visualization) {
        std::string name = std::string("HTWK/Vision/RansacEllipseFitter/") + (config.isUpperCam ? "Upper" : "Lower");

        auto* options = new OptionSet(name.c_str());
        options->addOption(new NaoControl::IntOption("angleCnt", &angleCnt, 0, 100, 1));
        options->addOption(new NaoControl::IntOption("minIterationTries", &minIterationTries, 1, 100, 1));
        options->addOption(new NaoControl::IntOption("minCurvedSegments", &minCurvedSegments, 6, 100, 1));
        options->addOption(new NaoControl::FloatOption("minRating", &minRating, 0.f, 1.f, .01f));
        options->addOption(new NaoControl::FloatOption("minDistanceFromEllipse", &minDistanceFromEllipse, 0.f, 1.f, .01f));
        NaoControl::RobotOption::instance().addOptionSet(options);
    }
}

void RansacEllipseFitter::proceed(const vector<LineSegment*> &lineEdgeSegments, uint8_t* image){
    Timer t("RansacEllipseFitter", 50);
    EASY_FUNCTION(profiler::colors::Orange100);

    ellipseFound=false;
    vector<LineSegment*> curveSegments;
    vector<LineSegment*> curveSegmentsFiltered;
    for(const LineSegment *ls : lineEdgeSegments){
        if(ls->parentLine!=nullptr&&!ls->parentLine->straight){
            //da nur eine Ellipse in der Mitte der Linie berechnet werden soll,
            //werden hier jeweils zwei zusammengehörige Linienkanten gemittelt
            LineSegment *lsMid=new LineSegment(	(ls->x+ls->link->x)/2,
                    (ls->y+ls->link->y)/2,
                    (ls->vx-ls->link->vx)/2,
                    (ls->vy-ls->link->vy)/2);
            curveSegments.push_back(lsMid);
            if(ls->parentLine->segments.size()>=3){
                curveSegmentsFiltered.push_back(lsMid);
            }
        }
    }

    abortState = 0;
    float ellipse[6];
    float iterMaxRating = 0;
    float rating=ransacFit(curveSegmentsFiltered,curveSegments,ellipse,minIterationTries,minCurvedSegments,image,iterMaxRating);

//    for (const LineSegment *ls : curveSegmentsFiltered) {
//        if (ls->x < 0 || ls->x >= 640 || ls->y < 0 || ls->y >= 480)
//            continue;
//        for (float d = 0; d <= 5; d += 0.1) {
//            int px = (int)(ls->x + ls->vx * d);
//            int py = (int)(ls->y + ls->vy * d);
//            if (px < 0 || py < 0 || px >= 640 || py >= 480)
//                continue;
//            setY(image, px, py, 255);
//        }
//        setY(image, ls->x, ls->y, 0);
//    }


    if(config.activate_visualization) {
        VisTransPtr ptr = Visualizer::instance().startTransaction({}, "RansacEllipseFitter", RELATIVE_BODY, REPLACE);
        ptr->addParameter(Parameter::createInt("Abort State", abortState));
        ptr->addParameter(Parameter::createInt("Curved Segment Count", (int)curveSegments.size()));
        ptr->addParameter(Parameter::createInt("Detected (w/LS)", sucessfullDetectionWithAdditionLineSegments));
        ptr->addParameter(Parameter::createInt("Detected (wo/LS)", sucessfullDetectionWithoutAdditionalLineSigments));
        ptr->addParameter(Parameter::createFloat("Max Rating", iterMaxRating));
        Visualizer::instance().commit(ptr);
    }

    resultEllipse=Ellipse(ellipse);
    if(rating>minRating&&transformEl(resultEllipse)==0){
        ellipseFound=true;
        resultEllipse.found=true;
    }else{
        resultEllipse.found=false;
    }

    for(LineSegment *ls : curveSegments){
        delete ls;
    }
}

float RansacEllipseFitter::getRating(const vector<LineSegment*> &carryover, const Ellipse& e){
    int histo[angleCnt];
    memset(histo,0,sizeof(int)*angleCnt);
    for(const LineSegment *le : carryover){
        point_2d point=point_2d(le->x, le->y);
        transformPo(point, e.trans, e.translation);

        if(e.ta == 0 || e.tb == 0)
            continue;

        float px=point.x/e.ta;
        float py=point.y/e.tb;
        float dist=px*px+py*py;

        float distanceFromEllipse = fabs(1-dist);

        if(distanceFromEllipse > minDistanceFromEllipse){
            continue;
        }
        float angle=atan2(py,px)+M_PI;
        int index=((int)(angle/(M_PI*2)*angleCnt))%angleCnt;
        histo[index]++;
    }
    float rating=0;
    for(int angle=0;angle<angleCnt/2;angle++){
        if(histo[angle]>0&&histo[angle+angleCnt/2]>0){
            rating+=100+histo[angle]+histo[angle+angleCnt/2];
        }
    }
    rating/=100.f*angleCnt/2;
    return rating;
}


float RansacEllipseFitter::ransacFit(const vector<LineSegment*> &carryover,
                                     const vector<LineSegment*> &lineEdgeSegments, float ellipse[6],
                                     int iter, unsigned int minMatches, uint8_t* image, float& iterMaxRating){
    if(carryover.size()<minMatches){
        for(int j=0;j<6;j++){
            ellipse[j]=0;
        }
        abortState = 1;
        return 0;
    }

    elli_init();
    float tmpEllipse[6];
    float bestEllTmp[6];
    bool foundBest=false;
    Ellipse bestEll;
    float max=minRating;
    //RANSAC iterations
    for(int i=0;i<iter;i++){

        //get 6 random LineSegments o construct an ellipse
        std::vector<LineSegment*> out;
        std::sample(carryover.begin(), carryover.end(), std::back_inserter(out), 6, rng);

        vector<point_2d> tmp;
        for(int j=0;j<6;j++){
            LineSegment *ls=out[j];
            point_2d p=point_2d(ls->x,ls->y);
            tmp.push_back(p);
        }

        if(fit(tmp,tmpEllipse)){
            Ellipse e(tmpEllipse);
            if(transformEl(e)==0){
                float rating=getRating(lineEdgeSegments,e);
                if(rating>iterMaxRating){
                    iterMaxRating=rating;
                }
                if(rating>max){
                    max=rating;
                    bestEll=e;
                    foundBest=true;
                    for(int j=0;j<6;j++){
                        bestEllTmp[j]=tmpEllipse[j];
                    }
                }
            }
        }
    }
    if(!foundBest){
        for(int j=0;j<6;j++){
            ellipse[j]=0;
        }
        abortState = 2;
        return 0;
    }

    //get consensus-set
    float radius=3;
    vector<point_2d> bestSet;
    for(const LineSegment *p : lineEdgeSegments){
        float dist=getEllDist(p->x,p->y,bestEll);
        point_2d p2=point_2d(p->x,p->y);
        if(fabsf(dist)<radius){
            bestSet.push_back(p2);
        }
    }
    //fit better ellipse from consensus-set
    if(fit(bestSet,tmpEllipse)){
        Ellipse test(tmpEllipse);
        if(transformEl(test)!=0){
            for(int j=0;j<6;j++){
                ellipse[j]=0;
            }
            abortState = 3;
            return 0;
        }

        //get inlier count for the better ellipse

        float finalRating=getRating(lineEdgeSegments,test);
        if(finalRating<minRating){
            for(int j=0;j<6;j++){
                ellipse[j]=0;
            }
            abortState = 4;
            return 0;
        }
        for(int j=0;j<6;j++){
            ellipse[j]=tmpEllipse[j];
        }

        sucessfullDetectionWithAdditionLineSegments++;
        return finalRating;
    }else{
        for(int j=0;j<6;j++){
            ellipse[j]=bestEllTmp[j];
        }

        abortState = 5;
        sucessfullDetectionWithoutAdditionalLineSigments++;
        return max;
    }

    abortState = 99;
    return max;
}

void RansacEllipseFitter::eigenvectors(float a, float b, const float eva[2],float eve[][2]){
    float l;

    //ev1
    eve[0][0]=b/(2*(eva[0]-a));
    eve[1][0]=1;
	l = sqrtf(eve[0][0]*eve[0][0]+1);
    eve[0][0]/=l;
    eve[1][0]/=l;
    //ev2
    eve[0][1]=b/(2*(eva[1]-a));
    eve[1][1]=1;
	l = sqrtf(eve[0][1]*eve[0][1]+1);
    eve[0][1]/=l;
    eve[1][1]/=l;
}

void RansacEllipseFitter::eigenvalues(float a, float b, float c,float erg[2]){

	float w = sqrtf(a*a + b*b + c*c - 2*a*c);
    float p = a+c;
    erg[0]=(p + w) / 2;
    erg[1]=(p - w) / 2;
}

int RansacEllipseFitter::det(float a[][2]){
    return (int)roundf(a[0][0]*a[1][1]-a[1][0]*a[0][1]);
}

int RansacEllipseFitter::transformEl(Ellipse &el){
    float eva[2];
    float temp1, temp2;

    eigenvalues(el.a,el.b,el.c,eva);
    if(eva[0]==el.a||eva[1]==el.a)
        return -1;
    eigenvectors(el.a,el.b,eva,el.trans);
    if(det(el.trans)==-1){
        el.trans[0][1]*=-1;
        el.trans[1][1]*=-1;
    }
    el.a1=eva[0];
    el.b1=0;
    el.c1=eva[1];
    temp1=el.trans[0][0]* el.d + el.trans[1][0]*el.e;
    temp2=el.trans[0][1]* el.d + el.trans[1][1]*el.e;
    el.d1=temp1;
    el.e1=temp2;
    el.translation[0]= el.d1 / (2 * el.a1);
    el.translation[1]= el.e1 / (2 * el.c1);;
    el.f1=el.f-(  ((el.d1*el.d1)/(4*(el.a1)))  + (el.e1*el.e1)/(4*(el.c1))  );
    if(-el.f1/el.a1<0||-el.f1/el.c1<0)
        return -1;
    el.ta=sqrtf(-el.f1/el.a1);
    el.tb=sqrtf(-el.f1/el.c1);

    if(numeric_limits<float>::infinity() == el.ta || el.tb == numeric_limits<float>::infinity())
        return -1;

    el.brennpunkt=sqrtf(fabsf(- el.ta*el.ta + el.tb*el.tb));
    return 0;
}

//gehört mal überarbeitet...
float RansacEllipseFitter::getEllDist(float px, float py, Ellipse trEl){
    point_2d point=point_2d(px,py);
    transformPo(point, trEl.trans, trEl.translation);
    px=point.x;
    py=point.y;
    float abstand=-9999;
    if(trEl.tb<trEl.ta){
        float e = powf(py,2);
        float aspect=1;
        abstand = aspect*(sqrt(powf(trEl.brennpunkt-px,2)+e)   +   sqrt(powf(-trEl.brennpunkt-px,2)+e)        -2*trEl.ta);
    }else{
        float e = powf(px,2);
        float aspect=1;
        abstand = aspect*(sqrt(powf(trEl.brennpunkt-py,2)+e)   +   sqrt(powf(-trEl.brennpunkt-py,2)+e)        -2*trEl.tb);
    }
    return abstand;
}

void RansacEllipseFitter::transformPo(point_2d &p, const float trans[][2], const float translation[2] ){
    float temp1=trans[0][0]* p.x + trans[1][0]*p.y;
    float temp2=trans[0][1]* p.x + trans[1][1]*p.y;
    p.x=temp1+translation[0];
    p.y=temp2+translation[1];
}

void RansacEllipseFitter::transformPoInv(point_2d &p, const float trans[][2], const float translation[2] ){
    p.x-=translation[0];
    p.y-=translation[1];
    float temp1=trans[1][1]* p.x - trans[1][0]*p.y;
    float temp2=-trans[0][1]* p.x + trans[0][0]*p.y;
    p.x=temp1;
    p.y=temp2;
}

Ellipse &RansacEllipseFitter::getEllipse(){
    return resultEllipse;
}

}  // namespace htwk
