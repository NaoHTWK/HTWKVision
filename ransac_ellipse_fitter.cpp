#include <ransac_ellipse_fitter.h>

#include <cstring>

#include <ellifit.h>
#include <ext_math.h>

using namespace std;

namespace htwk {

const float RansacEllipseFitter::minRating=0.35f;
const int RansacEllipseFitter::angleCnt=36;

RansacEllipseFitter::RansacEllipseFitter() : ellipseFound(false),
        camPitch(0), camRoll(0) {
}

RansacEllipseFitter::~RansacEllipseFitter(){
}

void RansacEllipseFitter::proceed(vector<LineSegment*> lineEdgeSegments, vector<LineEdge*> &lineEdges){
    ellipseFound=false;
    vector<LineSegment*> curveSegments;
    vector<LineSegment*> curveSegmentsFiltered;
    for(const LineSegment *ls : lineEdgeSegments){
        if(ls->parentLine!=0&&!ls->parentLine->straight){
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
    float ellipse[6];
    float rating=ransacFit(curveSegmentsFiltered,curveSegments,ellipse,15,20);
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

float RansacEllipseFitter::getRating(vector<LineSegment*> carryover, Ellipse e){
    int histo[angleCnt];
    memset(histo,0,sizeof(int)*angleCnt);
    for(const LineSegment *le : carryover){
        point_2d point=newPoint2D(le->x, le->y);
        transformPo(point, e.trans, e.translation);
        float px=point.x/e.ta;
        float py=point.y/e.tb;
        float dist=px*px+py*py;
        if(dist>1.1f*1.1f||dist<0.9f*0.9f){
            continue;
        }
        float angle=atan2(py,px)+M_PI;
        int index=((int)(angle/(M_PI*2)*angleCnt))%angleCnt;
        histo[index]++;
    }
    float rating=0;
    for(int angle=0;angle<angleCnt/2;angle++){
        if(histo[angle]>0&&histo[angle+angleCnt/2]>0){
            rating+=100+histo[angle];
        }
    }
    rating/=100.f*angleCnt/2;
    return rating;
}


float RansacEllipseFitter::ransacFit(vector<LineSegment*> &carryover, vector<LineSegment*> &lineEdgeSegments, float ellipse[6], int iter, unsigned int minMatches){
    if(carryover.size()<minMatches){
        for(int j=0;j<6;j++){
            ellipse[j]=0;
        }
        return 0;
    }

    elli_init();
    float tmpEllipse[6];
    bool foundBest=false;
    Ellipse bestEll;
    float radius=3;
    float max=minRating;
    //RANSAC iterations
    for(int i=0;i<iter;i++){

        //get 6 random LineSegments o construct an ellipse
        vector<point_2d> tmp;
        for(int j=0;j<6;j++){
            LineSegment *ls=carryover[(int)(dist(rng)*carryover.size())];
            point_2d p=newPoint2D(ls->x,ls->y);
            tmp.push_back(p);
        }

        if(fit(tmp,tmpEllipse)){
            Ellipse e(tmpEllipse);
            if(transformEl(e)==0){
                float rating=getRating(lineEdgeSegments,e);
                if(rating>max){
                    max=rating;
                    bestEll=e;
                    foundBest=true;
                }
            }
        }
    }
    if(!foundBest){
        for(int j=0;j<6;j++){
            ellipse[j]=0;
        }
        return 0;
    }

    //get consensus-set
    vector<point_2d> bestSet;
    for(const LineSegment *p : lineEdgeSegments){
        float dist=getEllDist(p->x,p->y,bestEll);
        point_2d p2=newPoint2D(p->x,p->y);
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
            return 0;
        }

        //get inlier count for the better ellipse

        float finalRating=getRating(lineEdgeSegments,test);
        if(finalRating<minRating){
            for(int j=0;j<6;j++){
                ellipse[j]=0;
            }
            return 0;
        }
        for(int j=0;j<6;j++){
            ellipse[j]=tmpEllipse[j];
        }
        return finalRating;
    }else{
        for(int j=0;j<6;j++){
            ellipse[j]=0;
        }
        return 0;
    }

}

void RansacEllipseFitter::eigenvectors(float a, float b, float eva[2],float eve[][2]){
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
    point_2d point=newPoint2D(px,py);
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

void RansacEllipseFitter::transformPo(point_2d &p, float trans[][2], float translation[2] ){
    float temp1=trans[0][0]* p.x + trans[1][0]*p.y;
    float temp2=trans[0][1]* p.x + trans[1][1]*p.y;
    p.x=temp1+translation[0];
    p.y=temp2+translation[1];
}

void RansacEllipseFitter::transformPoInv(point_2d &p, float trans[][2], float translation[2] ){
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
