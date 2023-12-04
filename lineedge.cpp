#include "lineedge.h"

#include <algorithm>

#include "linesegment.h"

namespace htwk {

LineEdge::LineEdge(std::vector<LineSegment*>  seg) : segments(std::move(seg)) {
    px1=py1=px2=py2=0;
    nx=ny=d=x=y=0;
    id=0;
    matchCnt=0;
    straight=false;
    valid=false;
	update();
}

LineEdge::LineEdge(int id){
	this->id=id;
    px1=py1=px2=py2=0;
    nx=ny=d=x=y=0;
    matchCnt=0;
    straight=false;
    valid=false;
}

LineEdge::LineEdge(){
    px1=py1=px2=py2=0;
    nx=ny=d=x=y=0;
    id=0;
    matchCnt=0;
    straight=false;
    valid=false;
}

void LineEdge::update(){
	x=0;
	y=0;
	if(segments.size()<3){
	    valid=false;
	    return;
	}
	for (const LineSegment *ls : segments){
		x+=ls->x;
		y+=ls->y;
	}
	x/=segments.size();
	y/=segments.size();

	float vx=0;
	float vy=0;
	float rx=0;
	float ry=0;
	for (const LineSegment *ls : segments){
		float dx=ls->x-x;
		float dy=ls->y-y;
		rx+=ls->vx;
		ry+=ls->vy;
		if(dx<0||(dx==0&&dy<0)){
			dx=-dx;
			dy=-dy;
		}
		vx+=dx;
		vy+=dy;
	}
	if(vx == 0 && vy == 0) {
	    valid=false;
	    return;
	}
    float lenInv=1.f/std::sqrt(vx*vx+vy*vy);//invSqrt(vx*vx+vy*vy);
	vx*=lenInv;
	vy*=lenInv;
	nx=vy;
	ny=-vx;
	if(nx*rx+ny*ry<0){
		nx=-nx;
		ny=-ny;
	}
	d=x*nx+y*ny;

	float minD=0;
	float maxD=0;
	float d2=vx*x+vy*y;
	px1=0;
	py1=0;
	px2=0;
	py2=0;
	std::vector<LineSegment*> segmentsFiltered(segments);
	segments.clear();
	for (LineSegment *lsCheck : segmentsFiltered){
		float dist=lsCheck->x*nx+lsCheck->y*ny-d;
        if(fabsf(dist)<4){
			segments.push_back(lsCheck);
		}
	}

	for (const LineSegment *lsCheck : segments){
		float dist2=lsCheck->x*vx+lsCheck->y*vy-d2;
		if(dist2<=minD){
			minD=dist2;
			px1=lsCheck->x;
			py1=lsCheck->y;
		}
		if(dist2>=maxD){
			maxD=dist2;
			px2=lsCheck->x;
			py2=lsCheck->y;
		}
	}

	float dist=px2*nx+py2*ny-d;
	px2-=nx*dist;
	py2-=ny*dist;
	dist=px1*nx+py1*ny-d;
	px1-=nx*dist;
	py1-=ny*dist;
	valid=true;
}

float LineEdge::estimateLineWidth() const {
	if(segments.empty())return 0;
	float lineWidth=0;
	float minDist=9999;
	for (const LineSegment *it : segments){
		float dx=it->x-it->link->x;
		float dy=it->y-it->link->y;
		float distSq=dx*dx+dy*dy;
		lineWidth+=distSq;
		minDist=std::min(minDist,distSq);
	}
    return 0.5f*(std::sqrt(minDist)+std::sqrt(lineWidth/segments.size()));
}

}  // namespace htwk
