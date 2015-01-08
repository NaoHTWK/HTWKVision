#ifndef __LINECROSS_H__
#define __LINECROSS_H__

namespace htwk {

struct LineCross{
	float px,py,vx,vy;
};

inline LineCross newLineCross(float px, float py, float vx, float vy){
    LineCross c;
    c.px=px;
    c.py=py;
    c.vx=vx;
    c.vy=vy;
    return c;
}

}  // namespace htwk

#endif // __LINECROSS_H__
