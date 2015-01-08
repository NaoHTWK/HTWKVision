#ifndef __ELLIPSE_H__
#define __ELLIPSE_H__

namespace htwk {

class Ellipse{
public:
    float a,b,c,d,e,f;
    float a1,b1,c1,d1,e1,f1;
    float ta, tb;
    float brennpunkt;
    float trans[2][2];
    float translation[2];
    bool found;

    Ellipse() : a(0),b(0),c(0),d(0),e(0),f(0),
        a1(0),b1(0),c1(0),d1(0),e1(0),f1(0),
        ta(0),tb(0), brennpunkt(0), found(false) {
        trans[0][0] = 0;
        trans[0][1] = 0;
        trans[1][0] = 0;
        trans[1][1] = 0;

        translation[0] = 0;
        translation[1] = 0;
    }

    Ellipse(float ellipse[6]) : a1(0),b1(0),c1(0),d1(0),e1(0),f1(0),
            ta(0),tb(0), brennpunkt(0), found(false) {
        a=ellipse[0];
        b=ellipse[1];
        c=ellipse[2];
        d=ellipse[3];
        e=ellipse[4];
        f=ellipse[5];

        trans[0][0] = 0;
        trans[0][1] = 0;
        trans[1][0] = 0;
        trans[1][1] = 0;

        translation[0] = 0;
        translation[1] = 0;
    }

    ~Ellipse(){}
};

}  // namespace htwk

#endif  // __ELLIPSE_H__
