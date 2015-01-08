#ifndef __GOALPOST_H__
#define __GOALPOST_H__


namespace htwk {

struct GoalPost{
    float x,y;
    float lineR[2];
    float lineL[2];
    int height[2];
    int width;
};

inline GoalPost newGoalPost(float x,float y){
    GoalPost g;
    g.x=x;
    g.y=y;
    return g;
}

}  // namespace htwk

#endif // __GOALPOST_H__
