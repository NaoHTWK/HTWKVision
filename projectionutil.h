#ifndef PROJECTION_UTIL_H
#define PROJECTION_UTIL_H

#include <line.h>
#include <point_2d.h>

#include <cmath>

namespace htwk{
class ProjectionUtil {
private:
    ProjectionUtil() = delete;

    static constexpr int CAM_WIDTH=640;
    static constexpr int CAM_HEIGHT=480;
    static constexpr float CAM_FOV=1.03;
    static constexpr float CAM_TAN_FOV2 = 0.565941592f;  // tan(fov/2)
    static constexpr float BOT_HEIGHT=0.4694;
    static constexpr float SIZE_OFFSET=-3;

public:
    inline static point_2d roll(const point_2d& p, float sRoll, float cRoll) {
        return point_2d((p.x-CAM_WIDTH/2)*cRoll+(p.y-CAM_HEIGHT/2)*sRoll+CAM_WIDTH/2,
                        -(p.x-CAM_WIDTH/2)*sRoll+(p.y-CAM_HEIGHT/2)*cRoll+CAM_HEIGHT/2);
    }

    inline static point_2d pitch(const point_2d& p, float sPitch,float cPitch) {
        const float f=CAM_WIDTH/2./std::tan(CAM_FOV/2);
        std::array<float, 3> pr{{p.x-CAM_WIDTH*0.5f,f,-(p.y-CAM_HEIGHT*0.5f)}};
        std::array<float, 3> pRot={{pr[0],cPitch*pr[1]+sPitch*pr[2],-sPitch*pr[1]+cPitch*pr[2]}};
        if(pRot[2]>=0){
            return point_2d(0,0);
        }
        float fac=BOT_HEIGHT/pRot[2];
        return point_2d(-pRot[1]*fac,pRot[0]*fac);
    }

    inline static point_2d projectPoint(const point_2d& p, float pitchRad, float rollRad) {
        return projectPoint(p.x, p.y, pitchRad, rollRad);
    }

    inline static point_2d projectPoint(float x, float y, float pitchRad, float rollRad) {
        float sRoll=std::sin(rollRad);
        float cRoll=std::cos(rollRad);
        float sPitch=std::sin(pitchRad);
        float cPitch=std::cos(pitchRad);
        point_2d p(x,y);
        p=roll(p,sRoll,cRoll);
        return pitch(p,sPitch, cPitch);
    }

    inline static float getObjectDist(float x, float y, float pitch, float roll) {
        point_2d rel=projectPoint(x,y, pitch, roll);
        float dist=std::sqrt(rel.x*rel.x+rel.y*rel.y);
        if (dist==0)
            return 0;
        return std::sqrt(dist*dist+BOT_HEIGHT*BOT_HEIGHT);
    }

    static float getObjectGroundDist(float x, float y, float pitch, float roll) {
        point_2d rel=projectPoint(x,y, pitch, roll);
        return rel.norm();
    }

    inline static float getObjectRadius(const int x, const int y, float pitch, float roll, float OBJ_SIZE) {
        const float distCam=getObjectDist(x,y,pitch,roll);
        if(distCam==0)
            return 10;
        return std::max(7.f,OBJ_SIZE/distCam+SIZE_OFFSET);//ballradius in px, wenn 1 Meter entfernt
    }

    inline static Line getHorizon(float pitch, float roll) {
        return {point_2d(0.f,
                 (-std::sin(pitch) * (CAM_WIDTH / 2 / CAM_TAN_FOV2) / std::cos(pitch) - CAM_WIDTH / 2 * std::sin(roll)) / std::cos(roll) + CAM_HEIGHT / 2),
                point_2d((float)CAM_WIDTH,
                 (-std::sin(pitch) * (CAM_WIDTH / 2 / CAM_TAN_FOV2) / std::cos(pitch) + CAM_WIDTH / 2 * std::sin(roll)) / std::cos(roll) + CAM_HEIGHT / 2)};
    }

    inline static bool belowHorizon(const point_2d& p, float pitch, float roll) {
        Line horizon = getHorizon(pitch, roll);
        return belowHorizon(p, horizon);
    }

    inline static bool belowHorizon(const point_2d& p, const Line& horizon) {
        return (p.x-horizon.px1)*(horizon.py1-horizon.py2) + (p.y-horizon.py1)*(horizon.px2-horizon.px1) > 0;
    }
};
}//namespace htwk

#endif // PROJECTIONUTIL_H
