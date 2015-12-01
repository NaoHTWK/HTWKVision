#ifndef COORD_H
#define COORD_H

namespace htwk {

struct Coord{
	float x;
	float y;
	float z;
	float yaw;
	float pitch;
	float roll;
};

inline Coord newCoord(float x,float y){
    Coord c;
    c.x=x;
    c.y=y;
    return c;
}

inline Coord newCoord(float x,float y,float z){
    Coord c;
    c.x=x;
    c.y=y;
    c.z=z;
    return c;
}

inline Coord newCoord(float x,float y,float z,float yaw,float pitch,float roll){
    Coord c;
    c.x=x;
    c.y=y;
    c.z=z;
    c.yaw=yaw;
    c.pitch=pitch;
    c.roll=roll;
    return c;
}

}

#endif // COORD_H
