#ifndef ROBOTRECT_H
#define ROBOTRECT_H

namespace htwk {

struct RobotRect {
	int xLeft,xRight,yTop,yBottom;
	float redTeamProbability;
	float detectionProbability;
	float greenCenterRatio;
	float greenSideRatio;
	float greenBottomRatio;
};

}

#endif // ROBOTRECT_H
