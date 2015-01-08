#include <htwk_vision.h>

#include <cstdlib>

using namespace std;

namespace htwk {

HTWKVision::HTWKVision(int width, int height): width(width), height(height) {
    createAdressLookups();
    fieldColorDetector=new FieldColorDetector(width, height, lutCb, lutCr);
    fieldDetector=new FieldDetector(width, height, lutCb, lutCr);
    regionClassifier=new RegionClassifier(lutCb, lutCr, width, height);
    goalDetector=new GoalDetector(width, height, lutCb, lutCr);
    lineDetector=new LineDetector(width, height, lutCb, lutCr);
    ballDetector=new BallDetector(width, height, lutCb, lutCr);
    feetDetector=new FeetDetector(width, height, lutCb, lutCr);
    ellipseFitter=new RansacEllipseFitter();
    robotAreaDetector=new RobotAreaDetector(width,height, lutCb, lutCr, regionClassifier->getScanVerticalSize());
    robotClassifier=new RobotClassifier(width, height, lutCb, lutCr);
}

HTWKVision::~HTWKVision(){}

void HTWKVision::proceed(uint8_t *img, bool is_upper_cam, bool use_feet_detection){
    fieldColorDetector->proceed(img);
    regionClassifier->proceed(img, fieldColorDetector);
    fieldDetector->proceed(	img, fieldColorDetector, regionClassifier);
    lineDetector->proceed(	img,
                            regionClassifier->getLineSegments(fieldDetector->getConvexFieldBorder()),
                            regionClassifier->lineSpacing);
    goalDetector->proceed(	img,
                            fieldDetector->getConvexFieldBorder(),
                            fieldColorDetector->getColor(),
                            lineDetector->getColor());
    ballDetector->proceed(	img,
                            fieldDetector->getConvexFieldBorder(),
                            fieldColorDetector->getColor(),
                            lineDetector->getColor(),
                            goalDetector->getColor());
    ellipseFitter->proceed(regionClassifier->getLineSegments(fieldDetector->getConvexFieldBorder()),lineDetector->lineEdges);

    if(!is_upper_cam){
        feetDetector->proceed(	img, fieldColorDetector, ballDetector->getColor(),ballDetector->isBallFound(), use_feet_detection);
    }

    resultRobotClassifier.clear();
    if(is_upper_cam) {  //TODO robot detection currently only for upper camera
        robotAreaDetector->proceed(img, regionClassifier->getScanVertical(), fieldDetector->getConvexFieldBorder(),
                                fieldColorDetector,ballDetector->getBall(),goalDetector->getGoalPosts());
        int rectCounter =0;
        for(const Rect &rect : *robotAreaDetector->getRobotAreas()) {
            rectCounter++;
            classifierResult res;
            robotClassifier->proceed(img, rect, res);
            if(res.isRobot){
                resultRobotClassifier.push_back(res);
            }
        }
    }
}

/**
 * creates a lookup table for fast access to the yuv422 pixel form in the camera image
 * just use the markos getY(), getCb() and getCr() to get the pixel value from a given coordinate
 */
void HTWKVision::createAdressLookups(){
    lutCb=(int*)malloc(sizeof(int)*width);
    lutCr=(int*)malloc(sizeof(int)*width);
    for(int i=1;i<width;i++){
        if((i&1)==0){
            lutCb[i]=5;
            lutCr[i]=3;
        }else{
            lutCb[i]=3;
            lutCr[i]=5;
        }
        while(lutCr[i]+i*2>=width*2)
            lutCr[i]-=4;
        while(lutCb[i]+i*2>=width*2)
            lutCb[i]-=4;
    }
    lutCb[0]=1;
    lutCr[0]=3;
}

}  // namespace htwk
