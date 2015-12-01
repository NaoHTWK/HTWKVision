#include <htwk_vision.h>

#include <cstdlib>
#include <cstdio>

using namespace std::chrono;

namespace htwk {

HTWKVision::HTWKVision(int width, int height): width(width), height(height) {
    createAdressLookups();
    fieldColorDetector=new FieldColorDetector(width, height, lutCb, lutCr);
    fieldDetector=new FieldDetector(width, height, lutCb, lutCr);
    regionClassifier=new RegionClassifier(width, height, lutCb, lutCr);
    goalDetector=new GoalDetector(width, height, lutCb, lutCr);
    lineDetector=new LineDetector(width, height, lutCb, lutCr);
    ballDetector=new BallDetector(width, height, lutCb, lutCr);
    feetDetector=new FeetDetector(width, height, lutCb, lutCr);
    ellipseFitter=new RansacEllipseFitter();
    robotAreaDetector=new RobotAreaDetector(width,height, lutCb, lutCr, regionClassifier->getScanVerticalSize());
    robotClassifier=new RobotClassifier(width, height, lutCb, lutCr);
    nearObstacleDetector = new NearObstacleDetector(width, height, lutCb, lutCr);
    jerseyColorDetector = new JerseyColorDetector(width, height, lutCb, lutCr);
}

HTWKVision::~HTWKVision(){}

void HTWKVision::proceed(uint8_t *img, bool is_upper_cam, bool use_feet_detection){

    if(enableProfiling) tFieldColorDetector = high_resolution_clock::now();
    fieldColorDetector->proceed(img);

    if(enableProfiling) tRegionClassifier = high_resolution_clock::now();
    regionClassifier->proceed(img, fieldColorDetector);

    if(enableProfiling) tFieldDetector = high_resolution_clock::now();
    fieldDetector->proceed(	img, fieldColorDetector, regionClassifier);

    if(enableProfiling) tLineDetector = high_resolution_clock::now();
    lineDetector->proceed(	img,
                            regionClassifier->getLineSegments(fieldDetector->getConvexFieldBorder()),
                            regionClassifier->lineSpacing);

    if(enableProfiling) tGoalDetector = high_resolution_clock::now();
    goalDetector->proceed(	img,
                            fieldDetector->getConvexFieldBorder(),
                            fieldColorDetector->getColor(),
                            lineDetector->getColor());

    if(enableProfiling) tBallDetector = high_resolution_clock::now();
    ballDetector->proceed(	img,
                            fieldDetector->getConvexFieldBorder(),
                            fieldColorDetector->getColor(),
                            lineDetector->getColor(),
                            goalDetector->getColor());

    if(enableProfiling) tFeetDetector         = high_resolution_clock::now();
    if(enableProfiling) tNearObstacleDetector = high_resolution_clock::now();

    if(!is_upper_cam){        
        feetDetector->proceed(	img, fieldColorDetector, ballDetector->getColor(),ballDetector->isBallFound(), use_feet_detection);
        if(enableProfiling) tNearObstacleDetector = high_resolution_clock::now();
        nearObstacleDetector->proceed(img, fieldColorDetector);
    }

    if(enableProfiling) tEllipseFitter = high_resolution_clock::now();
    if(enableProfiling) tRobotArea = high_resolution_clock::now();
    if(enableProfiling) tJersey = high_resolution_clock::now();

    resultRobotClassifier.clear();
    if(is_upper_cam) {  //TODO robot detection currently only for upper camera
        ellipseFitter->proceed(regionClassifier->getLineSegments(fieldDetector->getConvexFieldBorder()),lineDetector->lineEdges);

        if(enableProfiling) tRobotArea = high_resolution_clock::now();
        cntCreateIntegralImage += robotAreaDetector->proceed(img, regionClassifier->getScanVertical(), fieldDetector->getConvexFieldBorder(),
                                                           fieldColorDetector,ballDetector->getBall(),goalDetector->getGoalPosts());

        tJersey = high_resolution_clock::now();
        int rectCounter =0;
        for(RobotRect rect : *robotAreaDetector->getRobotAreas()) {
            rectCounter++;
            RobotClassifierResult res;
            robotClassifier->proceed(img, rect, res);

            res.teamColor=TeamMembership::NONE;
            if(res.detectionProbability > 0.125f){
                res.isBlueJersey = jerseyColorDetector->isBlueJersey(img, rect);

                if(res.isBlueJersey > 0.67f){
                    res.teamColor = TeamMembership::BLUE;
                } else if(res.isBlueJersey < 0.33f){
                    res.teamColor = TeamMembership::RED;
                }
            }

            resultRobotClassifier.push_back(res);
        }
    }

    printProfilingResults();
}

void HTWKVision::printProfilingResults()
{
    if(enableProfiling) {
        high_resolution_clock::time_point tEnd = high_resolution_clock::now();

        static const int IMG_CNT = 30*5;

        duration<double> dFieldColorDetector   = duration_cast<duration<double>>(tRegionClassifier - tFieldColorDetector);
        duration<double> dRegionClassifier     = duration_cast<duration<double>>(tFieldDetector - tRegionClassifier);
        duration<double> dFieldDetector        = duration_cast<duration<double>>(tLineDetector - tFieldDetector);
        duration<double> dLineDetector         = duration_cast<duration<double>>(tGoalDetector - tLineDetector);
        duration<double> dGoalDetector         = duration_cast<duration<double>>(tBallDetector - tGoalDetector);
        duration<double> dBallDetector         = duration_cast<duration<double>>(tFeetDetector - tBallDetector);
        duration<double> dFeetDetector         = duration_cast<duration<double>>(tNearObstacleDetector - tFeetDetector);
        duration<double> dNearObstacleDetector = duration_cast<duration<double>>(tEllipseFitter - tNearObstacleDetector);
        duration<double> dEllipseFitter        = duration_cast<duration<double>>(tRobotArea - tEllipseFitter);
        duration<double> dRobotArea            = duration_cast<duration<double>>(tJersey - tRobotArea);
        duration<double> dJersey               = duration_cast<duration<double>>(tEnd - tJersey);
        duration<double> dTotal                = duration_cast<duration<double>>(tEnd - tFieldColorDetector);

        cntFieldColorDetector   += dFieldColorDetector.count()*1000;
        cntRegionClassifier     += dRegionClassifier.count()*1000;
        cntFieldDetector        += dFieldDetector.count()*1000;
        cntLineDetector         += dLineDetector.count()*1000;
        cntGoalDetector         += dGoalDetector.count()*1000;
        cntBallDetector         += dBallDetector.count()*1000;
        cntFeetDetector         += dFeetDetector.count()*1000;
        cntNearObstacleDetector += dNearObstacleDetector.count()*1000;
        cntEllipseFitter        += dEllipseFitter.count()*1000;
        cntRobotArea            += dRobotArea.count()*1000;
        cntJersey               += dJersey.count()*1000;
        cntTotal                += dTotal.count()*1000;
        cntImages++;

        if(cntImages == IMG_CNT) {
            cntImages = 0;
            printf("total: %.fms, fc: %.2fms, rc: %.2fms, fd: %.2fms, ld: %.2fms, gd: %.2fms, bd: %.2fms, fd: %.2fms, nod: %.2fms, ef: %.2fms, ra: %.2fms, jd: %.2fms, ii: %.2f\n",
                   cntTotal/IMG_CNT,
                   cntFieldColorDetector/IMG_CNT,
                   cntRegionClassifier/IMG_CNT,
                   cntFieldDetector/IMG_CNT,
                   cntLineDetector/IMG_CNT,
                   cntGoalDetector/IMG_CNT,
                   cntBallDetector/IMG_CNT,
                   cntFeetDetector/IMG_CNT,
                   cntNearObstacleDetector/IMG_CNT,
                   cntEllipseFitter/IMG_CNT,
                   cntRobotArea/IMG_CNT,
                   cntJersey/IMG_CNT,
                   cntCreateIntegralImage/IMG_CNT
                   );
            cntFieldColorDetector=0;
            cntRegionClassifier=0;
            cntFieldDetector=0;
            cntLineDetector=0;
            cntGoalDetector=0;
            cntBallDetector=0;
            cntFeetDetector=0;
            cntNearObstacleDetector=0;
            cntEllipseFitter=0;
            cntRobotArea=0;
            cntJersey=0;
            cntCreateIntegralImage=0;
            cntTotal=0;
        }
    }
}


/**
 * creates a lookup table for fast access to the yuv422 pixel form in the camera image
 * just use the markos getY(), getCb() and getCr() to get the pixel value from a given coordinate
 *
 * Y0 Cb0  Y1 Cr0  Y2 Cb1  Y3 Cr1 Y4 Cb2 Y5 Cr2 Y6 Cb3 Y7 Cr3
 *  0   1   2   3   4   5   6   7  8   9 10  11 12  13 14  15
 *
 * (Y0,Cb0,Cr0), (Y1,Cb1,Cr1), (Y2,Cb2,Cr1), (Y3,Cb2,Cr2), (Y4,Cb3,Cr2), (Y5,Cb3,Cr3)
 * 0 -> ( 0, 1, 2),
 * 1 -> ( 2, 5, 7),
 * 3 -> ( 6, 9,11),
 * 4 -> ( 8,13,15).
 * 5 -> (10,13,15),
 * 6 -> (12,17,15),
 * 7 -> (14,17,19)

 * 0 -> ( 0, 1, 2),
 * 2 -> ( 4, 9, 7),
 * 4 -> ( 8,13,15).
 * 6 -> (12,17,15),

 */
void HTWKVision::createAdressLookups(){
    lutCb=(int8_t*)malloc(sizeof(*lutCb)*width);
    lutCr=(int8_t*)malloc(sizeof(*lutCr)*width);
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
