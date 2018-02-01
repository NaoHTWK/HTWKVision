#include "htwk_vision.h"

#include "hypotheses_generator_scanlines.h"

namespace htwk {

HTWKVision::HTWKVision(int width, int height, HtwkVisionConfig  _config)
    : width(width)
//    , height(height)
    , config(std::move(_config))
{
    createAdressLookups();
    fieldColorDetector=new FieldColorDetector(width, height, lutCb, lutCr);
    fieldDetector=new FieldDetector(width, height, lutCb, lutCr);
    regionClassifier=new RegionClassifier(width, height, config.isUpperCam, lutCb, lutCr);
    goalDetector=new GoalDetector(width, height, lutCb, lutCr);
    lineDetector=new LineDetector(width, height, lutCb, lutCr);
    ballFeatureExtractor=new BallFeatureExtractor(width, height, lutCb, lutCr);
    feetDetector=new FeetDetector(width, height, lutCb, lutCr);
    ellipseFitter=new RansacEllipseFitter();
    robotAreaDetector=new RobotAreaDetector(width,height, lutCb, lutCr, regionClassifier->getScanVerticalSize());
    robotClassifier=new RobotClassifier(width, height, lutCb, lutCr);
    nearObstacleDetector=new NearObstacleDetector(width, height, lutCb, lutCr);
    jerseyColorDetector=new JerseyColorDetector(width, height, lutCb, lutCr);
    integralImage=new IntegralImage(width, height, lutCb, lutCr);
    hypothesesGenerator=new HypothesesGeneratorScanlines(width, height, lutCb, lutCr, config);
    robotDetector=new RobotDetector(width, height, lutCb, lutCr, ballFeatureExtractor, config);
    
    if(config.activateObjectDetector)
    {
        objectDetector = new ObjectDetector(width, height, lutCb, lutCr, ballFeatureExtractor, config);
        ballDetector = objectDetector;
    }
    else
    {
        ballDetector = new BallDetectorLegacy(width, height, lutCb, lutCr, ballFeatureExtractor, config);
        objectDetector = nullptr;
    }
}

HTWKVision::~HTWKVision(){
    delete fieldColorDetector;
    delete fieldDetector;
    delete regionClassifier;
    delete lineDetector;
    delete goalDetector;
    delete ballFeatureExtractor;

    if(ballDetector == objectDetector) {
        delete ballDetector;
        ballDetector = nullptr;
        objectDetector = nullptr;
    } else {
        delete ballDetector;
        delete objectDetector;
    }

    delete ellipseFitter;
    delete feetDetector;
    delete robotAreaDetector;
    delete robotClassifier;
    delete nearObstacleDetector;
    delete jerseyColorDetector;
    delete integralImage;
    delete hypothesesGenerator;
    //delete robotDetector;

    free(lutCb);
    free(lutCr);
}

void HTWKVision::proceed(uint8_t *img, bool use_feet_detection, float pitch, float roll){

    if(enableProfiling) getTime(tIntegralImage);
    integralImage->proceed(img);

    if(enableProfiling) getTime(tFieldColorDetector);
    fieldColorDetector->proceed(img);

    if(enableProfiling) getTime(tRegionClassifier);
    regionClassifier->proceed(img, fieldColorDetector);

    if(enableProfiling) getTime(tFieldDetector);
    fieldDetector->proceed(	img, fieldColorDetector, regionClassifier, config.isUpperCam);

    if(enableProfiling) getTime(tLineDetector);
    lineDetector->proceed(	img,
                            regionClassifier->getLineSegments(fieldDetector->getConvexFieldBorder()),
                            regionClassifier->lineSpacing);

    if(enableProfiling) getTime(tGoalDetector);
//    goalDetector->proceed(	img,
//                            fieldDetector->getConvexFieldBorder(),
//                            fieldColorDetector->getColor(),
//                            lineDetector->getColor());

    if(enableProfiling) getTime(tHypoGenerator);
    hypothesesGenerator->proceed(   img,
                                    fieldDetector->getConvexFieldBorder(),
                                    pitch,
                                    roll,
                                    integralImage);
    std::vector<ObjectHypothesis> hypotheses = hypothesesGenerator->getHypotheses();
    if(enableProfiling) getTime(tBallDetector);
    ballDetector->proceed(img, hypotheses);

    if(enableProfiling) getTime(tFeetDetector);
//    robotDetector->proceed( img,
//                            hypotheses);

    if(enableProfiling) getTime(tNearObstacleDetector);
    if(!config.isUpperCam){
        const color c{180, 180, 100};
        feetDetector->proceed(	img, fieldColorDetector, c, ballDetector->isBallFound(), use_feet_detection);
        if(enableProfiling) getTime(tNearObstacleDetector);
        nearObstacleDetector->proceed(img, fieldColorDetector);
    }

    if(config.isUpperCam) {  //TODO robot detection currently only for upper camera
        if(enableProfiling) getTime(tEllipseFitter);
        ellipseFitter->proceed(regionClassifier->getLineSegments(fieldDetector->getConvexFieldBorder()));

        if(enableProfiling) getTime(tRobotHypotheses);
//        robotAreaDetector->proceed(regionClassifier->getScanVertical(), fieldDetector->getConvexFieldBorder(),
//                                                           pitch, roll);

        if(enableProfiling) getTime(tRobotClassifier);
//        resultRobotClassifier.clear();
//        int rectCounter=0;
//        for(RobotRect rect : *robotAreaDetector->getRobotAreas()) {
//            rectCounter++;
//            RobotClassifierResult res;
//            robotClassifier->proceed(img, rect, res);

//            resultRobotClassifier.push_back(res);
//        }
    }
    if(enableProfiling) getTime(tJersey);
    if(enableProfiling){
        getTime(tEnd);
        createProfilingStats();
    }
}

static float diff(timespec end, timespec start)
{
    timespec temp;
    if ((end.tv_nsec-start.tv_nsec)<0) {
        temp.tv_sec = end.tv_sec-start.tv_sec-1;
        temp.tv_nsec = 1000000000+end.tv_nsec-start.tv_nsec;
    } else {
        temp.tv_sec = end.tv_sec-start.tv_sec;
        temp.tv_nsec = end.tv_nsec-start.tv_nsec;
    }
    return temp.tv_nsec/1e6;
}

void HTWKVision::createProfilingStats()
{
    float dCreateIntegralImage  = diff(tFieldColorDetector, tIntegralImage);
    float dFieldColorDetector   = diff(tRegionClassifier, tFieldColorDetector);
    float dRegionClassifier     = diff(tFieldDetector, tRegionClassifier);
    float dFieldDetector        = diff(tLineDetector, tFieldDetector);
    float dLineDetector         = diff(tGoalDetector, tLineDetector);
    float dGoalDetector         = diff(tHypoGenerator, tGoalDetector);
    float dHypoGenerator        = diff(tBallDetector, tHypoGenerator);
    float dBallDetector         = diff(tFeetDetector, tBallDetector);
    float dFeetDetector         = diff(tNearObstacleDetector, tFeetDetector);
    float dNearObstacleDetector = diff(tEllipseFitter, tNearObstacleDetector);
    float dEllipseFitter        = diff(tRobotHypotheses, tEllipseFitter);
    float dRobotHypotheses      = diff(tRobotClassifier, tRobotHypotheses);
    float dRobotClassifier      = diff(tJersey, tRobotClassifier);
    //float dJersey               = diff(tIntegralImage, tJersey);
    float dTotal                = diff(tEnd, tFieldColorDetector);

    cntFieldColorDetector   += dFieldColorDetector;
    cntRegionClassifier     += dRegionClassifier;
    cntFieldDetector        += dFieldDetector;
    cntLineDetector         += dLineDetector;
    cntGoalDetector         += dGoalDetector;
    cntHypoGenerator        += dHypoGenerator;
    cntBallDetector         += dBallDetector;
    cntFeetDetector         += dFeetDetector;
    cntNearObstacleDetector += dNearObstacleDetector;
    cntEllipseFitter        += dEllipseFitter;
    cntRobotHypotheses      += dRobotHypotheses;
    cntRobotClassifier      += dRobotClassifier;
    cntJersey               += 0;//dJersey;
    cntCreateIntegralImage  += dCreateIntegralImage;
    cntTotal                += dTotal;

    maxFieldColorDetector   = std::max(maxFieldColorDetector,   dFieldColorDetector);
    maxRegionClassifier     = std::max(maxRegionClassifier,     dRegionClassifier);
    maxFieldDetector        = std::max(maxFieldDetector,        dFieldDetector);
    maxLineDetector         = std::max(maxLineDetector,         dLineDetector);
    maxGoalDetector         = std::max(maxGoalDetector,         dGoalDetector);
    maxHypoGenerator        = std::max(maxHypoGenerator,        dHypoGenerator);
    maxBallDetector         = std::max(maxBallDetector,         dBallDetector);
    maxFeetDetector         = std::max(maxFeetDetector,         dFeetDetector);
    maxNearObstacleDetector = std::max(maxNearObstacleDetector, dNearObstacleDetector);
    maxEllipseFitter        = std::max(maxEllipseFitter,        dEllipseFitter);
    maxRobotHypotheses      = std::max(maxRobotHypotheses,      dRobotHypotheses);
    maxRobotClassifier      = std::max(maxRobotClassifier,      dRobotClassifier);
    maxJersey               = 0;//std::max(maxJersey,               dJersey);
    maxCreateIntegralImage  = std::max(maxCreateIntegralImage,  dCreateIntegralImage);

    minFieldColorDetector   = std::min(minFieldColorDetector,   dFieldColorDetector);
    minRegionClassifier     = std::min(minRegionClassifier,     dRegionClassifier);
    minFieldDetector        = std::min(minFieldDetector,        dFieldDetector);
    minLineDetector         = std::min(minLineDetector,         dLineDetector);
    minGoalDetector         = std::min(minGoalDetector,         dGoalDetector);
    minHypoGenerator        = std::min(minHypoGenerator,        dHypoGenerator);
    minBallDetector         = std::min(minBallDetector,         dBallDetector);
    minFeetDetector         = std::min(minFeetDetector,         dFeetDetector);
    minNearObstacleDetector = std::min(minNearObstacleDetector, dNearObstacleDetector);
    minEllipseFitter        = std::min(minEllipseFitter,        dEllipseFitter);
    minRobotHypotheses      = std::min(minRobotHypotheses,      dRobotHypotheses);
    minRobotClassifier      = std::min(minRobotClassifier,      dRobotClassifier);
    minJersey               = 0;//std::min(minJersey,               dJersey);
    minCreateIntegralImage  = std::min(minCreateIntegralImage,  dCreateIntegralImage);

    maxTotal = minFieldColorDetector + maxRegionClassifier + maxFieldDetector + maxLineDetector + maxGoalDetector + maxHypoGenerator + maxBallDetector + maxFeetDetector + maxNearObstacleDetector + maxEllipseFitter + maxRobotHypotheses + maxRobotClassifier + maxJersey + maxCreateIntegralImage;
    minTotal = minFieldColorDetector + minRegionClassifier + minFieldDetector + minLineDetector + minGoalDetector + minHypoGenerator + minBallDetector + minFeetDetector + minNearObstacleDetector + minEllipseFitter + minRobotHypotheses + minRobotClassifier + minJersey + minCreateIntegralImage;
    cntImage++;
}


inline void HTWKVision::writeProfilHeader(const std::string& fileName) {
    if (FILE *check = fopen(fileName.c_str(), "r")) {
        fclose(check);
    }else{
        std::ofstream file;
        file.open(fileName, std::ios::out | std::ios::app);
        if (file.is_open()) {
            file << "image;";
            writeHead(file,";","total");
            writeHead(file,";","FieldColorDetector");
            writeHead(file,";","RegionClassifier");
            writeHead(file,";","FieldDetector");
            writeHead(file,";","LineDetector");
            writeHead(file,";","GoalDetector");
            writeHead(file,";","CreateIntegralImage");
            writeHead(file,";","HypoGenerator");
            writeHead(file,";","BallDetector");
            writeHead(file,";","FeetDetector");
            writeHead(file,";","NearObstacleDetector");
            writeHead(file,";","EllipseFitter");
            writeHead(file,";","RobotHypotheses");
            writeHead(file,";","RobotClassifier");
            file << std::endl;
            file << ";";
            for (int i=0;i<14;i++)
                file << "min;avg;max;";
            file << std::endl;
            file.close();
        }
    }
}

void HTWKVision::writeProfilingFile(const std::string& fileName, const std::string& imageName)
{
    if(enableProfiling && cntImage > 0) {
        writeProfilHeader(fileName);

        std::ofstream file;
        file.open(fileName, std::ios::out | std::ios::app);
        if (file.is_open()) {
            std::string sep = ";";
            file << imageName << sep;
            writeData(file,sep,minTotal,                (cntTotal/cntImage),                 maxTotal);
            writeData(file,sep,minFieldColorDetector,   (cntFieldColorDetector/cntImage),    maxFieldColorDetector);
            writeData(file,sep,minRegionClassifier,     (cntRegionClassifier/cntImage),      maxRegionClassifier);
            writeData(file,sep,minFieldDetector,        (cntFieldDetector/cntImage),         maxFieldDetector);
            writeData(file,sep,minLineDetector,         (cntLineDetector/cntImage),          maxLineDetector);
            writeData(file,sep,minGoalDetector,         (cntGoalDetector/cntImage),          maxGoalDetector);
            writeData(file,sep,minCreateIntegralImage,  (cntCreateIntegralImage/cntImage),   maxCreateIntegralImage);
            writeData(file,sep,minHypoGenerator,        (cntHypoGenerator/cntImage),         maxHypoGenerator);
            writeData(file,sep,minBallDetector,         (cntBallDetector/cntImage),          maxBallDetector);
            writeData(file,sep,minFeetDetector,         (cntFeetDetector/cntImage),          maxFeetDetector);
            writeData(file,sep,minNearObstacleDetector, (cntNearObstacleDetector/cntImage),  maxNearObstacleDetector);
            writeData(file,sep,minEllipseFitter,        (cntEllipseFitter/cntImage),         maxEllipseFitter);
            writeData(file,sep,minRobotHypotheses,      (cntRobotHypotheses/cntImage),       maxRobotHypotheses);
            writeData(file,sep,minRobotClassifier,      (cntRobotClassifier/cntImage),       maxRobotClassifier);
            file << std::endl;
            file.close();
        }
    }
}

void HTWKVision::printProfilingResults(bool isUpperCam)
{
    if(enableProfiling && cntImage > 0) {
        printf("avg total: %06.2fms, fc: %05.2fms, rc: %05.2fms, fd: %05.2fms, ld: %05.2fms, gd: %05.2fms, hg: %05.2fms, bd: %05.2fms, fd: %05.2fms, nod: %05.2fms, ef: %05.2fms, rh: %05.2fms, rc: %05.2fms, ii: %05.5fms\n"
               //"max total: %06.2fms, fc: %05.2fms, rc: %05.2fms, fd: %05.2fms, ld: %05.2fms, gd: %05.2fms, hg: %05.2fms, bd: %05.2fms, fd: %05.2fms, nod: %05.2fms, ef: %05.2fms, rh: %05.2fms, rc: %05.2fms, ii: %05.5fms\n"
               "min total: %06.2fms, fc: %05.2fms, rc: %05.2fms, fd: %05.2fms, ld: %05.2fms, gd: %05.2fms, hg: %05.2fms, bd: %05.2fms, fd: %05.2fms, nod: %05.2fms, ef: %05.2fms, rh: %05.2fms, rc: %05.2fms, ii: %05.5fms\n\n",
               cntTotal/cntImage,
               cntFieldColorDetector/cntImage,
               cntRegionClassifier/cntImage,
               cntFieldDetector/cntImage,
               cntLineDetector/cntImage,
               cntGoalDetector/cntImage,
               cntHypoGenerator/cntImage,
               cntBallDetector/cntImage,
               cntFeetDetector/cntImage,
               cntNearObstacleDetector/cntImage,
               cntEllipseFitter/cntImage,
               cntRobotHypotheses/cntImage,
               cntRobotClassifier/cntImage,
               //cntJersey/cntImage,
               cntCreateIntegralImage/cntImage,
//               maxTotal,
//               maxFieldColorDetector,
//               maxRegionClassifier,
//               maxFieldDetector,
//               maxLineDetector,
//               maxGoalDetector,
//               maxHypoGenerator,
//               maxBallDetector,
//               maxFeetDetector,
//               maxNearObstacleDetector,
//               maxEllipseFitter,
//               maxRobotHypotheses,
//               maxRobotClassifier,
//               //maxJersey,
//               maxCreateIntegralImage,
               minTotal,
               minFieldColorDetector,
               minRegionClassifier,
               minFieldDetector,
               minLineDetector,
               minGoalDetector,
               minHypoGenerator,
               minBallDetector,
               minFeetDetector,
               minNearObstacleDetector,
               minEllipseFitter,
               minRobotHypotheses,
               minRobotClassifier,
               //minJersey,
               minCreateIntegralImage
               );
        fflush(stdout);
        resetProfilingStats(isUpperCam);
    }
}

void HTWKVision::resetProfilingStats(bool isUpperCam)
{
    cntImage = 0;

    cntFieldColorDetector=0;
    cntRegionClassifier=0;
    cntFieldDetector=0;
    cntLineDetector=0;
    cntGoalDetector=0;
    cntCreateIntegralImage=0;
    cntHypoGenerator=0;
    cntBallDetector=0;
    cntFeetDetector=0;
    cntNearObstacleDetector=0;
    cntEllipseFitter=0;
    cntRobotHypotheses=0;
    cntRobotClassifier=0;
    cntJersey=0;
    cntTotal=0;

    maxFieldColorDetector=0;
    maxRegionClassifier=0;
    maxFieldDetector=0;
    maxLineDetector=0;
    maxGoalDetector=0;
    maxCreateIntegralImage=0;
    maxHypoGenerator=0;
    maxBallDetector=0;
    maxFeetDetector=0;
    maxNearObstacleDetector=0;
    maxEllipseFitter=0;
    maxRobotHypotheses=0;
    maxRobotClassifier=0;
    maxJersey=0;
    maxTotal=0;

    minFieldColorDetector=99999999;
    minRegionClassifier=99999999;
    minFieldDetector=99999999;
    minLineDetector=99999999;
    minGoalDetector=99999999;
    minCreateIntegralImage=99999999;
    minHypoGenerator=99999999;
    minBallDetector=99999999;
    minFeetDetector=99999999;
    minNearObstacleDetector=99999999;
    minEllipseFitter=isUpperCam ? 99999999 : 0;
    minRobotHypotheses=isUpperCam ? 99999999 : 0;
    minRobotClassifier=isUpperCam ? 99999999 : 0;
    minJersey=isUpperCam ? 99999999 : 0;
    minTotal=99999999;
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
    for(int i=0;i<width;i++){
        if((i&1)==0){
            lutCb[i]=1;
            lutCr[i]=3;
        }else{
            lutCb[i]=-1;
            lutCr[i]=1;
        }
    }
}

}  // namespace htwk
