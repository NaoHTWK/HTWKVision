#include "ball_detector_legacy.h"

namespace htwk {

BallDetectorLegacy::BallDetectorLegacy(const int width, const int height,
                                       const int8_t *lutCb, const int8_t *lutCr,
                                       BallFeatureExtractor* _featureExtractor,
                                       const HtwkVisionConfig& config)
    : BallDetector(width, height, lutCb, lutCr)
    , MIN_PROB(config.ballProbabilityThreshold)
    , found(false)
    , featureExtractor(_featureExtractor)
{
    ballClassifierPre=new ClassifierReLU("./data/20160325_131810_NoRot_classifierPre.net");
    ballClassifier=new ClassifierReLU("./data/20160325_131810_NoRotB_classifier.net");
}

BallDetectorLegacy::~BallDetectorLegacy() {}

/*
 * detects the ball (if visible) and outputs its position
 */
void BallDetectorLegacy::proceed(const uint8_t *img, std::vector<ObjectHypothesis> &hypoList){
    ratedHypothesis.clear();
    found=false;

    if(hypoList.size()>1){
        for(ObjectHypothesis& hyp:hypoList){
            Eigen::MatrixXf featurePre = getFeature(hyp, img, FEATURE_SIZE_PRE);
            hyp.prob=ballClassifierPre->propagate(featurePre)(0,0);
        }
        std::sort(hypoList.begin(), hypoList.end(), [](const ObjectHypothesis& a, const ObjectHypothesis& b){return a.prob > b.prob;});
    }

    int cnt=0;
    float maxRating=MIN_PROB;
    for(ObjectHypothesis &hyp:hypoList){
        Eigen::MatrixXf feature=getFeature(hyp,img,FEATURE_SIZE);
        const float prob=ballClassifier->propagate(feature)(0,0);

        if(prob>maxRating){
            maxRating=hyp.prob;
            hyp.prob = prob;
            ratedHypothesis.push_back(hyp);
            bestHypothesis=hyp;
        }

        if(++cnt>=NUM_EVALUATIONS){
            break;
        }
    }
    if(maxRating>MIN_PROB){
        found=true;
    }
}

Eigen::MatrixXf BallDetectorLegacy::getFeature(const ObjectHypothesis &p, const uint8_t *img, const int size){
    float patch[size*size];
    featureExtractor->getFeature(p, img, size, patch);

    Eigen::MatrixXf feature=Eigen::MatrixXf::Zero(1,size*size);

    for(int i=0;i<size*size;i++){
        feature(i)=patch[i];
    }

    return feature;
}
}  // namespace htwk
