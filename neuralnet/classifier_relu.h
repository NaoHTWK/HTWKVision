#ifndef __CLASSIFIERRELU_H__
#define __CLASSIFIERRELU_H__

#include <cstdio>
#include <string>
#include <vector>
#include <Eigen/Dense>

#include <ext_math.h>
#include <layer.h>
#include <layer_activations.h>
#include <pooling_result.h>
#include <convolutions.h>

namespace htwk {

class ClassifierReLU{
private:
    int version;
    int numClasses;
    int numLayers;
    bool useSoftmax;
    float dropoutRate;
    float inputDropoutRate;
    int inputWidth;
    int inputHeight;
    bool useNormalization;
    std::vector<Layer> layers;
    std::vector<float> meanValues;
    std::vector<float> minmaxValues;
    std::vector<Eigen::MatrixXf> thetas;
    void rectify(Eigen::MatrixXf &m);
    void softmax(Eigen::MatrixXf &m);
    Eigen::MatrixXf propagateLayers(const Eigen::MatrixXf& x);
    void normalizeData(Eigen::MatrixXf &x, int col, float average, float standardDeviation);
    int getWidth(size_t layer);
    int getHeight(size_t layer);

public:
    explicit ClassifierReLU(const std::string& weightFile);
    Eigen::MatrixXf propagate(Eigen::MatrixXf &samples);
};

}  // namespace htwk

#endif  // __CLASSIFIERRELU_H__
