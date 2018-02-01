#ifndef CONVOLUTIONS_H
#define CONVOLUTIONS_H

#include <Eigen/Dense>
#include <convolutions.h>
#include <pooling_result.h>

namespace htwk {

class Convolutions{
    public:
    static PoolingResult maxPool(Eigen::MatrixXf &inputs, int inputWidth, int inputHeight, int poolWidth, int poolHeight);
    static Eigen::MatrixXf generatePatchesFromInputLayer(Eigen::MatrixXf &inputs, int inputWidth, int inputHeight, int patchWidth, int patchHeight);
    static Eigen::MatrixXf generatePatchesFromHiddenLayer(Eigen::MatrixXf &inputs, int inputWidth, int inputHeight, int patchWidth, int patchHeight);
    static Eigen::MatrixXf movePatchesToColumns(Eigen::MatrixXf &inputs, int numExamples, int numFeatureMaps, int numPatches);
};

}  // namespace htwk

#endif // CONVOLUTIONS_H
