#ifndef POOLING_RESULT_H
#define POOLING_RESULT_H

#include <Eigen/Dense>
#include <utility>

class PoolingResult{
    public:
    Eigen::MatrixXf pooledActivations;
    Eigen::MatrixXf prePoolRowIndexes;
    PoolingResult(Eigen::MatrixXf pooledActivations, Eigen::MatrixXf prePoolRowIndexes):pooledActivations(std::move(pooledActivations)),prePoolRowIndexes(std::move(prePoolRowIndexes)){}

};

#endif // POOLING_RESULT_H
