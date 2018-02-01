#ifndef LAYER_ACTIVATIONS_H
#define LAYER_ACTIVATIONS_H

#include <Eigen/Dense>

class LayerActivations{
    public:
    Eigen::MatrixXf input;
    Eigen::MatrixXf output;
    Eigen::MatrixXf prePoolRowIndexes;
    int numRowsPrePool=0;
};

#endif // LAYER_ACTIVATIONS_H
