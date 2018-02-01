#ifndef LAYER_H
#define LAYER_H

class Layer{
    public:
    int numFeatures=0;
    int patchWidth=0;
    int patchHeight=0;
    int poolWidth=0;
    int poolHeight=0;
    Layer(){}

    /**
     * Create a definition of a fully connected layer.
     */
    Layer(int numFeatures):numFeatures(numFeatures){}

    /**
     * Create a definition of a convolutional connected layer.
     */
    Layer(int numFeatures, int patchWidth, int patchHeight, int poolWidth, int poolHeight):numFeatures(numFeatures),patchWidth(patchWidth),patchHeight(patchHeight),poolWidth(poolWidth),poolHeight(poolHeight){}

//    ~Layer();

    bool isConvolutional(){
        return patchWidth>0&&patchHeight>0;
    }

    bool isPooled(){
        return poolWidth>1||poolHeight>1;
    }
};

#endif // LAYER_H
