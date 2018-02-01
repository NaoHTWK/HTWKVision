#include "classifier_relu.h"

#include <iostream>

using namespace ext_math;
using namespace Eigen;
using namespace std;

namespace htwk {

/**
 * propagates the activations through all neural network layers and returns the layer activations
 * @param samples one sample per row
 * @return returns the resulting class predictions
 */
MatrixXf ClassifierReLU::propagateLayers(const MatrixXf& x){
    int numExamples=x.rows();
    int numLayers=layers.size();
    vector<LayerActivations> output;
    output.resize(numLayers);
    output[0].output=x;
//    printf("propagate\n");
    for(int layer=1;layer<numLayers;layer++){
//        printf("LAYER %d\n",layer);
        if(layers[layer].isConvolutional()){
            int patchWidth=layers[layer].patchWidth;
            int patchHeight=layers[layer].patchHeight;
            int poolWidth=layers[layer].poolWidth;
            int poolHeight=layers[layer].poolHeight;
//            printf("patchWidth: %d\n",patchWidth);
//            printf("patchHeight: %d\n",patchHeight);
//            printf("poolWidth: %d\n",poolWidth);
//            printf("poolHeight: %d\n",poolHeight);

            output[layer].input=layer==1
                    ?Convolutions::generatePatchesFromInputLayer(output[layer-1].output,getWidth(layer-1),getHeight(layer-1),patchWidth,patchHeight)
                    :Convolutions::generatePatchesFromHiddenLayer(output[layer-1].output,getWidth(layer-1),getHeight(layer-1),patchWidth,patchHeight);
            MatrixXf bias(output[layer].input.rows(),output[layer].input.cols()+1);
            bias << MatrixXf::Ones(output[layer].input.rows(),1), output[layer].input;
            output[layer].input=std::move(bias); // Move into generatePatches() ?
//            printf("inputRows%d\n",output[layer].input.rows());
//            printf("inputCols%d\n",output[layer].input.cols());
//            printf("thetasRows%d\n",thetas[layer].rows());
//            printf("thetasCols%d\n",thetas[layer].cols());
            output[layer].output.noalias()=output[layer].input*thetas[layer].transpose();
//            printf("outputRows%d\n",output[layer].output.rows());
//            printf("outputCols%d\n",output[layer].output.cols());

            if(layers[layer].isPooled()){
                PoolingResult pr=Convolutions::maxPool(output[layer].output,(getWidth(layer-1)-patchWidth+1),(getHeight(layer-1)-patchHeight+1),poolWidth,poolHeight);
                output[layer].numRowsPrePool=output[layer].output.rows();

                output[layer].output=pr.pooledActivations;
                output[layer].prePoolRowIndexes=pr.prePoolRowIndexes;
            }
        }else{
            if(layer>1&&layers[layer-1].isConvolutional()){
                int numPatches=getWidth(layer-1)*getHeight(layer-1);
                int numFeatureMaps=layers[layer-1].numFeatures;
                output[layer-1].output=Convolutions::movePatchesToColumns(output[layer-1].output,numExamples,numFeatureMaps,numPatches);
            }
            double dr=layer-1==0?inputDropoutRate:dropoutRate;
            if(dr>0.0){
                output[layer-1].output*(1.0-dr);
            }

            MatrixXf biasCol(output[layer-1].output.rows(),1);
            biasCol.setOnes();
            MatrixXf bias(output[layer-1].output.rows(),output[layer-1].output.cols()+1);
            bias << biasCol, output[layer-1].output;
            output[layer].input=bias;
            output[layer].output=output[layer].input*thetas[layer].transpose();
        }


        if(layer<numLayers-1){
            rectify(output[layer].output);
        }else if(useSoftmax){
            softmax(output[layer].output);
        }
    }

    return output[numLayers-1].output;
}

void ClassifierReLU::softmax(MatrixXf &m){
    for(int row=0;row<m.rows();row++){
        double max=0.0;
        for(int col=0;col<m.cols();col++){
            double value=m(row,col);
            if(value>max){
                max=value;
            }
        }
        double sum=0.0;
        for(int col=0;col<m.cols();col++){
            double value=m(row,col);
            value-=max;
            value=exp(value);
            m(row,col)=value;
            sum+=value;
        }
        for(int col=0;col<m.cols();col++){
            if(sum>0){
                m(row,col)=m(row,col)/sum;
            }else{
                m(row,col)=1./m.cols();
            }

        }
    }
}

void ClassifierReLU::rectify(MatrixXf &m){
    for(int i=0;i<m.cols()*m.rows();i++){
        m(i)=max(0.f,m(i));
    }
}

/**
 * propagates the current sample activations through the neural network
 * @param samples one sample per row
 * @return returns the resulting class predictions
 */
MatrixXf ClassifierReLU::propagate(MatrixXf &samples){//copy samples, because internal normalization should not have an effect outside
    for(int col=0;col<samples.cols();col++){
        normalizeData(samples,col,meanValues[col],minmaxValues[col]);
    }
    return propagateLayers(samples);
}

void ClassifierReLU::normalizeData(MatrixXf &x, int col, float average, float standardDeviation){
    if(standardDeviation==0.0f){
        standardDeviation=1.0f;
    }
    for(int row=0;row<x.rows();row++){
        x(row,col)=(x(row,col)-average)/standardDeviation;
    }
}

int ClassifierReLU::getWidth(size_t layer){
    if(layer>0&&!layers[layer].isConvolutional()){ return 1; }
    int width=inputWidth;
    for(size_t l=1;l<=layer;l++){
        int patchWidth=layers[l].patchWidth;
        int poolWidth=layers[l].poolWidth;
        width=width-patchWidth+1;
        width=width%poolWidth==0?width/poolWidth:width/poolWidth+1;
    }
    return width;
}

int ClassifierReLU::getHeight(size_t layer){
    if(layer>0&&!layers[layer].isConvolutional()){ return 1; }
    int height=inputHeight;
    for(size_t l=1;l<=layer;l++){
        int patchHeight=layers[l].patchHeight;
        int poolHeight=layers[l].poolHeight;
        height=height-patchHeight+1;
        height=height%poolHeight==0?height/poolHeight:height/poolHeight+1;
    }
    return height;
}

ClassifierReLU::ClassifierReLU(const string& weightFile){
    FILE *fp=fopen(weightFile.c_str(),"r");
    if (fp == nullptr) {
        printf("Could not find %s.\n", weightFile.c_str());
        exit(-1);
    }
    float lr;

    int ret=fscanf(fp,"%d;%d;%f;%f;%f;%d;%d;%d;",&version,&numClasses,&dropoutRate,&inputDropoutRate,&lr,&inputWidth,&inputHeight,&numLayers);
    if(ret!=8){
        printf("reading net file failed!\n");
        exit(-1);
    }
    if(version!=2){
        printf("wrong version of classifier weight file!\n");
        exit(-1);
    }
//    printf("file: %s\n",weightFile.c_str());
//    printf("version: %d\n",version);
//    printf("numClasses: %d\n",numClasses);
//    printf("dropoutRate: %f\n",dropoutRate);
//    printf("inputDropoutRate: %f\n",inputDropoutRate);
//    printf("lr: %f\n",lr);
//    printf("inputWidth: %d\n",inputWidth);
//    printf("inputHeight: %d\n",inputHeight);
//    printf("numLayers: %d\n",numLayers);
    if(inputHeight==0){
        inputHeight=inputWidth;
    }
    useSoftmax=numClasses>1;
    layers=vector<Layer>(numLayers);
    thetas.resize(numLayers);
    for(int i=1;i<numLayers;i++){
//        printf("\nLayer %d:\n",i);
        int numFeatures;
        int patchWidth;
        int patchHeight;
        int poolWidth;
        int poolHeight;
        int numRows;
        int numCols;
        int rc = fscanf(fp,"%d;%d;%d;%d;%d;%d;%d;",&numFeatures,&patchWidth,&patchHeight,&poolWidth,&poolHeight,&numRows,&numCols);

        if(rc != 7) {
            printf("%s %d %s Error getting all fscanf arguments!\n", __FILE__, __LINE__, __FUNCTION__);
        }

//        printf("numFeatures: %d\n",numFeatures);
//        printf("patchWidth: %d\n",patchWidth);
//        printf("patchHeight: %d\n",patchHeight);
//        printf("poolWidth: %d\n",poolWidth);
//        printf("poolHeight: %d\n",poolHeight);
//        printf("numRows: %d\n",numRows);
//        printf("numCols: %d\n",numCols);
        layers[i]=Layer(numFeatures,patchWidth,patchHeight,poolWidth,poolHeight);
        thetas[i]=MatrixXf(numRows,numCols);
        for(int j=0;j<numRows*numCols;j++){
            float weight;
            if(fscanf(fp,"%f;",&weight) != 1) {
                printf("%s %d %s Error getting all fscanf arguments!\n", __FILE__, __LINE__, __FUNCTION__);
            }
            thetas[i](j)=weight;
        }
    }
//    printf("layers: \n");
//    for(int lay=0;lay<numLayers;lay++){
//        printf("layer %d: %d\n",lay,layerSizes[lay]);
//    }

    int rc = fscanf(fp,"true;");
    if(rc != 0) {
        printf("%s %d %s this should never happen\n", __FILE__, __LINE__, __FUNCTION__);
    }

    useNormalization=true;
    if(useNormalization){
        int meanValuesSize;
        if(fscanf(fp,"%d;",&meanValuesSize) != 1) {
            printf("%s %d %s Error getting all fscanf arguments!\n", __FILE__, __LINE__, __FUNCTION__);
        }

        meanValues.resize(meanValuesSize);
        for(int j=0;j<meanValuesSize;j++){
            float value;
            if(fscanf(fp,"%f;",&value) != 1) {
                printf("%s %d %s Error getting all fscanf arguments!\n", __FILE__, __LINE__, __FUNCTION__);
            }
            meanValues[j]=value;
        }
        int minmaxValuesSize;
        if(fscanf(fp,"%d;",&minmaxValuesSize) != 1) {
            printf("%s %d %s Error getting all fscanf arguments!\n", __FILE__, __LINE__, __FUNCTION__);
        }

        minmaxValues.resize(minmaxValuesSize);
        for(int j=0;j<minmaxValuesSize;j++){
            float value;
            if(fscanf(fp,"%f;",&value) != 1) {
                printf("%s %d %s Error getting all fscanf arguments!\n", __FILE__, __LINE__, __FUNCTION__);
            }
            minmaxValues[j]=value;
        }
    }

    fclose(fp);
}

}  // namespace htwk
