#include <chrono>
#include <cstring>
#include <cstdlib>
#include <iostream>

#include <Eigen/Dense>
#include <ext_math.h>
#include <convolutions.h>

using namespace ext_math;
using namespace Eigen;
using namespace std;
using namespace std::chrono;
namespace htwk {
    PoolingResult Convolutions::maxPool(MatrixXf &inputs, int inputWidth, int inputHeight, int poolWidth, int poolHeight){
        // Input pixel coordinates are stored as rows in m.
        // Each column in m is for a different channel.
        // (Channels for input layer contain R/G/B-values if colour-image.)
        // (Channels for hidden layer contains the features of the previous layer.)
        int inputSize=inputHeight*inputWidth;
        int numExamples=inputs.rows()/inputSize;
        int numChannels=inputs.cols();
        int outputHeight=inputHeight%poolHeight==0?inputHeight/poolHeight:inputHeight/poolHeight+1;
        int outputWidth=inputWidth%poolWidth==0?inputWidth/poolWidth:inputWidth/poolWidth+1;
        int outputSize=outputHeight*outputWidth;
        MatrixXf outputs=MatrixXf(numExamples*outputSize,numChannels);
        MatrixXf prePoolRowIndexes=MatrixXf(outputs.rows(),outputs.cols());
        for(int example=0;example<numExamples;example++){
            for(int outputY=0;outputY<outputHeight;outputY++){
                for(int outputX=0;outputX<outputWidth;outputX++){
                    for(int channel=0;channel<numChannels;channel++){
                        float maxValue=-std::numeric_limits<float>::infinity();
                        int maxInputsRowIndex=0;
                        for(int inputY=outputY*poolHeight;inputY<outputY*poolHeight+poolHeight&&inputY<inputHeight;inputY++){
                            for(int inputX=outputX*poolWidth;inputX<outputX*poolWidth+poolWidth&&inputX<inputWidth;inputX++){
                                int inputsRowIndex=example*inputSize+inputY*inputWidth+inputX;
                                float value=inputs(inputsRowIndex,channel);
                                if(value>maxValue){
                                    maxValue=value;
                                    maxInputsRowIndex=inputsRowIndex;
                                }
                            }
                        }
                        int outputsRowIndex=example*outputSize+outputY*outputWidth+outputX;
                        outputs(outputsRowIndex,channel)=maxValue;
                        prePoolRowIndexes(outputsRowIndex,channel)=maxInputsRowIndex;
                    }
                }
            }
        }
        return PoolingResult(outputs,prePoolRowIndexes);
    }

    MatrixXf Convolutions::generatePatchesFromInputLayer(MatrixXf &inputs, int inputWidth, int inputHeight, int patchWidth, int patchHeight){
        // Input data has one row per example.
        // Input data has one column per pixel.
        //      Assumes pixel-numbers are generated row-wise.
        //      (i.e. first all columns of row 0, then all columns of row 1 etc.)
        //      This is how the image-data is represented in the Kaggle Digits competition. (http://www.kaggle.com/c/digit-recognizer/data)
        // Output data have one row per example/patch
        // Output data have one column per patchPixel.

        int numChannels=inputs.cols()/(inputWidth*inputHeight);
        int numPatchesPerExample=(inputWidth-patchWidth+1)*(inputHeight-patchHeight+1);
        int numExamples=inputs.rows();
        MatrixXf output=MatrixXf(numExamples*numPatchesPerExample,numChannels*patchWidth*patchHeight);
        for(int example=0;example<numExamples;example++){
            int patchNum=0;
            for(int inputStartY=0;inputStartY<inputHeight-patchHeight+1;inputStartY++){
                for(int inputStartX=0;inputStartX<inputWidth-patchWidth+1;inputStartX++){
                    for(int channel=0;channel<numChannels;channel++){
                        for(int patchPixelY=0;patchPixelY<patchHeight;patchPixelY++){
                            for(int patchPixelX=0;patchPixelX<patchWidth;patchPixelX++){
                                int inputY=inputStartY+patchPixelY;
                                int inputX=inputStartX+patchPixelX;
                                float value=inputs(example,channel*inputHeight*inputWidth+inputY*inputWidth+inputX);
                                output(example*numPatchesPerExample+patchNum,channel*patchHeight*patchWidth+patchPixelY*patchWidth+patchPixelX)=value;
                            }
                        }
                    }
                    patchNum++;
                }
            }
        }
//        printf("GInputRows: %d\n",output.rows());
//        printf("GInputCols: %d\n",output.cols());
        return output;
    }

    MatrixXf Convolutions::generatePatchesFromHiddenLayer(MatrixXf &inputs, int inputWidth, int inputHeight, int patchWidth, int patchHeight){
        // Input data has one row per example/pixel
        // Input data has one column per channel.

        // Input data is one row per example and one column per pixel.
        //      Assumes pixel-numbers are generated row-wise.
        //      (i.e. first all columns of row 0, then all columns of row 1 etc.)
        //      This is how the image-data is represented in the Kaggle Digits competition. (http://www.kaggle.com/c/digit-recognizer/data)
        // Output data have one row per example/patch
        // Output data have one column per channel/patchPixel

        int numPatchesPerExample=(inputWidth-patchWidth+1)*(inputHeight-patchHeight+1);
        int inputSize=inputHeight*inputWidth;
        int numExamples=inputs.rows()/inputSize;
        int numChannels=inputs.cols();
        int patchSize=patchHeight*patchWidth;
        MatrixXf output=MatrixXf(numExamples*numPatchesPerExample,numChannels*patchSize);
        for(int example=0;example<numExamples;example++){
            int patchNum=0;
            for(int inputStartY=0;inputStartY<inputHeight-patchHeight+1;inputStartY++){
                for(int inputStartX=0;inputStartX<inputWidth-patchWidth+1;inputStartX++){
                    for(int channel=0;channel<numChannels;channel++){
                        for(int patchPixelY=0;patchPixelY<patchHeight;patchPixelY++){
                            for(int patchPixelX=0;patchPixelX<patchWidth;patchPixelX++){
                                int inputY=inputStartY+patchPixelY;
                                int inputX=inputStartX+patchPixelX;
                                double value=inputs(example*inputSize+inputY*inputWidth+inputX,channel);
                                output(example*numPatchesPerExample+patchNum,channel*patchSize+patchPixelY*patchWidth+patchPixelX)=value;
                            }
                        }
                    }
                    patchNum++;
                }
            }
        }
//        printf("GHiddenRows: %d\n",output.rows());
//        printf("GHiddenCols: %d\n",output.cols());
        return output;
    }

    MatrixXf Convolutions::movePatchesToColumns(MatrixXf &inputs, int numExamples, int numFeatureMaps, int numPatches){
        MatrixXf output=MatrixXf(numExamples,numFeatureMaps*numPatches);
        for(int example=0;example<numExamples;example++){
            for(int featureMap=0;featureMap<numFeatureMaps;featureMap++){
                for(int patch=0;patch<numPatches;patch++){
                    float value=inputs(example*numPatches+patch,featureMap);
                    output(example,featureMap*numPatches+patch)=value;
                }
            }
        }
        return output;
    }

}
