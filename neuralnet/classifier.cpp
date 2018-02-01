#include "classifier.h"

#include <cmath>

using namespace ext_math;
using namespace std;

namespace htwk {

void Classifier::extractWeights(vec2df &arr, FILE *fp, int sizeX, int sizeY){
    for(int i=0;i<sizeX;i++){
        for(int j=0;j<sizeY;j++){
            float weight;
            int ret=fscanf(fp,"%f;",&weight);
            if(ret!=1){
                printf("reading of weight file failed!\n");
                exit(-1);
            }
            arr[i][j]=weight;
        }
    }
}

vec2df Classifier::proceed(const vec2df &pixels){
    vec2df a2=mmul(W1,(pixels-normMid)/normVar);
    a2=a2+b1;
    this->tanh(a2);
    vec2df a3=mmul(W2,a2);
    a3=a3+b2;
    this->tanh(a3);
    return a3;
}

void Classifier::tanh(vec2df &a){
    int aRows=a.size();
    int aCols=a[0].size();
    for(int r=0;r<aRows;r++){
        for(int c=0;c<aCols;c++){
            a[r][c]=1.f/(1+expf(-a[r][c]));
        }
    }
}

vec2df Classifier::mmul(const vec2df &a, const vec2df &b){
    int aRows=a.size();
    int aCols=a[0].size();
    int bCols=b[0].size();
    vec2df result=createVec2df(aRows,bCols);
    for(int r=0;r<aRows;r++){
        for(int c=0;c<bCols;c++){
            result[r][c]=0;
            for(int i=0;i<aCols;i++){
                result[r][c]+=a[r][i]*b[i][c];
            }
        }
    }
    return result;
}

Classifier::Classifier(const string& weightFile){
    FILE *fp=fopen(weightFile.c_str(),"r");
    if (fp == nullptr) {
        printf("Could not find %s.\n", weightFile.c_str());
        exit(-1);
    }
    int ret=fscanf(fp,"%d;%d;%d;%d;",&version,&inputPixels,&hiddenSize,&outputSize);
    if(ret!=4){
        printf("reading of robot angle estimation file failed!\n");
        exit(-1);
    }
    if(version!=2){
        printf("wrong version of classifier weight file!\n");
        exit(-1);
    }
    normMid=createVec2df(inputPixels,1);
    normVar=createVec2df(inputPixels,1);
    W1=createVec2df(hiddenSize,inputPixels);
    W2=createVec2df(outputSize,hiddenSize);
    b1=createVec2df(hiddenSize,1);
    b2=createVec2df(outputSize,1);

    extractWeights(normMid,fp,inputPixels,1);
    extractWeights(normVar,fp,inputPixels,1);
    extractWeights(W1,fp,hiddenSize,inputPixels);
    extractWeights(W2,fp,outputSize,hiddenSize);
    extractWeights(b1,fp,hiddenSize,1);
    extractWeights(b2,fp,outputSize,1);

    fclose(fp);
}

}  // namespace htwk
