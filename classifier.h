#ifndef __CLASSIFIER_H__
#define __CLASSIFIER_H__

#include <cstdio>
#include <string>

#include <ext_math.h>

namespace htwk {

class Classifier{
private:
    int version;
    int inputPixels;
    int hiddenSize;
    int outputSize;
    ext_math::vec2df W0;
    ext_math::vec2df W1;
    ext_math::vec2df W2;
    ext_math::vec2df b1;
    ext_math::vec2df b2;
    ext_math::vec2df normMid;
    ext_math::vec2df normVar;

    static void extractWeights(ext_math::vec2df &arr, FILE *fp, int sizeX, int sizeY);
    static void tanh(ext_math::vec2df &a);
    static ext_math::vec2df mmul(const ext_math::vec2df &a, const ext_math::vec2df &b);

public:
    Classifier(std::string weightFile);
    ext_math::vec2df proceed(const ext_math::vec2df &pixels);
};

}  // namespace htwk

#endif  // __CLASSIFIER_H__
