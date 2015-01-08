#ifndef __FIELD_DETECTOR_H__
#define __FIELD_DETECTOR_H__

#include <random>

#include <field_color_detector.h>
#include <region_classifier.h>

namespace htwk {

struct Region {
  int xLeft, xRight, yTop, yBottom;
  int size;
  bool isField;
};

class FieldDetector {
 private:
    int width;
    int height;
  std::mt19937 rng;
  std::uniform_real_distribution<> dist{0,1};

  int *fieldBorderFull;

  int *lutCb;
  int *lutCr;

  FieldDetector();
  FieldDetector(const FieldDetector &cpy);
  FieldDetector operator=(FieldDetector &f);

 public:
  FieldDetector(int width, int height, int *lutCb, int *lutCr) __attribute__((nonnull));
  ~FieldDetector();
  void proceed(const uint8_t *const img, const FieldColorDetector *const field,
               const RegionClassifier *const regionClassifier)
      __attribute__((nonnull));
  const int *getConvexFieldBorder() const;
};

}  // namespace htwk
#endif  // __FIELD_DETECTOR_H__
