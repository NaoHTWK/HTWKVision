#ifndef PENALTYSPOT_DETECTOR_H
#define PENALTYSPOT_DETECTOR_H

#include <cstdint>
#include <memory>
#include <optional>
#include <vector>

#include <base_detector.h>
#include <object_hypothesis.h>

namespace htwk {

class PenaltySpotDetector {
public:
    PenaltySpotDetector() = default;
    PenaltySpotDetector(const PenaltySpotDetector&) = delete;
    PenaltySpotDetector(const PenaltySpotDetector&&) = delete;
    PenaltySpotDetector& operator=(const PenaltySpotDetector&) = delete;
    PenaltySpotDetector& operator=(const PenaltySpotDetector&&) = delete;
    virtual ~PenaltySpotDetector() = default;

    // This can be empty when there was no ball in the image.
    virtual const std::optional<ObjectHypothesis> getPenaltySpot() const = 0;

    // This is used by the machine learning tools. This must not be used in the firmware!
    virtual const std::vector<ObjectHypothesis>& getRatedPenaltySpotHypotheses() const = 0;
};

template<class T>
class PenaltySpotDetectorAdapter : public PenaltySpotDetector {
public:
    explicit PenaltySpotDetectorAdapter(std::shared_ptr<T> objDetector) : objDetector(objDetector) {}
    ~PenaltySpotDetectorAdapter() = default;

    const std::optional<ObjectHypothesis> getPenaltySpot() const override {
        return objDetector->getPenaltySpot();
    }

    const std::vector<ObjectHypothesis>& getRatedPenaltySpotHypotheses() const override {
        return objDetector->getRatedPenaltySpotHypotheses();
    }

private:
    std::shared_ptr<T> objDetector;
};


}  // namespace htwk

#endif  // PENALTYSPOT_DETECTOR_H
