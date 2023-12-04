#ifndef HTWKPNGMETADATA_H
#define HTWKPNGMETADATA_H

#include <optional>

#include <cam_constants.h>
#include <imu.h>

namespace htwk {
namespace image {

struct PngMetadata {
    float pitch;
    float roll;
    float legHeight;

    YPR bodyAngles;
    YawPitch headAngles;

    PitchRoll bodyOffset;
    PitchRoll headOffset;

    CamID camId;

    std::optional<PitchRoll> ellipseAngles;
    std::optional<std::string> version = std::nullopt;
};

}  // namespace image
}  // namespace htwk

#endif  // HTWKPNGMETADATA_H
