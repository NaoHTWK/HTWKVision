#ifndef CAMPOSEUTILS_H
#define CAMPOSEUTILS_H

#include <htwkpngmetadata.h>
#include <localization_utils.h>

namespace htwk {
namespace image {

inline CamPose getCamPoseFromMetadata(const htwk::image::PngMetadata& md) {
    if (!md.version) {
        return CamPose::fromV5(PitchRoll(md.pitch, md.roll));
    } else if (*md.version == "2019.05") {
        auto pose = CamPose(md.legHeight, md.bodyAngles, md.headAngles, md.camId, md.bodyOffset, md.headOffset);

        if (md.ellipseAngles)
            pose.setEllipseAngles(*md.ellipseAngles);

        return pose;
//        printf("cam: %s\n", md.camId == CamID::UPPER ? "upper" : "lower");
//        printf("leg: %.3f\n", md.legHeight);
//        printf("head: %.3f %.3f\n", md.headAngles.yaw, md.headAngles.pitch);
//        printf("body: %.3f %.3f %.3f\n", md.bodyAngles.yaw, md.bodyAngles.pitch, md.bodyAngles.roll);
//        printf("body offset: %.3f %.3f\n", md.bodyOffset.pitch, md.bodyOffset.roll);
//        printf("head offset: %.3f %.3f\n", md.headOffset.pitch, md.headOffset.roll);

//        if (md.ellipseAngles)
//            printf("ellipse: %.3f %.3f\n", md.ellipseAngles->pitch, md.ellipseAngles->roll);
    }

    return CamPose();
}

}
}

#endif  // CAMPOSEUTILS_H
