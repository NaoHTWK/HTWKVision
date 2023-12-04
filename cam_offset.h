#ifndef CAM_OFFSET_H_
#define CAM_OFFSET_H_

class CamOffset {
public:
    static void init();
    static float headPitch;
    static float headRoll;
    static float bodyPitch;
    static float bodyRoll;
    static float deltaPitch[30];
    static float deltaRoll[30];
    static int deltaIdx;
    static int lowerCamOffsetX;
    static int lowerCamOffsetY;
    static int lowerCamOffsetCnt;

    static void write();

    CamOffset() = delete;
    CamOffset(const CamOffset&) = delete;
    CamOffset(CamOffset&&) = delete;
    CamOffset& operator=(const CamOffset&) = delete;
    CamOffset& operator=(CamOffset&&) = delete;
    ~CamOffset() = delete;
};

#endif /* CAM_OFFSET_H_ */
