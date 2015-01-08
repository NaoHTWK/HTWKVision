#ifndef GAMESTATETYPES_H
#define GAMESTATETYPES_H

#include <cstdint>

namespace htwk {

enum team_color_t{
    ROBOT_TEAM_NONE,ROBOT_TEAM_BLUE,ROBOT_TEAM_RED
};

typedef uint8_t nao_teamcolor_t; // Type is given by RoboCupGameControlData.h which is given to us by the SPL organizers.
typedef int16_t jerseynumber_t; // Type is given by RoboCupGameControlData.h which is given to us by the SPL organizers. Valid Range is 1..WM_NUM_PLAYERS inclusive.
typedef int16_t teamnumber_t; // Type is given by RoboCupGameControlData.h which is given to us by the SPL organizers.
typedef int naohtwk_timestamp_t;//!<DCM-based timestamp, x*10ms from last naoqi start, 0 is the oldest value
#define NAO_HTWK_TIMESTAMPS_PER_SEC (100)

// Additional constants to the ones which are already present in RoboCupGameControlData.h .
#define NO_JERSEY_NUMBER 0
#define NO_TEAM_NUMBER   0
#define NO_COLOR         2
#define GOALIE_JERSEY_NUMBER ((jerseynumber_t)1)

}  // namespace htwk

#endif // GAMESTATETYPES_H
