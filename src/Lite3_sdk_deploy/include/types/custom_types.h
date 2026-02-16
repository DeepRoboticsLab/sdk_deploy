#pragma once

#include "common_types.h"

namespace types{
    enum RobotName{
        Lite3 = 12,
    };

    enum RobotType{
        Quadruped = 2,
    };

    enum RobotMotionState{
        WaitingForStand = 0,
        StandingUp      = 1,
        JointDamping    = 2,
        RLControlMode   = 6,
    };

    enum StateName{
        kInvalid      = -1,
        kIdle         = 0,
        kStandUp      = 1,
        kJointDamping = 2,
        kRLControl    = 6,
    };

    enum RemoteCommandType{
        kKeyBoard = 0,
        kRetroidGamepad = 1,
    };
    

    inline std::string GetAbsPath(){
        char buffer[PATH_MAX];
        if(getcwd(buffer, sizeof(buffer)) != NULL){
            return std::string(buffer);
        }
        return "";
    }
};
