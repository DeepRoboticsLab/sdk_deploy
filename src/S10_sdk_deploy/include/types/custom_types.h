#pragma once

#include "common_types.h"

namespace types{
    enum RobotName{
        S10 = 11,
    };

    enum RobotType{
        QuadrupedWheel = 1,
    };

    enum RobotMotionState{
        WaitingForStand = 0,
        StandingUp      = 1,
        JointDamping    = 2,
        LieDown         = 4,
        RLControlMode   = 6,
    };

    enum StateName{
        kInvalid      = -1,
        kIdle         = 0,
        kStandUp      = 1,
        kJointDamping = 2,
        kLieDown      = 4,
        kRLControl    = 6,
    };

    enum RemoteCommandType{
        kKeyBoard = 0,
        kGamepad,
        kDDS,
    };
    
    enum KeyCode {
        L1,
        L2,
        R1,
        R2,
        UNKNOWN
    };

    inline std::string GetAbsPath(){
        char buffer[PATH_MAX];
        if(getcwd(buffer, sizeof(buffer)) != NULL){
            return std::string(buffer);
        }
        return "";
    }
};
