#include "control_parameters.h"

void ControlParameters::GenerateM20Parameters(){
    body_len_x_ = 0.3095*2;
    body_len_y_ = 0.065*2;
    hip_len_ = 0.104;
    thigh_len_ = 0.25;
    shank_len_ = 0.25;
    pre_height_ = 0.12;
    stand_height_ = 0.48;
    swing_leg_kp_ << 200., 200., 200.;
    swing_leg_kd_ << 4., 4., 4.;

    fl_joint_lower_ << -0.4363, -2.443, -2.758;
    fl_joint_upper_ << 0.6109, 2.443, 2.758;
    joint_vel_limit_ << 22.4, 22.4, 22.4;
    torque_limit_ << 76.4, 76.4, 76.4;
}