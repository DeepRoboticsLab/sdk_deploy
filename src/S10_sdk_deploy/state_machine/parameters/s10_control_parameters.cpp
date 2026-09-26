#include "control_parameters.h"

void ControlParameters::GenerateS10Parameters(){
    body_len_x_ = 0.225 * 2;
    body_len_y_ = 0.05254 * 2;
    hip_len_ = 0.05;
    thigh_len_ = 0.18;
    shank_len_ = 0.18;
    pre_height_ = 0.08;
    stand_height_ = 0.28;
    swing_leg_kp_ << 120., 120., 120.;
    swing_leg_kd_ << 2., 2., 2.;

    fl_joint_lower_ << -0.6109, -2.5307, -2.7227;
    fl_joint_upper_ << 0.6109, 2.5307, 2.7227;
    joint_vel_limit_ << 25.76, 25.76, 25.76;
    torque_limit_ << 50, 50, 50;
}