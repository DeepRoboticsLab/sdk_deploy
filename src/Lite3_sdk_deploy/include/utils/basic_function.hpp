/**
 * @file basic_function.hpp
 * @brief basic function
 * @author DeepRobotics
 * @version 1.0
 * @date 2025-11-07
 * 
 * @copyright Copyright (c) 2025  DeepRobotics
 * 
 */
#pragma once
#include "common_types.h"
#include "custom_types.h"

using namespace types;

inline float Deg2Rad(float deg){
    return deg /180.f * M_PI;
}

inline float Rad2Deg(float rad){
    return rad / M_PI * 180.;
}

inline float LimitNumber(float data, float limit){
    limit = fabs(limit);
    if(data > limit) data = limit;
    else if(data < -limit) data = -limit;
    return data;
}

inline float LimitNumber(float data, float low, float high){
    if(low > high){
        std::cerr << "error limit range" << std::endl;
        return data;
    }
    if(data > high) return high;
    if(data < low) return low;
    return data;
}

inline Mat3f RpyToRm(const Vec3f &rpy){
    Eigen::AngleAxisf yawAngle(rpy(2), Vec3f::UnitZ());
    Eigen::AngleAxisf pitchAngle(rpy(1), Vec3f::UnitY());
    Eigen::AngleAxisf rollAngle(rpy(0), Vec3f::UnitX());
    Eigen::Quaternion<float> q = yawAngle*pitchAngle*rollAngle;
    return q.matrix();
}

inline float NormalizeAngle(float angle) {
    float result = std::fmod(angle, 2 * M_PI);
    if (result < 0) {
        result += 2 * M_PI;
    }
    if (result > M_PI) {
        result -= 2 * M_PI;
    }
    return result;
}

inline float Sign(const float &i){
    if (i > 0) {
        return 1.;
    } else if (i == 0) {
        return 0.;
    } else {
        return -1.;
    }
}

inline  double GetTimestampMs(){
    static timespec startup_timestamp;
    timespec now_timestamp;
    if (startup_timestamp.tv_sec + startup_timestamp.tv_nsec == 0) {
        clock_gettime(CLOCK_MONOTONIC,&startup_timestamp);
    }
    clock_gettime(CLOCK_MONOTONIC,&now_timestamp);
    return (now_timestamp.tv_sec-startup_timestamp.tv_sec)*1e3 + (now_timestamp.tv_nsec-startup_timestamp.tv_nsec)/1e6;
}
