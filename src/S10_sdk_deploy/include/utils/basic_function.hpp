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
    const float cr = std::cos(rpy(0)), sr = std::sin(rpy(0));
    const float cp = std::cos(rpy(1)), sp = std::sin(rpy(1));
    const float cy = std::cos(rpy(2)), sy = std::sin(rpy(2));
    Mat3f R;
    R(0,0) = cy * cp;  R(0,1) = cy * sp * sr - sy * cr;  R(0,2) = cy * sp * cr + sy * sr;
    R(1,0) = sy * cp;  R(1,1) = sy * sp * sr + cy * cr;  R(1,2) = sy * sp * cr - cy * sr;
    R(2,0) = -sp;      R(2,1) = cp * sr;                 R(2,2) = cp * cr;
    return R;
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

inline float GetCubicSplinePos(float x0, float v0, float xf, float vf, float t, float T){
        if(t >= T) return xf;
        float a, b, c, d;
        d = x0;
        c = v0;
        a = (vf*T - 2*xf + v0*T + 2*x0) / pow(T, 3);
        b = (3*xf - vf*T - 2*v0*T - 3*x0) / pow(T, 2);
        return a*pow(t, 3)+b*pow(t, 2)+c*t+d;
}

inline float GetCubicSplineVel(float x0, float v0, float xf, float vf, float t, float T){
    if(t >= T) return 0;
    float a, b, c;
    c = v0;
    a = (vf*T - 2*xf + v0*T + 2*x0) / pow(T, 3);
    b = (3*xf - vf*T - 2*v0*T - 3*x0) / pow(T, 2);
    return 3.*a*pow(t, 2) + 2.*b*t + c;
}
