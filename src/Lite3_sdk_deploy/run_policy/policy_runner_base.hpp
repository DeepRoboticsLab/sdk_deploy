/**
 * @file policy_runner_base.hpp
 * @brief policy runner base class
 * @author DeepRobotics
 * @version 1.0
 * @date 2025-11-07
 * 
 * @copyright Copyright (c) 2025  DeepRobotics
 * 
 */

 #pragma once

 #include "common_types.h"
 #include "basic_function.hpp"
 #include "onnxruntime_cxx_api.h"
 
 using namespace types;
 
 class PolicyRunnerBase{
 public:
     PolicyRunnerBase(std::string policy_name):policy_name_(policy_name){
         vel_delta_const_ << 0.3, 0.2, 0.3;
         cmd_vel_input_.setZero();
     }
     virtual ~PolicyRunnerBase(){}
     /**
      * @brief use to display policy detail
      */
     virtual void DisplayPolicyInfo(){
         
     }
 
     /**
      * @brief Get the robot action object by run your policy
      * @return RobotAction 
      */
     virtual RobotAction getRobotAction(const RobotBasicState&, const UserCommand&) = 0;
 
     /**
      * @brief execute function when first entering policy runner
      */
     virtual void OnEnter(const RobotBasicState&) = 0;
 
     /**
      * @brief Set the decimation
      * @param  d decimation
      */
     virtual void SetDecimation(int d){
         decimation_ = d;
     }
 
     const std::string policy_name_;
     int decimation_;
     int run_cnt_;
     Vec3f vel_delta_const_, cmd_vel_input_;
 };
 
 