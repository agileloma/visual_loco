/* ----------------------------------------------------------------------------
 * Copyright (c) 2025, Harbin Institute of Technology.
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   hw_robot_settings.hpp
 *  @author Jun Li (junli@hit.edu.cn)
 *  @brief  Header file for HwRobotSetting class
 *  @date   April 19, 2025
 **/

 #pragma once

 #include <string>
 
 #include <Eigen/Dense>
 
 #include "go2_hw/hw_robot_params.hpp"
 
 
 namespace go2_hw {
 
 class HwRobotSetting
 {
 public:
     EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 
     typedef Eigen::VectorXd VectorXd;
 
     HwRobotSetting() {}
     ~HwRobotSetting() {}
 
     void initialize(const std::string& root_dir, 
                     const std::string& cfg_file, 
                     const std::string& robot_vars_yaml="hw_robot_varibables");
 
     const std::string& get(RobotStringParam param) const;
     double get(RobotDoubleParam param) const;
     const VectorXd& get(RobotVectorParam param) const;
 
 private:
     std::string network_interface_{}; 
 
     double timestep_;
     double torque_factor_;
 
     VectorXd homing_pos_;
 
     VectorXd cont_force_calibr_offset_;
     VectorXd cont_force_calibr_factor_;
 };
 
 }  // namespace go2_hw