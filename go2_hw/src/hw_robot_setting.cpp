/* ----------------------------------------------------------------------------
 * Copyright (c) 2025, Harbin Institute of Technology.
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   hw_robot_settings.cpp
 *  @author Jun Li (junli@hit.edu.cn)
 *  @brief  Source file for HwRobotSetting class
 *  @date   April 19, 2025
 **/

#include "go2_hw/hw_robot_setting.hpp"

#include <iostream>

#include "commutils/yaml/yaml_cpp_fwd.hpp"

namespace go2_hw {

void HwRobotSetting::initialize(const std::string& root_dir, 
                                const std::string& cfg_file, 
                                const std::string& robot_vars_yaml)
{
    try {
        YAML::Node robot_cfg = YAML::LoadFile(root_dir + cfg_file.c_str());
        YAML::Node robot_vars = robot_cfg[robot_vars_yaml.c_str()];

        YAML::readParameter(robot_vars, "network_interface", network_interface_);

        YAML::readParameter(robot_vars, "timestep", timestep_);
        YAML::readParameter(robot_vars, "torque_factor", torque_factor_);

        YAML::readParameter(robot_vars, "homing_pos", homing_pos_);

        YAML::readParameter(robot_vars, "cont_force_calibr_offset", cont_force_calibr_offset_);
        YAML::readParameter(robot_vars, "cont_force_calibr_factor", cont_force_calibr_factor_);
    }
    catch (std::runtime_error& e) {
        std::cout << "[go2_hw/HwRobotSetting::initialize]: Error reading parameter ["
                  << e.what() << "]" << std::endl;
    }
}

double HwRobotSetting::get(RobotDoubleParam param) const
{
    switch (param) {
        case RobotDoubleParam_Timestep:
            return timestep_;
            break;
        case RobotDoubleParam_TorqueFactor:
            return torque_factor_;
            break;
        default:
            throw std::runtime_error("[go2_hw/HwRobotSetting::initialize]: RobotDoubleParam invalid");
            break;
    }
}

const HwRobotSetting::VectorXd& HwRobotSetting::get(RobotVectorParam param) const
{
    switch (param) {
        case RobotVectorParam_HomingPos:
            return homing_pos_;
            break;
        case RobotVectorParam_ContForceCalibrOffset:
            return cont_force_calibr_offset_;
            break;
        case RobotVectorParam_ContForceCalibrFactor:
            return cont_force_calibr_factor_;
            break;
        default:
        throw std::runtime_error("[go2_hw/HwRobotSetting::initialize]: RobotVectorParam invalid");
        break;
    }
}

}  // namespace go2_hw