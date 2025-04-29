/* ----------------------------------------------------------------------------
 * Copyright (c) 2025, Harbin Institute of Technology.
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   robot_interface.cpp
 *  @author Jun Li (junli@hit.edu.cn)
 *  @brief  Source file for hardware interface of Go2 robot
 *  @date   April 28, 2025
 **/

#include "go2_hw/robot_interface.hpp"

#include <commutils/yaml/yaml_cpp_fwd.hpp>


namespace go2_hw {

RobotInterface::RobotInterface(const std::string& root_dir, 
                               const std::string& cfg_file, 
                               const std::string& hw_vars)
    : channel_factory_(unitree::robot::ChannelFactory::Instance())
{
    /* read config parameters from yaml file*/
    try {
        YAML::Node robot_cfg = YAML::LoadFile(root_dir + cfg_file.c_str());
        YAML::Node robot_vars = robot_cfg[hw_vars.c_str()];

        YAML::readParameter(robot_vars, "network_interface", network_interface_);

        YAML::readParameter(robot_vars, "timestep", timestep_);
        YAML::readParameter(robot_vars, "torque_factor", torque_factor_);

        YAML::readParameter(
            robot_vars, "homing_configuration", homing_configuration_);

        YAML::readParameter(
            robot_vars, "cont_force_calibr_offset", cont_force_calibr_offset_);
        YAML::readParameter(
            robot_vars, "cont_force_calibr_factor", cont_force_calibr_factor_);
    }
    catch (std::runtime_error& e) {
        std::cout << "Error reading parameter [" << e.what() << "]" << std::endl;
    }

    channel_factory_->Init(0, network_interface_);

    /*initialize low command*/
    low_cmd_.head()[0] = 0xFE;
    low_cmd_.head()[1] = 0xEF;
    low_cmd_.level_flag() = 0xFF;
    low_cmd_.gpio() = 0;

    for(int i = 0; i < kNumJoint; i++) {
        low_cmd_.motor_cmd()[i].mode() = (0x01);  // motor switch to servo mode
        low_cmd_.motor_cmd()[i].q() = (PosStopF);
        low_cmd_.motor_cmd()[i].kp() = (0);
        low_cmd_.motor_cmd()[i].dq() = (VelStopF);
        low_cmd_.motor_cmd()[i].kd() = (0);
        low_cmd_.motor_cmd()[i].tau() = (0);
    }

    /*create publisher*/
    lowcmd_publisher_.reset(
        new unitree::robot::ChannelPublisher<
            unitree_go::msg::dds_::LowCmd_>(TOPIC_LOWCMD));
    lowcmd_publisher_->InitChannel();

    /*create subscriber*/
    lowstate_subscriber_.reset(
        new unitree::robot::ChannelSubscriber<
            unitree_go::msg::dds_::LowState_>(TOPIC_LOWSTATE));
    lowstate_subscriber_->InitChannel();
}

}  // namespace go2_hw