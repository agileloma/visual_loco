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

#include <unistd.h>
#include <cmath>

#include <commutils/yaml/yaml_cpp_fwd.hpp>

#define USECS_PER_SEC  1000000

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

    joint_pos_min_ << kHipMinPos, kThighMinPos, kCalfMinPos, 
                      kHipMinPos, kThighMinPos, kCalfMinPos,
                      kHipMinPos, kThighMinPos, kCalfMinPos,
                      kHipMinPos, kThighMinPos, kCalfMinPos;

    joint_pos_max_ << kHipMaxPos, kThighMaxPos, kCalfMaxPos,
                      kHipMaxPos, kThighMaxPos, kCalfMaxPos,
                      kHipMaxPos, kThighMaxPos, kCalfMaxPos,
                      kHipMaxPos, kThighMaxPos, kCalfMaxPos;

    q_hom_ = homing_configuration_;

    joint_pos_.setZero();
    joint_vel_.setZero();
    joint_trq_.setZero();

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
    lowstate_subscriber_->InitChannel(
        std::bind(&RobotInterface::recvLowState, this, std::placeholders::_1), 1);

    /*loop publishing*/
    lowCmd_send_thread_ = unitree::common::CreateRecurrentThreadEx(
        "writebasiccmd", UT_CPU_ID_NONE, 2000, &RobotInterface::sendLowCmd, this);
}

uint32_t RobotInterface::crc32_core(uint32_t* ptr, uint32_t len)
{
    unsigned int xbit = 0;
    unsigned int data = 0;
    unsigned int CRC32 = 0xFFFFFFFF;
    const unsigned int dwPolynomial = 0x04c11db7;

    for (unsigned int i = 0; i < len; i++)
    {
        xbit = 1 << 31;
        data = ptr[i];
        for (unsigned int bits = 0; bits < 32; bits++)
        {
            if (CRC32 & 0x80000000) {
                CRC32 <<= 1;
                CRC32 ^= dwPolynomial;
            }
            else {
                CRC32 <<= 1;
            }

            if (data & xbit)
                CRC32 ^= dwPolynomial;
            xbit >>= 1;
        }
    }

    return CRC32;
}

void RobotInterface::sendLowCmd() 
{
    low_cmd_.crc() = crc32_core(
        (uint32_t *)&low_cmd_, (sizeof(unitree_go::msg::dds_::LowCmd_)>>2)-1);

    lowcmd_publisher_->Write(low_cmd_);
}

void RobotInterface::recvLowState(const void* message)
{
    low_state_ = *(unitree_go::msg::dds_::LowState_*)message;
}

void RobotInterface::homing()
{
    bool is_valid = false;
    is_valid = (q_hom_.array() >= joint_pos_min_.array()).all() && 
                  (q_hom_.array() <= joint_pos_max_.array()).all();

    assert(is_valid && "homing positions are not valid!");

    int steps = static_cast<int>(hom_dur_/timestep_);
    int counter = 0;
    double rate = 0.0;

    while (counter <= steps) {
        counter++;

        // first, get record initial configuration
        if (counter > 0 && counter < 10) {
            q_init_ = getJointPositions();
            q_des_ = q_init_;

            low_cmd_.motor_cmd()[0].kp() = 5.0;
            low_cmd_.motor_cmd()[0].kd() = 1.0;
            low_cmd_.motor_cmd()[1].kp() = 5.0;
            low_cmd_.motor_cmd()[1].kd() = 1.0;
            low_cmd_.motor_cmd()[2].kp() = 5.0;
            low_cmd_.motor_cmd()[2].kd() = 1.0;

            low_cmd_.motor_cmd()[3].kp() = 5.0;
            low_cmd_.motor_cmd()[3].kd() = 1.0;
            low_cmd_.motor_cmd()[4].kp() = 5.0;
            low_cmd_.motor_cmd()[4].kd() = 1.0;
            low_cmd_.motor_cmd()[5].kp() = 5.0;
            low_cmd_.motor_cmd()[5].kd() = 1.0;

            low_cmd_.motor_cmd()[6].kp() = 5.0;
            low_cmd_.motor_cmd()[6].kd() = 1.0;
            low_cmd_.motor_cmd()[7].kp() = 5.0;
            low_cmd_.motor_cmd()[7].kd() = 1.0;
            low_cmd_.motor_cmd()[8].kp() = 5.0;
            low_cmd_.motor_cmd()[8].kd() = 1.0;

            low_cmd_.motor_cmd()[9].kp() = 5.0;
            low_cmd_.motor_cmd()[9].kd() = 1.0;
            low_cmd_.motor_cmd()[10].kp() = 5.0;
            low_cmd_.motor_cmd()[10].kd() = 1.0;
            low_cmd_.motor_cmd()[11].kp() = 5.0;
            low_cmd_.motor_cmd()[11].kd() = 1.0;
        }
        else {
            rate = static_cast<double>(counter) / static_cast<double>(steps);
            double scale = 0.5 * (1.0 - std::cos(M_PI * rate));
            q_des_ = q_init_ + scale * (q_hom_ - q_init_);

            low_cmd_.motor_cmd()[0].kp() = 25.0;
            low_cmd_.motor_cmd()[0].kd() = 0.5;
            low_cmd_.motor_cmd()[1].kp() = 25.0;
            low_cmd_.motor_cmd()[1].kd() = 0.5;
            low_cmd_.motor_cmd()[2].kp() = 25.0;
            low_cmd_.motor_cmd()[2].kd() = 0.5;

            low_cmd_.motor_cmd()[3].kp() = 25.0;
            low_cmd_.motor_cmd()[3].kd() = 0.0;
            low_cmd_.motor_cmd()[4].kp() = 25.0;
            low_cmd_.motor_cmd()[4].kd() = 0.0;
            low_cmd_.motor_cmd()[5].kp() = 25.0;
            low_cmd_.motor_cmd()[5].kd() = 0.0;

            low_cmd_.motor_cmd()[6].kp() = 25.0;
            low_cmd_.motor_cmd()[6].kd() = 0.5;
            low_cmd_.motor_cmd()[7].kp() = 25.0;
            low_cmd_.motor_cmd()[7].kd() = 0.5;
            low_cmd_.motor_cmd()[8].kp() = 25.0;
            low_cmd_.motor_cmd()[8].kd() = 0.5;

            low_cmd_.motor_cmd()[9].kp() = 25.0;
            low_cmd_.motor_cmd()[9].kd() = 0.5;
            low_cmd_.motor_cmd()[10].kp() = 25.0;
            low_cmd_.motor_cmd()[10].kd() = 0.5;
            low_cmd_.motor_cmd()[11].kp() = 25.0;
            low_cmd_.motor_cmd()[11].kd() = 0.5;
        }

        low_cmd_.motor_cmd()[0].q() = q_des_[0];
        low_cmd_.motor_cmd()[1].q() = q_des_[1];
        low_cmd_.motor_cmd()[2].q() = q_des_[2];
        low_cmd_.motor_cmd()[3].q() = q_des_[3];
        low_cmd_.motor_cmd()[4].q() = q_des_[4];
        low_cmd_.motor_cmd()[5].q() = q_des_[5];
        low_cmd_.motor_cmd()[6].q() = q_des_[6];
        low_cmd_.motor_cmd()[7].q() = q_des_[7];
        low_cmd_.motor_cmd()[8].q() = q_des_[8];
        low_cmd_.motor_cmd()[9].q() = q_des_[9];
        low_cmd_.motor_cmd()[10].q() = q_des_[10];
        low_cmd_.motor_cmd()[11].q() = q_des_[11];

        usleep(timestep_ * USECS_PER_SEC);
    }

    const JointStateVector& q_end = getJointPositions();
    assert(q_end.isApprox(q_hom_, 1e-1) && "Homing failed!");

    std::cout << "Finish moving to homing position.\n" << std::endl;
}

const RobotInterface::JointStateVector& RobotInterface::getJointPositions()
{
    joint_pos_ << low_state_.motor_state()[0].q(),   // FR Hip
                  low_state_.motor_state()[1].q(),   // FR Thigh
                  low_state_.motor_state()[2].q(),   // FR Calf
                  low_state_.motor_state()[3].q(),   // FL Hip
                  low_state_.motor_state()[4].q(),   // FL Thigh
                  low_state_.motor_state()[5].q(),   // FL Calf
                  low_state_.motor_state()[6].q(),   // RR Hip
                  low_state_.motor_state()[7].q(),   // RR Thigh
                  low_state_.motor_state()[8].q(),   // RR Calf
                  low_state_.motor_state()[9].q(),   // RR Hip
                  low_state_.motor_state()[10].q(),  // RR Thigh
                  low_state_.motor_state()[11].q();  // RR Calf
    return joint_pos_;
}

}  // namespace go2_hw