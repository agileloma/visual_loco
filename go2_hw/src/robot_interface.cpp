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

#include <cmath>

#include <commutils/yaml/yaml_cpp_fwd.hpp>

#define USECS_PER_SEC  1000000

namespace go2_hw {

namespace {

uint32_t crc32_core(uint32_t* ptr, uint32_t len)
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

}  // namespace

RobotInterface::RobotInterface(const std::string& root_dir, 
                               const std::string& cfg_file, 
                               const std::string& hw_vars)
{
//     /* read config parameters from yaml file*/
//     try {
//         YAML::Node robot_cfg = YAML::LoadFile(root_dir + cfg_file.c_str());
//         YAML::Node robot_vars = robot_cfg[hw_vars.c_str()];

//         YAML::readParameter(robot_vars, "timestep", timestep_);
//         // YAML::readParameter(robot_vars, "torque_factor", torque_factor_);

//         YAML::readParameter(robot_vars, "homing_posture", q_homing_);

//         YAML::readParameter(
//             robot_vars, "cont_force_calibr_offset", cont_force_calibr_offset_);
//         YAML::readParameter(
//             robot_vars, "cont_force_calibr_factor", cont_force_calibr_factor_);
//     }
//     catch (std::runtime_error& e) {
//         std::cout << "Error reading parameter [" << e.what() << "]" << std::endl;
//     }
// std::cout << "after YAML" << std::endl;
//     channel_factory_ = unitree::robot::ChannelFactory::Instance();
//     channel_factory_->Init(0);
// std::cout << "after channel_factory_->Init(0);" << std::endl;
//     joint_pos_min_ << kHipMinPos, kThighMinPos, kCalfMinPos, 
//                       kHipMinPos, kThighMinPos, kCalfMinPos,
//                       kHipMinPos, kThighMinPos, kCalfMinPos,
//                       kHipMinPos, kThighMinPos, kCalfMinPos;
//     joint_pos_max_ << kHipMaxPos, kThighMaxPos, kCalfMaxPos,
//                       kHipMaxPos, kThighMaxPos, kCalfMaxPos,
//                       kHipMaxPos, kThighMaxPos, kCalfMaxPos,
//                       kHipMaxPos, kThighMaxPos, kCalfMaxPos;

//     step_counter_ = 0;

//     /*initialize low command*/
//     low_cmd_.head()[0] = 0xFE;
//     low_cmd_.head()[1] = 0xEF;
//     low_cmd_.level_flag() = 0xFF;
//     low_cmd_.gpio() = 0;

//     for(int i = 0; i < kNumJoint; i++) {
//         low_cmd_.motor_cmd()[i].mode() = (0x01);  // motor switch to servo mode
//         low_cmd_.motor_cmd()[i].q() = (PosStopF);
//         low_cmd_.motor_cmd()[i].kp() = (0);
//         low_cmd_.motor_cmd()[i].dq() = (VelStopF);
//         low_cmd_.motor_cmd()[i].kd() = (0);
//         low_cmd_.motor_cmd()[i].tau() = (0);
//     }

//     /*create publisher*/
//     lowcmd_publisher_.reset(
//         new unitree::robot::ChannelPublisher<
//             unitree_go::msg::dds_::LowCmd_>(TOPIC_LOWCMD));
//     lowcmd_publisher_->InitChannel();

//     /*create subscriber*/
//     lowstate_subscriber_.reset(
//         new unitree::robot::ChannelSubscriber<
//             unitree_go::msg::dds_::LowState_>(TOPIC_LOWSTATE));
//     lowstate_subscriber_->InitChannel(
//         std::bind(&RobotInterface::recvLowState, this, std::placeholders::_1), 1);

//     /*loop publishing*/
//     lowCmd_send_thread_ = unitree::common::CreateRecurrentThreadEx(
//         "writebasiccmd", UT_CPU_ID_NONE, 2000, &RobotInterface::sendLowCmd, this);

//     /*Set MotionSwitcherClient*/
//     switcher_client_.SetTimeout(10.0f);

//     std::cout << "after switcher_client_.SetTimeout(10.0f)" << std::endl;
//     switcher_client_.Init();
//     /*Shut down motion control related service*/
//     while (queryMotionStatus()) {
//         std::cout << "Try to deactivate the motion control related service."
//                   << std::endl;
//         int32_t ret = switcher_client_.ReleaseMode();
//         if (ret == 0) {
//             std::cout << "ReleaseMode succeeded." << std::endl;
//         }
//         else {
//             std::cout << "ReleaseMode failed. Error code: " << ret << std::endl;
//         }
//         sleep(5);
//     }
// std::cout << "after MotionSwitcherClient" << std::endl;
}

// int RobotInterface::queryMotionStatus()
// {
//     std::string robot_form, motion_name;
//     int motion_status;
//     int32_t ret = switcher_client_.CheckMode(robot_form, motion_name);
//     if (ret == 0) {
//         std::cout << "CheckMode succeeded." << std::endl;
//     }
//     else {
//         std::cout << "CheckMode failed. Error code: " << ret << std::endl;
//     }

//     if (motion_name.empty()) {
//         std::cout << "The motion control related service is deactivated." 
//                   << std::endl;
//         motion_status = 0;
//     }
//     else {
//         std::string service_name = queryServiceName(robot_form, motion_name);
//         std::cout << "Service: " << service_name << " is activated." 
//                   << std::endl;
//         motion_status = 1;
//     }
//     return motion_status;
// }

// std::string RobotInterface::queryServiceName(std::string form, std::string name)
// {
//     if (form == "0") {
//         if (name == "normal")
//             return "sport_mode";
//         if (name == "ai")
//             return "ai_sport";
//         if (name == "advanced")
//             return "advanced_sport";
//     }
//     else {
//         if (name == "ai-w")
//             return "wheeled_sport(go2w)";
//         if (name == "normal-w")
//             return "wheeled_sport(b2w)";
//     }

//     return "";
// }

// void RobotInterface::sendLowCmd() 
// {
//     low_cmd_.crc() = crc32_core(
//         (uint32_t *)&low_cmd_, (sizeof(unitree_go::msg::dds_::LowCmd_)>>2)-1);

//     lowcmd_publisher_->Write(low_cmd_);
//     std::cout << "sendLowCmd() is called." << std::endl;
// }

// void RobotInterface::recvLowState(const void* message)
// {
//     low_state_ = *(unitree_go::msg::dds_::LowState_*)message;
//     std::cout << "recvLowState() is called." << std::endl;
// }

// const RobotInterface::StateVector& RobotInterface::getJointPositions()
// {
//     joint_pos_ << low_state_.motor_state()[FR_H].q(),
//                   low_state_.motor_state()[FR_T].q(),
//                   low_state_.motor_state()[FR_C].q(),
//                   low_state_.motor_state()[FL_H].q(),
//                   low_state_.motor_state()[FL_T].q(),
//                   low_state_.motor_state()[FL_C].q(),
//                   low_state_.motor_state()[RR_H].q(),
//                   low_state_.motor_state()[RR_T].q(),
//                   low_state_.motor_state()[RR_C].q(),
//                   low_state_.motor_state()[RL_H].q(),
//                   low_state_.motor_state()[RL_T].q(),
//                   low_state_.motor_state()[RL_C].q();
//     return joint_pos_;
//     low_state_.motor_state()[RL_H].q(),
//     low_state_.motor_state()[RL_T].q(),
//     low_state_.motor_state()[RL_C].q();
// return joint_pos_;
// }

// const RobotInterface::StateVector& RobotInterface::getJointVelocities()
// {
// joint_vel_ << low_state_.motor_state()[FR_H].dq(),
//     low_state_.motor_state()[FR_T].dq(),
//     low_state_.motor_state()[FR_C].dq(),
//     low_state_.motor_state()[FL_H].dq(),
//     low_state_.motor_state()[FL_T].dq(),
//     low_state_.motor_state()[FL_C].dq(),
//     low_state_.motor_state()[RR_H].dq(),
//     low_state_.motor_state()[RR_T].dq(),
//     low_state_.motor_state()[RR_C].dq(),
//     low_state_.motor_state()[RL_H].dq(),
//     low_state_.motor_state()[RL_T].dq(),
//     low_state_.motor_state()[RL_C].dq();
// return joint_vel_;
// }

// const RobotInterface::StateVector& RobotInterface::getJointTorques()
// {
// joint_trq_ << low_state_.motor_state()[FR_H].tau_est(),
//     low_state_.motor_state()[FR_T].tau_est(),
//     low_state_.motor_state()[FR_C].tau_est(),
//     low_state_.motor_state()[FL_H].tau_est(),
//     low_state_.motor_state()[FL_T].tau_est(),
//     low_state_.motor_state()[FL_C].tau_est(),
//     low_state_.motor_state()[RR_H].tau_est(),
//     low_state_.motor_state()[RR_T].tau_est(),
//     low_state_.motor_state()[RR_C].tau_est(),
//     low_state_.motor_state()[RL_H].tau_est(),
//     low_state_.motor_state()[RL_T].tau_est(),
//     low_state_.motor_state()[RL_C].tau_est();
// return joint_trq_;
// }}

// const RobotInterface::StateVector& RobotInterface::getJointVelocities()
// {
//     joint_vel_ << low_state_.motor_state()[FR_H].dq(),
//                   low_state_.motor_state()[FR_T].dq(),
//                   low_state_.motor_state()[FR_C].dq(),
//                   low_state_.motor_state()[FL_H].dq(),
//                   low_state_.motor_state()[FL_T].dq(),
//                   low_state_.motor_state()[FL_C].dq(),
//                   low_state_.motor_state()[RR_H].dq(),
//                   low_state_.motor_state()[RR_T].dq(),
//                   low_state_.motor_state()[RR_C].dq(),
//                   low_state_.motor_state()[RL_H].dq(),
//                   low_state_.motor_state()[RL_T].dq(),
//                   low_state_.motor_state()[RL_C].dq();
//     return joint_vel_;
// }

// const RobotInterface::StateVector& RobotInterface::getJointTorques()
// {
//     joint_trq_ << low_state_.motor_state()[FR_H].tau_est(),
//                   low_state_.motor_state()[FR_T].tau_est(),
//                   low_state_.motor_state()[FR_C].tau_est(),
//                   low_state_.motor_state()[FL_H].tau_est(),
//                   low_state_.motor_state()[FL_T].tau_est(),
//                   low_state_.motor_state()[FL_C].tau_est(),
//                   low_state_.motor_state()[RR_H].tau_est(),
//                   low_state_.motor_state()[RR_T].tau_est(),
//                   low_state_.motor_state()[RR_C].tau_est(),
//                   low_state_.motor_state()[RL_H].tau_est(),
//                   low_state_.motor_state()[RL_T].tau_est(),
//                   low_state_.motor_state()[RL_C].tau_est();
//     return joint_trq_;
// }

// void RobotInterface::homing()
// {
//     bool is_valid = false;
//     is_valid = (q_homing_.array() >= joint_pos_min_.array()).all() && 
//                (q_homing_.array() <= joint_pos_max_.array()).all();

//     assert(is_valid && "homing positions are not valid!");

//     int steps = static_cast<int>(homing_duration_ / timestep_);
//     int counter = 0;
//     double rate = 0.0;
//     StateVector q_init = getJointPositions();
//     // StateVector q_prev = q_init;
//     StateVector q_des, dq_des;

//     std::cout << "q_homing_: " << q_homing_.transpose() << std::endl;

//     // // Gravity compensation
//     // low_cmd_.motor_cmd()[0].tau() = -2.50f;  // FR hip
//     // low_cmd_.motor_cmd()[3].tau() = +2.50f;  // FL hip 
//     // low_cmd_.motor_cmd()[6].tau() = -2.50f;  // RR hip
//     // low_cmd_.motor_cmd()[9].tau() = +2.50f;  // RL hip

//     while (counter <= steps) {
//         counter++;

//         // first, get record initial position
//         if (counter >= 0 && counter <= 10) {
//             q_init = getJointPositions();
//             q_des = q_init;

//             std::cout << "q_init: " << q_init.transpose() << std::endl;

//             // Set motor control gains
//             // low_cmd_.motor_cmd()[0].kp() = 5.0;
//             // low_cmd_.motor_cmd()[0].kd() = 1.0;
//             // low_cmd_.motor_cmd()[1].kp() = 5.0;
//             // low_cmd_.motor_cmd()[1].kd() = 1.0;
//             // low_cmd_.motor_cmd()[2].kp() = 5.0;
//             // low_cmd_.motor_cmd()[2].kd() = 1.0;

//             // low_cmd_.motor_cmd()[3].kp() = 5.0;
//             // low_cmd_.motor_cmd()[3].kd() = 0.1;
//             // low_cmd_.motor_cmd()[4].kp() = 5.0;
//             // low_cmd_.motor_cmd()[4].kd() = 0.1;
//             // low_cmd_.motor_cmd()[5].kp() = 5.0;
//             // low_cmd_.motor_cmd()[5].kd() = 0.1;

//             // low_cmd_.motor_cmd()[6].kp() = 5.0;
//             // low_cmd_.motor_cmd()[6].kd() = 0.1;
//             // low_cmd_.motor_cmd()[7].kp() = 5.0;
//             // low_cmd_.motor_cmd()[7].kd() = 0.1;
//             // low_cmd_.motor_cmd()[8].kp() = 5.0;
//             // low_cmd_.motor_cmd()[8].kd() = 0.1;

//             // low_cmd_.motor_cmd()[9].kp() = 5.0;
//             // low_cmd_.motor_cmd()[9].kd() = 0.1;
//             // low_cmd_.motor_cmd()[10].kp() = 5.0;
//             // low_cmd_.motor_cmd()[10].kd() = 0.1;
//             // low_cmd_.motor_cmd()[11].kp() = 5.0;
//             // low_cmd_.motor_cmd()[11].kd() = 0.1;
//         }
//         else {
//             rate = static_cast<double>(counter) / static_cast<double>(steps);
//             double scale = 0.5 * (1.0 - std::cos(M_PI * rate));
    
//             // Calculate the desired joint positions and velocities
//             q_des = q_init + scale * (q_homing_ - q_init);
//             // dq_des = (q_des - q_prev) / timestep_;

//             std::cout << "q_des: " << q_des.transpose() << std::endl;
//             std::cout << "q_act: " << getJointPositions().transpose() << std::endl;

//             // Update the previous desired positions
//             // q_prev = q_des;  
    
//             // // Set motor control gains
//             // low_cmd_.motor_cmd()[0].kp() = 15.0;
//             // low_cmd_.motor_cmd()[0].kd() = 1.0;
//             // low_cmd_.motor_cmd()[1].kp() = 15.0;
//             // low_cmd_.motor_cmd()[1].kd() = 1.0;
//             // low_cmd_.motor_cmd()[2].kp() = 25.0;
//             // low_cmd_.motor_cmd()[2].kd() = 1.0;
    
//             // low_cmd_.motor_cmd()[3].kp() = 5.0;
//             // low_cmd_.motor_cmd()[3].kd() = 0.1;
//             // low_cmd_.motor_cmd()[4].kp() = 5.0;
//             // low_cmd_.motor_cmd()[4].kd() = 0.1;
//             // low_cmd_.motor_cmd()[5].kp() = 5.0;
//             // low_cmd_.motor_cmd()[5].kd() = 0.1;
    
//             // low_cmd_.motor_cmd()[6].kp() = 5.0;
//             // low_cmd_.motor_cmd()[6].kd() = 0.1;
//             // low_cmd_.motor_cmd()[7].kp() = 5.0;
//             // low_cmd_.motor_cmd()[7].kd() = 0.1;
//             // low_cmd_.motor_cmd()[8].kp() = 5.0;
//             // low_cmd_.motor_cmd()[8].kd() = 0.1;
    
//             // low_cmd_.motor_cmd()[9].kp() = 5.0;
//             // low_cmd_.motor_cmd()[9].kd() = 0.1;
//             // low_cmd_.motor_cmd()[10].kp() = 5.0;
//             // low_cmd_.motor_cmd()[10].kd() = 0.1;
//             // low_cmd_.motor_cmd()[11].kp() = 5.0;
//             // low_cmd_.motor_cmd()[11].kd() = 0.1;
//         }

//         // // Set desired joint positions and velocities
//         // for (int i = 0; i < kNumJoint; ++i) {
//         //     low_cmd_.motor_cmd()[i].q() = q_des[i];
//         //     low_cmd_.motor_cmd()[i].dq() = 0.0;
//         //     // low_cmd_.motor_cmd()[i].dq() = dq_des[i];
//         // }
//         low_cmd_.motor_cmd()[0].q() = q_des[0];
//         low_cmd_.motor_cmd()[0].dq() = 0.0;
//         low_cmd_.motor_cmd()[0].kp() = 5.0;
//         low_cmd_.motor_cmd()[0].kd() = 1.0;
//         low_cmd_.motor_cmd()[0].tau() = 0.0;

//         low_cmd_.motor_cmd()[1].q() = q_des[1];
//         low_cmd_.motor_cmd()[1].dq() = 0.0;
//         low_cmd_.motor_cmd()[1].kp() = 5.0;
//         low_cmd_.motor_cmd()[1].kd() = 1.0;
//         low_cmd_.motor_cmd()[1].tau() = 0.0;

//         low_cmd_.motor_cmd()[2].q() = q_des[2];
//         low_cmd_.motor_cmd()[2].dq() = 0.0;
//         low_cmd_.motor_cmd()[2].kp() = 25.0;
//         low_cmd_.motor_cmd()[2].kd() = 1.0;
//         low_cmd_.motor_cmd()[2].tau() = 0.0;

//         // std::cout << "q_des: " << q_des.transpose() << std::endl;
//         // std::cout << "q_act: " << getJointPositions().transpose() << std::endl;

//         sendLowCmd();

//         usleep(timestep_ * USECS_PER_SEC);
//     }

//     StateVector q_end = getJointPositions();
//     assert(q_end.isApprox(q_homing_, 1e-1) && "Homing failed!");

//     std::cout << "Finish moving to homing position.\n" << std::endl;
// }

// const JointController& RobotInterface::getJointController() const 
// {
//     return controller_;
// }

// double RobotInterface::getTimeSinceStart() const 
// {
//     return step_counter_ * timestep_;
// }

// void RobotInterface::setFrontLeftLegJointPositions(const Vector3d& positions)
// {
//     low_cmd_.motor_cmd()[3].q() = positions[0];
//     low_cmd_.motor_cmd()[4].q() = positions[1];
//     low_cmd_.motor_cmd()[5].q() = positions[2];
// }

// void RobotInterface::setFrontRightLegJointPositions(const Vector3d& positions)
// {
//     low_cmd_.motor_cmd()[0].q() = positions[0];
//     low_cmd_.motor_cmd()[1].q() = positions[1];
//     low_cmd_.motor_cmd()[2].q() = positions[2];
// }

// void RobotInterface::setRearLeftLegJointPositions(const Vector3d& positions)
// {
//     low_cmd_.motor_cmd()[9].q() = positions[0];
//     low_cmd_.motor_cmd()[10].q() = positions[1];
//     low_cmd_.motor_cmd()[11].q() = positions[2];
// }

// void RobotInterface::setRearRightLegJointPositions(const Vector3d& positions)
// {
//     low_cmd_.motor_cmd()[6].q() = positions[0];
//     low_cmd_.motor_cmd()[7].q() = positions[1];
//     low_cmd_.motor_cmd()[8].q() = positions[2];
// }

// void RobotInterface::setFrontLeftLegJointVelocities(const Vector3d& velocities)
// {
//     low_cmd_.motor_cmd()[3].dq() = velocities[0];
//     low_cmd_.motor_cmd()[4].dq() = velocities[1];
//     low_cmd_.motor_cmd()[5].dq() = velocities[2];
// }

// void RobotInterface::setFrontRightLegJointVelocities(const Vector3d& velocities)
// {
//     low_cmd_.motor_cmd()[0].dq() = velocities[0];
//     low_cmd_.motor_cmd()[1].dq() = velocities[1];
//     low_cmd_.motor_cmd()[2].dq() = velocities[2];
// }

// void RobotInterface::setRearLeftLegJointVelocities(const Vector3d& velocities)
// {
//     low_cmd_.motor_cmd()[9].dq() = velocities[0];
//     low_cmd_.motor_cmd()[10].dq() = velocities[1];
//     low_cmd_.motor_cmd()[11].dq() = velocities[2];
// }

// void RobotInterface::setRearRightLegJointVelocities(const Vector3d& velocities)
// {
//     low_cmd_.motor_cmd()[6].dq() = velocities[0];
//     low_cmd_.motor_cmd()[7].dq() = velocities[1];
//     low_cmd_.motor_cmd()[8].dq() = velocities[2];
// }

// void RobotInterface::setFrontLeftLegJointTorques(const Vector3d& torques)
// {
//     low_cmd_.motor_cmd()[3].tau() = torques[0];
//     low_cmd_.motor_cmd()[4].tau() = torques[1];
//     low_cmd_.motor_cmd()[5].tau() = torques[2];
// }

// void RobotInterface::setFrontRightLegJointTorques(const Vector3d& torques)
// {
//     low_cmd_.motor_cmd()[0].tau() = torques[0];
//     low_cmd_.motor_cmd()[1].tau() = torques[1];
//     low_cmd_.motor_cmd()[2].tau() = torques[2];
// }

// void RobotInterface::setRearLeftLegJointTorques(const Vector3d& torques)
// {
//     low_cmd_.motor_cmd()[9].tau() = torques[0];
//     low_cmd_.motor_cmd()[10].tau() = torques[1];
//     low_cmd_.motor_cmd()[11].tau() = torques[2];
// }

// void RobotInterface::setRearRightLegJointTorques(const Vector3d& torques)
// {
//     low_cmd_.motor_cmd()[6].tau() = torques[0];
//     low_cmd_.motor_cmd()[7].tau() = torques[1];
//     low_cmd_.motor_cmd()[8].tau() = torques[2];
// }

// void RobotInterface::setFrontLeftLegJointPositionGains(const Vector3d& gains)
// {
//     low_cmd_.motor_cmd()[3].kp() = gains[0];
//     low_cmd_.motor_cmd()[4].kp() = gains[1];
//     low_cmd_.motor_cmd()[5].kp() = gains[2];
// }

// void RobotInterface::setFrontRightLegJointPositionGains(const Vector3d& gains)
// {
//     low_cmd_.motor_cmd()[0].kp() = gains[0];
//     low_cmd_.motor_cmd()[1].kp() = gains[1];
//     low_cmd_.motor_cmd()[2].kp() = gains[2];
// }

// void RobotInterface::setRearLeftLegJointPositionGains(const Vector3d& gains)
// {
//     low_cmd_.motor_cmd()[9].kp() = gains[0];
//     low_cmd_.motor_cmd()[10].kp() = gains[1];
//     low_cmd_.motor_cmd()[11].kp() = gains[2];
// }

// void RobotInterface::setRearRightLegJointPositionGains(const Vector3d& gains)
// {
//     low_cmd_.motor_cmd()[6].kp() = gains[0];
//     low_cmd_.motor_cmd()[7].kp() = gains[1];
//     low_cmd_.motor_cmd()[8].kp() = gains[2];
// }

// void RobotInterface::setFrontLeftLegJointVelocityGains(const Vector3d& gains)
// {
//     low_cmd_.motor_cmd()[3].kd() = gains[0];
//     low_cmd_.motor_cmd()[4].kd() = gains[1];
//     low_cmd_.motor_cmd()[5].kd() = gains[2];
// }

// void RobotInterface::setFrontRightLegJointVelocityGains(const Vector3d& gains)
// {
//     low_cmd_.motor_cmd()[0].kd() = gains[0];
//     low_cmd_.motor_cmd()[1].kd() = gains[1];
//     low_cmd_.motor_cmd()[2].kd() = gains[2];
// }

// void RobotInterface::setRearLeftLegJointVelocityGains(const Vector3d& gains)
// {
//     low_cmd_.motor_cmd()[9].kd() = gains[0];
//     low_cmd_.motor_cmd()[10].kd() = gains[1];
//     low_cmd_.motor_cmd()[11].kd() = gains[2];
// }

// void RobotInterface::setRearRightLegJointVelocityGains(const Vector3d& gains)
// {
//     low_cmd_.motor_cmd()[6].kd() = gains[0];
//     low_cmd_.motor_cmd()[7].kd() = gains[1];
//     low_cmd_.motor_cmd()[8].kd() = gains[2];
// }

// RobotInterface::Vector4d RobotInterface::getBaseImuQuaternion()
// {
//     // [x, y, z, w]
//     Vector4d quaternion;
//     quaternion << low_state_.imu_state().quaternion()[1],
//                   low_state_.imu_state().quaternion()[2],
//                   low_state_.imu_state().quaternion()[3],
//                   low_state_.imu_state().quaternion()[0];
//     return quaternion;
// }

// RobotInterface::Vector3d RobotInterface::getBaseImuEulerRPY()
// {
//     // [roll, pitch, yaw]
//     Vector3d euler_rpy;
//     euler_rpy << low_state_.imu_state().rpy()[0],
//                  low_state_.imu_state().rpy()[1],
//                  low_state_.imu_state().rpy()[2];
//     return euler_rpy;
// }

// RobotInterface::Vector3d RobotInterface::getBaseImuAngularVelocity()
// {
//     // [wx, wy, wz]
//     Vector3d angular_velocity;
//     angular_velocity << low_state_.imu_state().gyroscope()[0],
//                         low_state_.imu_state().gyroscope()[1],
//                         low_state_.imu_state().gyroscope()[2];
//     return angular_velocity;
// }

// RobotInterface::Vector3d RobotInterface::getBaseImuLinearAcceleration()
// {
//     // [ax, ay, az]
//     Vector3d linear_acceleration;
//     linear_acceleration << low_state_.imu_state().accelerometer()[0],
//                            low_state_.imu_state().accelerometer()[1],
//                            low_state_.imu_state().accelerometer()[2];
//     return linear_acceleration;
// }

// RobotInterface::Vector3d RobotInterface::getFrontLeftLegJointPositions()
// {
//     // [hip, thigh, calf]
//     Vector3d positions;
//     positions << low_state_.motor_state()[3].q(), 
//                  low_state_.motor_state()[4].q(),
//                  low_state_.motor_state()[5].q();
//     return positions;
// }

// RobotInterface::Vector3d RobotInterface::getFrontRightLegJointPositions()
// {
//     // [hip, thigh, calf]
//     Vector3d positions;
//     positions << low_state_.motor_state()[0].q(),
//                  low_state_.motor_state()[1].q(),
//                  low_state_.motor_state()[2].q();
//     return positions;
// }

// RobotInterface::Vector3d RobotInterface::getRearLeftLegJointPositions()
// {
//     // [hip, thigh, calf]
//     Vector3d positions;
//     positions << low_state_.motor_state()[9].q(),
//                  low_state_.motor_state()[10].q(),
//                  low_state_.motor_state()[11].q();
//     return positions;
// }

// RobotInterface::Vector3d RobotInterface::getRearRightLegJointPositions()
// {
//     // [hip, thigh, calf]
//     Vector3d positions;
//     positions << low_state_.motor_state()[6].q(),
//                  low_state_.motor_state()[7].q(),
//                  low_state_.motor_state()[8].q();
//     return positions;
// }

// RobotInterface::Vector3d RobotInterface::getFrontLeftLegJointVelocities()
// {
//     // [hip, thigh, calf]
//     Vector3d velocities;
//     velocities << low_state_.motor_state()[3].dq(),
//                   low_state_.motor_state()[4].dq(),
//                   low_state_.motor_state()[5].dq();
//     return velocities;
// }

// RobotInterface::Vector3d RobotInterface::getFrontRightLegJointVelocities()
// {
//     // [hip, thigh, calf]
//     Vector3d velocities;
//     velocities << low_state_.motor_state()[0].dq(),
//                   low_state_.motor_state()[1].dq(),
//                   low_state_.motor_state()[2].dq();
//     return velocities;
// }

// RobotInterface::Vector3d RobotInterface::getRearLeftLegJointVelocities()
// {
//     // [hip, thigh, calf]
//     Vector3d velocities;
//     velocities << low_state_.motor_state()[9].dq(),
//                   low_state_.motor_state()[10].dq(),
//                   low_state_.motor_state()[11].dq();
//     return velocities;
// }

// RobotInterface::Vector3d RobotInterface::getRearRightLegJointVelocities()
// {
//     // [hip, thigh, calf]
//     Vector3d velocities;
//     velocities << low_state_.motor_state()[6].dq(),
//                   low_state_.motor_state()[7].dq(),
//                   low_state_.motor_state()[8].dq();
//     return velocities;
// }

// RobotInterface::Vector3d RobotInterface::getFrontLeftLegJointTorques()
// {
//     // [hip, thigh, calf]
//     Vector3d torques;
//     torques << low_state_.motor_state()[3].tau_est(),
//                low_state_.motor_state()[4].tau_est(),
//                low_state_.motor_state()[5].tau_est();
//     return torques;
// }

// RobotInterface::Vector3d RobotInterface::getFrontRightLegJointTorques()
// {
//     // [hip, thigh, calf]
//     Vector3d torques; 
//     torques << low_state_.motor_state()[0].tau_est(),
//                low_state_.motor_state()[1].tau_est(),
//                low_state_.motor_state()[2].tau_est();
//     return torques;
// }

// RobotInterface::Vector3d RobotInterface::getRearLeftLegJointTorques()
// {
//     // [hip, thigh, calf]
//     Vector3d torques;
//     torques << low_state_.motor_state()[9].tau_est(),
//                low_state_.motor_state()[10].tau_est(),
//                low_state_.motor_state()[11].tau_est();
//     return torques;
// }

// RobotInterface::Vector3d RobotInterface::getRearRightLegJointTorques()
// {
//     // [hip, thigh, calf]
//     Vector3d torques;
//     torques << low_state_.motor_state()[6].tau_est(),
//                low_state_.motor_state()[7].tau_est(),
//                low_state_.motor_state()[8].tau_est();
//     return torques;
// }

// double RobotInterface::getFrontLeftFootContactForce()
// {
//     double force = low_state_.foot_force()[1];
//     return force;
// }

// double RobotInterface::getFrontRightFootContactForce()
// {
//     double force = low_state_.foot_force()[0];
//     return force;
// }

// double RobotInterface::getRearLeftFootContactForce()
// {
//     double force = low_state_.foot_force()[3];
//     return force;
// }

// double RobotInterface::getRearRightFootContactForce()
// {
//     double force = low_state_.foot_force()[2];
//     return force;
// }

// void RobotInterface::applyJointActions(const ActionVector& actions)
// {
//     VectorXd joint_positions = getJointPositions();
//     VectorXd joint_velocities = getJointVelocities();

//     VectorXd joint_actions;
//     joint_actions.setZero(actions.size());
//     // FR leg
//     joint_actions.segment(0, kDimAction * kNumJointPerLeg) = 
//         actions.segment(15, kDimAction * kNumJointPerLeg);
//     // FL leg
//     joint_actions.segment(15, kDimAction * kNumJointPerLeg) = 
//         actions.segment(0, kDimAction * kNumJointPerLeg);
//     // RR leg
//     joint_actions.segment(30, kDimAction * kNumJointPerLeg) = 
//         actions.segment(45, kDimAction * kNumJointPerLeg);
//     // RL leg
//     joint_actions.segment(45, kDimAction * kNumJointPerLeg) = 
//         actions.segment(30, kDimAction * kNumJointPerLeg);

//     VectorXd clipped_torques = controller_.computeJointTorques(
//         joint_actions, joint_positions, joint_velocities);

//     for (int i = 0; i < kNumJoint; ++i) {
//         low_cmd_.motor_cmd()[i].q() = PosStopF;
//         low_cmd_.motor_cmd()[i].kp() = 0.0;
//         low_cmd_.motor_cmd()[i].dq() = VelStopF;
//         low_cmd_.motor_cmd()[i].kd() = 0.0;
//         low_cmd_.motor_cmd()[i].tau() = clipped_torques[i];
//     }

//     step_counter_++;
// }

}  // namespace go2_hw