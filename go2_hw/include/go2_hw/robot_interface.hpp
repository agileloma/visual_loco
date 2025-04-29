/* ----------------------------------------------------------------------------
 * Copyright (c) 2025, Harbin Institute of Technology.
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   robot_interface.hpp
 *  @author Jun Li (junli@hit.edu.cn)
 *  @brief  Header file for hardware interface of Go2 robot
 *  @date   April 28, 2025
 **/

#pragma once

#include <iostream>
#include <memory>

#include <Eigen/Dense>

#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/go2/LowState_.hpp>
#include <unitree/idl/go2/LowCmd_.hpp>
#include <unitree/common/time/time_tool.hpp>
#include <unitree/common/thread/thread.hpp>

#include "go2_hw/joint_controller.hpp"

#define TOPIC_LOWCMD "rt/lowcmd"
#define TOPIC_LOWSTATE "rt/lowstate"

constexpr double PosStopF = (2.146E+9f);
constexpr double VelStopF = (16000.0f);


namespace go2_hw {

class RobotInterface
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using VectorXd = Eigen::VectorXd;
    using JointStateVector = Eigen::Matrix<double, kNumJoint, 1>;
    using JointActionVector = Eigen::Matrix<double, kDimAction*kNumJoint, 1>;

    RobotInterface(const std::string& root_dir, 
                   const std::string& cfg_file, 
                   const std::string& hw_vars);

    void homing();
    void sleeping();

    const JointController& getJointController() const;

    void applyJointActions(const Eigen::Ref<const JointActionVector>& actions);

private:
    std::string network_interface_;

    double timestep_;
    double torque_factor_;

    VectorXd homing_configuration_;
    VectorXd cont_force_calibr_offset_, cont_force_calibr_factor_;

    unitree::robot::ChannelFactory* channel_factory_;

    unitree_go::msg::dds_::LowCmd_ low_cmd_{};      // default init
    unitree_go::msg::dds_::LowState_ low_state_{};  // default init

    /*publisher*/
    unitree::robot::ChannelPublisherPtr<
        unitree_go::msg::dds_::LowCmd_> lowcmd_publisher_;
    /*subscriber*/
    unitree::robot::ChannelSubscriberPtr<
        unitree_go::msg::dds_::LowState_> lowstate_subscriber_;
};

}  // namespace go2_hw