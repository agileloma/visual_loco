/* ----------------------------------------------------------------------------
 * Copyright (c) 2025, Harbin Institute of Technology.
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   hw_go2_interface.hpp
 *  @author Jun Li (junli@hit.edu.cn)
 *  @brief  Header file for hardware interface of Go2 robot
 *  @date   April 19, 2025
 **/

#pragma once

#include <iostream>

#include <Eigen/Dense>

#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/go2/LowState_.hpp>
#include <unitree/idl/go2/LowCmd_.hpp>
#include <unitree/common/time/time_tool.hpp>
#include <unitree/common/thread/thread.hpp>

#include "go2_hw/hw_robot_setting.hpp"
#include "go2_hw/joint_controller.hpp"

#define TOPIC_LOWCMD "rt/lowcmd"
#define TOPIC_LOWSTATE "rt/lowstate"

constexpr double PosStopF = (2.146E+9f);
constexpr double VelStopF = (16000.0f);


namespace go2_hw {

class HwGo2Interface
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef double Scalar;
    typedef Eigen::Matrix<Scalar, kNumJoint, 1> JointStateVector;
    typedef Eigen::Matrix<Scalar, kDimAction*kNumJoint, 1> JointActionVector;

    HwGo2Interface(const HwRobotSetting& setting);

    void homing();
    void sleeping();

    const JointController& getJointController() const;

    void applyJointActions(const Eigen::Ref<const JointActionVector>& actions);

    
private:
    double timestep_;

    /*manager for the creation and lifecycle of message channels*/
    unitree::robot::ChannelFactory* channel_factory_; 

    unitree_go::msg::dds_::LowCmd_ low_cmd_{};      // default init
    unitree_go::msg::dds_::LowState_ low_state_{};  // default init

    /*publisher*/
    unitree::robot::ChannelPublisherPtr<unitree_go::msg::dds_::LowCmd_> lowcmd_publisher_;
    /*subscriber*/
    unitree::robot::ChannelSubscriberPtr<unitree_go::msg::dds_::LowState_> lowstate_subscriber_;
};

}  // namespace go2_hw