/* ----------------------------------------------------------------------------
 * Copyright (c) 2025, Harbin Institute of Technology.
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   hw_go2_interface.cpp
 *  @author Jun Li (junli@hit.edu.cn)
 *  @brief  Source file for hardware interface of Go2 robot
 *  @date   April 19, 2025
 **/

#include "go2_hw/hw_go2_interface.hpp"


namespace go2_hw {

HwGo2Interface::HwGo2Interface(const HwRobotSetting& setting)
    :channel_factory_(unitree::robot::ChannelFactory::Instance())
{
    channel_factory_->Init(0, setting.get(RobotStringParam_NetworkInterface));

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