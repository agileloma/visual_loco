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
#include <unitree/robot/b2/motion_switcher/motion_switcher_client.hpp>

#include "go2_hw/robot_constants.hpp"
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

    using Vector3d = Eigen::Vector3d;
    using Vector4d = Eigen::Vector4d;
    using VectorXd = Eigen::VectorXd;
    using StateVector = Eigen::Matrix<double, kNumJoint, 1>;
    using ActionVector = Eigen::Matrix<double, kDimAction * kNumJoint, 1>;

    RobotInterface(const std::string& root_dir, 
                            const std::string& cfg_file, 
                            const std::string& hw_vars="hw_robot");

    unitree_go::msg::dds_::LowCmd_& accessLowCmd() { return low_cmd_; }
    unitree_go::msg::dds_::LowState_& accessLowState() { return low_state_; }

    void homing();
    void proning();

    // const JointController& getJointController() const;

    // double getTimeSinceStart() const;

    // void setFrontLeftLegJointPositions(const Vector3d& positions);
    // void setFrontRightLegJointPositions(const Vector3d& positions);
    // void setRearLeftLegJointPositions(const Vector3d& positions);
    // void setRearRightLegJointPositions(const Vector3d& positions);

    // void setFrontLeftLegJointVelocities(const Vector3d& velocities);
    // void setFrontRightLegJointVelocities(const Vector3d& velocities);
    // void setRearLeftLegJointVelocities(const Vector3d& velocities);
    // void setRearRightLegJointVelocities(const Vector3d& velocities);

    // void setFrontLeftLegJointTorques(const Vector3d& torques);
    // void setFrontRightLegJointTorques(const Vector3d& torques);
    // void setRearLeftLegJointTorques(const Vector3d& torques);
    // void setRearRightLegJointTorques(const Vector3d& torques);

    // void setFrontLeftLegJointPositionGains(const Vector3d& gains);
    // void setFrontRightLegJointPositionGains(const Vector3d& gains);
    // void setRearLeftLegJointPositionGains(const Vector3d& gains);
    // void setRearRightLegJointPositionGains(const Vector3d& gains);

    // void setFrontLeftLegJointVelocityGains(const Vector3d& gains);
    // void setFrontRightLegJointVelocityGains(const Vector3d& gains);
    // void setRearLeftLegJointVelocityGains(const Vector3d& gains);
    // void setRearRightLegJointVelocityGains(const Vector3d& gains);

    // Vector4d getBaseImuQuaternion();
    // Vector3d getBaseImuEulerRPY();
    // Vector3d getBaseImuAngularVelocity();
    // Vector3d getBaseImuLinearAcceleration();

    // Vector3d getFrontLeftLegJointPositions();
    // Vector3d getFrontRightLegJointPositions();
    // Vector3d getRearLeftLegJointPositions();
    // Vector3d getRearRightLegJointPositions();

    // Vector3d getFrontLeftLegJointVelocities();
    // Vector3d getFrontRightLegJointVelocities();
    // Vector3d getRearLeftLegJointVelocities();
    // Vector3d getRearRightLegJointVelocities();

    // Vector3d getFrontLeftLegJointTorques();
    // Vector3d getFrontRightLegJointTorques();
    // Vector3d getRearLeftLegJointTorques();
    // Vector3d getRearRightLegJointTorques();

    // double getFrontLeftFootContactForce();
    // double getFrontRightFootContactForce();
    // double getRearLeftFootContactForce();
    // double getRearRightFootContactForce();

    // void applyJointActions(const ActionVector& actions);

private:
    int queryMotionStatus();
    std::string queryServiceName(std::string form, std::string name);

    void sendLowCmd();
    void recvLowState(const void* message);

    const StateVector& getJointPositions();
    // const StateVector& getJointVelocities();
    // const StateVector& getJointTorques();

    double timestep_;

    int step_counter_;

    // // for joint PD control
    // double torque_factor_{1.0};
    // JointController controller_;

    // Joint position limits
    StateVector joint_pos_min_, joint_pos_max_;

    // for homing action
    StateVector q_homing_;  // represent the robot's default standing posture
    double homing_duration_{5.0};

    // for proning action
    StateVector q_proning_; // represent the robot's default sleeping posture
    double proning_duration_{5.0};

    // for footend force calibration
    Vector4d cont_force_calibr_offset_, cont_force_calibr_factor_;

    StateVector joint_pos_, joint_vel_, joint_trq_;

    // Init DDS channel
    unitree::robot::ChannelFactory* channel_factory_;
    // Close sport_mode
    unitree::robot::b2::MotionSwitcherClient* switcher_client_;

    unitree_go::msg::dds_::LowCmd_ low_cmd_{};      // default init
    unitree_go::msg::dds_::LowState_ low_state_{};  // default init

    /*publisher*/
    unitree::robot::ChannelPublisherPtr<
        unitree_go::msg::dds_::LowCmd_> lowcmd_publisher_;
    /*subscriber*/
    unitree::robot::ChannelSubscriberPtr<
        unitree_go::msg::dds_::LowState_> lowstate_subscriber_;

    /*LowCmd send thread*/
    unitree::common::ThreadPtr lowCmd_send_thread_;
};

}  // namespace go2_hw