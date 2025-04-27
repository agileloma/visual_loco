/* ----------------------------------------------------------------------------
 * Copyright (c) 2025, Harbin Institute of Technology.
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   joint_controller.cpp
 *  @author Jun Li (junli@hit.edu.cn)
 *  @brief  Source file for JointController class
 *  @date   April 19, 2025
 **/

#include "go2_hw/joint_controller.hpp"


namespace go2_hw {

JointController::JointController(double torque_fector)
{
    max_tau_ << kHipMaxTau, kThighMaxTau, kCalfMaxTau,  // FL leg
                kHipMaxTau, kThighMaxTau, kCalfMaxTau,  // FR leg
                kHipMaxTau, kThighMaxTau, kCalfMaxTau,  // RL leg
                kHipMaxTau, kThighMaxTau, kCalfMaxTau;  // RR leg
    max_tau_ *= torque_fector;

    pos_fd_term_.setZero();
    vel_fd_term_.setZero();
    trq_ff_term_.setZero();
    des_trq_.setZero();
}

const JointController::JointStateVector& 
JointController::getPositionFeedbackTerm() const 
{
    return pos_fd_term_; 
}

const JointController::JointStateVector& 
JointController::getVelocityFeedbackTerm() const 
{
    return vel_fd_term_; 
}

const JointController::JointStateVector& 
JointController::getTorqueFeedforwardTerm() const 
{
    return trq_ff_term_; 
}

const JointController::JointStateVector& 
JointController::getDesiredJointTorques() const 
{
    return des_trq_; 
}

const JointController::JointStateVector& JointController::computeJointTorques(
        ConstRefJointActionVector des_actions, 
        ConstRefJointStateVector meas_pos, 
        ConstRefJointStateVector meas_vel)
{
    for (std::size_t i = 0; i < kNumJoint; i++) {
        pos_fd_term_[i] = des_actions[kDimAction*i+1] * 
            (des_actions[kDimAction*i+0] - meas_pos[i]);
        vel_fd_term_[i] = des_actions[kDimAction*i+3] * 
            (des_actions[kDimAction*i+2] - meas_vel[i]);
        trq_ff_term_[i] = des_actions[kDimAction*i+4];
    }
    des_trq_ = pos_fd_term_ + vel_fd_term_ + trq_ff_term_;

    // clip joint torques 
    des_trq_ = des_trq_.cwiseMax(-max_tau_).cwiseMin(max_tau_);

    return des_trq_;
}

}  // namespace go2_hw