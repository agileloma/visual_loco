/* ----------------------------------------------------------------------------
 * Copyright (c) 2025, Harbin Institute of Technology.
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   joint_controller.hpp
 * @author Jun Li (junli@hit.edu.cn)
 * @brief  Header file for JointController class
 * @date   April 28, 2025
 **/

 #pragma once

#include <iostream>
#include <stdexcept>

#include <Eigen/Dense>

#include "go2_hw/robot_constants.hpp"

namespace go2_hw {

/**
 * @class JointController
 * @brief Handles joint torque computation for all joints.
 */
class JointController
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using JointStateVector = Eigen::Matrix<double, kNumJoint, 1>;
    using JointActionVector = Eigen::Matrix<double, kDimAction*kNumJoint, 1> ;
    using ConstRefJointStateVector = const Eigen::Ref<const JointStateVector>&;
    using ConstRefJointActionVector = const Eigen::Ref<const JointActionVector>&;

    /**
     * @brief Constructor to initialize the controller with a scaling factor.
     * @param torque_factor Scaling factor for maximum torque.
     */
    JointController(double torque_factor);

    // Accessors for feedback and feedforward terms
    const JointStateVector& getPositionFeedbackTerm() const;
    const JointStateVector& getVelocityFeedbackTerm() const;
    const JointStateVector& getTorqueFeedforwardTerm() const;

    // Accessors to computed torques
    const JointStateVector& getDesiredJointTorques() const;

    /**
     * @brief Computes joint torques based on desired actions and measured states.
     * @param des_actions Desired joint actions.
     * @param meas_pos Measured joint positions.
     * @param meas_vel Measured joint velocities.
     * @return Computed joint torques.
     */
    const JointStateVector& computeJointTorques(
        ConstRefJointActionVector des_actions, 
        ConstRefJointStateVector meas_pos, 
        ConstRefJointStateVector meas_vel);

private:
    /**
     * @brief Validates the sizes of input vectors.
     * @param des_actions Desired joint actions.
     * @param meas_pos Measured joint positions.
     * @param meas_vel Measured joint velocities.
     */
    void validateInputSizes(ConstRefJointActionVector des_actions, 
                            ConstRefJointStateVector meas_pos, 
                            ConstRefJointStateVector meas_vel) const;
        
    // Maximum joint torque limits
    JointStateVector max_trq_;

    // variables for output
    JointStateVector pos_fd_term_, vel_fd_term_, trq_ff_term_;

    // Computed desired joint torques 
    JointStateVector des_trq_;
};

}  // namespace go2_hw