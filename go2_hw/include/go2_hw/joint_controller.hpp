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

    using StateVector = Eigen::Matrix<double, kNumJoint, 1>;
    using ActionVector = Eigen::Matrix<double, kDimAction*kNumJoint, 1>;
    using ConstRefStateVector = const Eigen::Ref<const StateVector>&;
    using ConstRefActionVector = const Eigen::Ref<const ActionVector>&;

    /**
     * @brief Constructor to initialize the controller with a scaling factor.
     * @param torque_factor Scaling factor for maximum torque.
     */
    JointController(double torque_factor);

    // Accessors for feedback and feedforward terms
    const StateVector& getPositionFeedbackTerm() const;
    const StateVector& getVelocityFeedbackTerm() const;
    const StateVector& getTorqueFeedforwardTerm() const;

    // Accessors to computed torques
    const StateVector& getDesiredJointTorques() const;

    /**
     * @brief Computes joint torques based on desired actions and measured states.
     * @param des_actions Desired joint actions.
     * @param meas_pos Measured joint positions.
     * @param meas_vel Measured joint velocities.
     * @return Computed joint torques.
     */
    const StateVector& computeJointTorques(
        ConstRefActionVector des_actions, 
        ConstRefStateVector meas_pos, 
        ConstRefStateVector meas_vel);

private:
    /**
     * @brief Validates the sizes of input vectors.
     * @param des_actions Desired joint actions.
     * @param meas_pos Measured joint positions.
     * @param meas_vel Measured joint velocities.
     */
    void validateInputSizes(ConstRefActionVector des_actions, 
                            ConstRefStateVector meas_pos, 
                            ConstRefStateVector meas_vel) const;
        
    // Maximum joint torque limits
    StateVector max_trq_;

    // variables for output
    StateVector pos_fd_term_, vel_fd_term_, trq_ff_term_;

    // Computed desired joint torques 
    StateVector des_trq_;
};

}  // namespace go2_hw