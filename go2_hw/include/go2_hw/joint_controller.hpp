/* ----------------------------------------------------------------------------
 * Copyright (c) 2025, Harbin Institute of Technology.
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   joint_controller.hpp
 *  @author Jun Li (junli@hit.edu.cn)
 *  @brief  Header file for JointController class
 *  @date   April 19, 2025
 **/

 #pragma once

 #include <Eigen/Dense>
 
 #include "go2_hw/go2_const.hpp"
 
 namespace go2_hw {
 
 class JointController
 {
 public:
     EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 
     typedef double Scalar;
     typedef Eigen::Matrix<Scalar, kNumJoint, 1> JointStateVector;
     typedef Eigen::Matrix<Scalar, kDimAction*kNumJoint, 1> JointActionVector;
     typedef Eigen::Ref<JointStateVector>                RefJointStateVector;
     typedef const Eigen::Ref<const JointStateVector>&   ConstRefJointStateVector;
     typedef Eigen::Ref<JointActionVector>               RefJointActionVector;
     typedef const Eigen::Ref<const JointActionVector>&  ConstRefJointActionVector;
 
     JointController(double torque_fector);
 
     const JointStateVector& getPositionFeedbackTerm() const;
     const JointStateVector& getVelocityFeedbackTerm() const;
     const JointStateVector& getTorqueFeedforwardTerm() const;
 
     const JointStateVector& getDesiredJointTorques() const;
 
     const JointStateVector& computeJointTorques(
            ConstRefJointActionVector des_actions, 
            ConstRefJointStateVector meas_pos, 
            ConstRefJointStateVector meas_vel);
 
 private:
     // limits
     JointStateVector max_tau_;
 
     // variables for output
     JointStateVector pos_fd_term_, vel_fd_term_, trq_ff_term_;
     JointStateVector des_trq_;
 };
 
 }  // namespace go2_hw