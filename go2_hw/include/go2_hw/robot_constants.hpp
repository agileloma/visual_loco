/* ----------------------------------------------------------------------------
 * Copyright (c) 2025, Harbin Institute of Technology.
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   robot_constants.hpp
 * @author Jun Li (junli@hit.edu.cn)
 * @brief  Header file for constant paprameters of the Go2 robot
 * @date   April 28, 2025
 */

#pragma once

namespace go2_hw {

// General robot parameters
constexpr int kNumLeg = 4;
constexpr int kNumJointPerLeg = 3;
constexpr int kNumJoint = 12;
constexpr int kDimAction = 5;  // (q, kp, dq, kd, tau)

// Joint position limits (in radians)
constexpr double kHipMinPos = -1.0472;     // unit:rad
constexpr double kHipMaxPos = +1.0472;     // unit:rad
constexpr double kThighMinPos = -1.5708;   // unit:rad
constexpr double kThighMaxPos = +3.4907;   // unit:rad
constexpr double kCalfMinPos = -2.7227;    // unit:rad
constexpr double kCalfMaxPos = -0.8377;    // unit:rad

// Joint velocity limits (in radians/second)
constexpr double kHipMaxVel = 30.1;        // unit:rad/s
constexpr double kThighMaxVel = 30.1;      // unit:rad/s
constexpr double kCalfMaxVel = 15.7;       // unit:rad/s

// Joint torque limits (in Newton-meters)
constexpr double kHipMaxTrq = 23.7;        // unit:Nm
constexpr double kThighMaxTrq = 23.7;      // unit:Nm
constexpr double kCalfMaxTrq = 45.4;       // unit:Nm

// Joint index
constexpr int FR_H = 0;     // Front Right Hip Joint
constexpr int FR_T = 1;     // Front Right Thigh Joint
constexpr int FR_C = 2;     // Front Right Calf Joint

constexpr int FL_H = 3;     // Front Left Hip Joint
constexpr int FL_T = 4;     // Front Left Thigh Joint
constexpr int FL_C = 5;     // Front Left Calf Joint

constexpr int RR_H = 6;     // Rear Right Hip Joint
constexpr int RR_T = 7;     // Rear Right Thigh Joint
constexpr int RR_C = 8;     // Rear Right Calf Joint

constexpr int RL_H = 9;     // Rear Left Hip Joint
constexpr int RL_T = 10;    // Rear Left Thigh Joint
constexpr int RL_C = 11;    // Rear Left Calf Joint

}  // namespace go2_hw