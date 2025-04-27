/* ----------------------------------------------------------------------------
 * Copyright (c) 2025, Harbin Institute of Technology.
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   go2_const.hpp
 *  @author Jun Li (junli@hit.edu.cn)
 *  @brief  Header file for constant paprameters of A1 robot
 *  @date   April 19, 2025
 **/

 #pragma once

 namespace go2_hw {
 
 constexpr static int kNumLeg = 4;
 constexpr static int kNumJointPerLeg = 3;
 constexpr static int kNumJoint = 12;
 constexpr static int kDimAction = 5;  // (q, kp, dq, kd, tau)
 
 constexpr static double kHipMinPos = -1.0472;     // unit:rad
 constexpr static double kHipMaxPos = +1.0472;     // unit:rad
 constexpr static double kThighMinPos = -1.5708;   // unit:rad
 constexpr static double kThighMaxPos = +3.4907;   // unit:rad
 constexpr static double kCalfMinPos = -2.7227;    // unit:rad
 constexpr static double kCalfMaxPos = -0.83776;   // unit:rad
 
 constexpr static double kHipMaxVel = 30.1;        // unit:rad/s
 constexpr static double kThighMaxVel = 30.1;      // unit:rad/s
 constexpr static double kCalfMaxVel = 15.7;       // unit:rad/s
 
 constexpr static double kHipMaxTau = 23.7;        // unit:Nm
 constexpr static double kThighMaxTau = 23.7;      // unit:Nm
 constexpr static double kCalfMaxTau = 45.4;       // unit:Nm
 
 }  // namespace go2_hw