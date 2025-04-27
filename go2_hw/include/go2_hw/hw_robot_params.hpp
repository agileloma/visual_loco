/* ----------------------------------------------------------------------------
 * Copyright (c) 2025, Harbin Institute of Technology.
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   hw_robot_params.hpp
 *  @author Jun Li (junli@hit.edu.cn)
 *  @brief  Header file for parameters
 *  @date   April 19, 2025
 **/

 #pragma once

 namespace go2_hw {
 
 /*! Available string variables used for go2 robot */
 enum RobotStringParam {
     RobotStringParam_NetworkInterface,
 };
 
 /*! Available double variables used for go2 robot */
 enum RobotDoubleParam {
     RobotDoubleParam_Timestep,
     RobotDoubleParam_TorqueFactor,
 };
 
 /*! Available vector variables used for go2 robot */
 enum RobotVectorParam {
     RobotVectorParam_HomingPos,
     RobotVectorParam_ContForceCalibrOffset,
     RobotVectorParam_ContForceCalibrFactor,
 };
 
 }  // namespace go2_hw