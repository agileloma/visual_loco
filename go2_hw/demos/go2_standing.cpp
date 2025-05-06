/* ----------------------------------------------------------------------------
 * Copyright (c) 2025, Harbin Institute of Technology.
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   go2_standing.hpp
 * @author Jun Li (junli@hit.edu.cn)
 * @brief  Demo for Go2 standing
 * @date   April 28, 2025
 **/

#include <atomic>
#include <csignal>
#include <string>
#include <iostream>

#include <Eigen/Dense>

#include "go2_hw/robot_interface.hpp"

/**
 * Interrupt signal flag
 * This is set to true if an interrupt signal (e.g. Ctrl-c)
 * is sent to the process.
 */
std::atomic<bool> sigint{false};

/**
 * Handle the interrupt signal.
 * This enables the user to stop the program while still keeping the robot safe.
 */
void handleSigint(int )
{
    sigint = true;
}

int main(int argc, char *argv[])
{
    std::cout << "Press Enter to continue..." << std::endl;
    std::cin.ignore();

    // Load configuration file
    std::string cfg_file;
    if (argc == 2) {
        cfg_file = std::string(argv[1]);
    }
    else {
        std::cout << "Usage: ./demo /<config file within root folder>" << std::endl;
        return 1;
    }

    go2_hw::RobotInterface hw_robot(ROOT_PATH, cfg_file);
    // hw_robot.homing();


    // hw_robot.setGravitycompensation();
    // std::cout << "after setGravitycompensation()" << std::endl;

    Eigen::Matrix<double, go2_hw::kNumJoint, 1> desired_joint_positions;
    desired_joint_positions << 0.00, 1.36, -2.65,  // Front Left leg
                               0.00, 1.36, -2.65,  // Front Right leg
                               0.00, 1.36, -2.65,  // Rear Left leg
                               0.00, 1.36, -2.65;  // Rear Right leg

    Eigen::Matrix<double, go2_hw::kNumJoint, 1> desired_joint_velocities;
    desired_joint_velocities << 0.00, 0.00, 0.00,  // Front Left leg
                                0.00, 0.00, 0.00,  // Front Right leg
                                0.00, 0.00, 0.00,  // Rear Left leg
                                0.00, 0.00, 0.00;  // Rear Right leg    

    double kp = 15.0;
    double kd = 1.0;

    // Eigen::VectorXd desired_joint_actions;
    // for (int i = 0; i < go2_hw::kNumJoint; ++i) {
    //     desired_joint_actions[go2_hw::kDimAction * i + 0] = desired_joint_positions[i];
    //     desired_joint_actions[go2_hw::kDimAction * i + 1] = 10.0;
    //     desired_joint_actions[go2_hw::kDimAction * i + 2] = desired_joint_velocities[i];
    //     desired_joint_actions[go2_hw::kDimAction * i + 3] = 0.5;
    //     desired_joint_actions[go2_hw::kDimAction * i + 4] = 0.0;
    // }

    signal(SIGINT, handleSigint);

    while (!sigint)
    {
        // std::cout << "Time since Start: " << hw_robot.getTimeSinceStart() << std::endl;
        // std::cout << "Base IMU Quaternion: " << hw_robot.getBaseImuQuaternion().transpose() << std::endl;
        // std::cout << "Base IMU Euler RPY: " << hw_robot.getBaseImuEulerRPY().transpose() << std::endl;
        // std::cout << "Front Right Positions: " << hw_robot.getFrontRightLegJointPositions().transpose() << std::endl;
        // std::cout << "Front Right velocities: " << hw_robot.getFrontRightLegJointVelocities().transpose() << std::endl;

        double des_fr_hip_tau = kp * (desired_joint_positions(0) - hw_robot.accessLowState().motor_state()[0].q()) + kd * (desired_joint_velocities(0) - hw_robot.accessLowState().motor_state()[0].dq());
        // std::cout << "des_fr_hip_tau: " << des_fr_hip_tau << std::endl;
        std::cout << "fr_hip_pos_err: " << desired_joint_positions(0) - hw_robot.accessLowState().motor_state()[0].q() << std::endl;

        double des_fr_thigh_tau = kp * (desired_joint_positions(1) - hw_robot.accessLowState().motor_state()[1].q()) + kd * (desired_joint_velocities(1) - hw_robot.accessLowState().motor_state()[1].dq());
        std::cout << "fr_thigh_pos_err: " << desired_joint_positions(1) - hw_robot.accessLowState().motor_state()[1].q() << std::endl;
        // std::cout << "des_fr_thigh_tau: " << des_fr_thigh_tau << std::endl;

        double des_fr_calf_tau = kp * (desired_joint_positions(2) - hw_robot.accessLowState().motor_state()[2].q()) + kd * (desired_joint_velocities(2) - hw_robot.accessLowState().motor_state()[2].dq());
        std::cout << "fr_calf_pos_err: " << desired_joint_positions(2) - hw_robot.accessLowState().motor_state()[2].q() << std::endl;
        // std::cout << "des_fr_thigh_tau: " << des_fr_thigh_tau << std::endl;

        hw_robot.accessLowCmd().motor_cmd()[0].tau() = des_fr_hip_tau;

        hw_robot.accessLowCmd().motor_cmd()[1].tau() = des_fr_thigh_tau;

        hw_robot.accessLowCmd().motor_cmd()[1].tau() = des_fr_calf_tau;

        // std::cout << "fr_thigh_pos: " << hw_robot.accessLowState().motor_state()[1].q() << "  fr_calf_pos: " << hw_robot.accessLowState().motor_state()[2].q() << std::endl;

        // std::cout << "hw_robot.accessLowState().motor_state()[0].tau_est(): " << hw_robot.accessLowState().motor_state()[0].tau_est() << std::endl;

        // hw_robot.applyJointActions(desired_joint_actions);
        // hw_robot.getFRhipTorque();

        sleep(0.002);
    }

    return 0;
}