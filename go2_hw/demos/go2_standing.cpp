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

#include <iostream>
#include <string>

#include "go2_hw/robot_interface.hpp"

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
    hw_robot.homing();
    std::cout << "joint positions: " << hw_robot.getJointPositions().transpose() << std::endl;

    while (1)
    {
        std::cout << "joint positions: " << hw_robot.getJointPositions().transpose() << std::endl;
        sleep(1);
    }

    return 0;
}