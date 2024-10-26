"""
@file robot_wrapper.py
@package loco_bullet
@author Li Jun (junli@hit.edu.cn)
@license License BSD 3-Clause
@Copyright (c) 2024, Harbin Institute of Technology.
@date 2024-09-15
"""

import numpy as np
import pybullet as pyb

class RobotWrapper:
  def __init__(self, configs, rob_vars="sim_rob_params"):
    """Initializes the simulation interface of a robot.
    
    Args:
      configs: a container for configuration parameters.    
    """
    self.urdf_file = configs[rob_vars]["urdf_file"]
    self.fixed_base = configs[rob_vars]["fixed_base"]
    self.base_name = configs[rob_vars]["base_name"]
    self.joint_names = configs[rob_vars]["joint_names"]
    self.limb_end_names = configs[rob_vars]["limb_end_names"]
    self.base_init_pos = np.array(configs[rob_vars]["base_init_pos"])
    self.base_init_vel = np.array(configs[rob_vars]["base_init_vel"])
    self.joint_init_pos = np.array(configs[rob_vars]["joint_init_pos"])
    self.joint_init_vel = np.array(configs[rob_vars]["joint_init_vel"])

    if self.fixed_base:
        self.nq = len(self.joint_names)
        self.nv = len(self.joint_names)
        self.nj = len(self.joint_names)
        self.ne = len(self.limb_end_names)
    else:
        self.nq = len(self.base_init_pos) + len(self.joint_init_pos)
        self.nv = len(self.base_init_vel) + len(self.joint_init_vel)
        self.nj = len(self.joint_names)
        self.ne = len(self.limb_end_names)

    self.robot_id = pyb.loadURDF(
        self.urdf_file, 
        self.base_init_pos[0:3], 
        self.base_init_pos[3:7], 
        useFixedBase=self.fixed_base,
        flags=pyb.URDF_USE_INERTIA_FROM_FILE
    )

    _, self.init_orn_inv = pyb.invertTransform(
        [0,0,0], self.base_init_pos[3:7]
    )

    # Query all the joints.
    num_joints = pyb.getNumJoints(self.robot_id)