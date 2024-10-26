"""
@file world.py
@package loco_bullet
@author Li Jun (junli@hit.edu.cn)
@license License BSD 3-Clause
@Copyright (c) 2024, Harbin Institute of Technology.
@date 2024-09-15
"""

import importlib.resources as importlib_resources

import time 
import pybullet as pyb

class World:
    """This class manages a PyBullet simulation environment and provides utility 
       functions to interact with :py:obj:`RobotWrapper` objects.

    Attributes:
        dt (float): The length of the simulation integration step.
        step_counter (int): The number of times the simulation has been integrated.
        data_rendering (bool): Whether to render camera data at each step.
        objects (list): The list of the PyBullet ids for all the non-robot objects.
        robots (list): The list of the robot wrapper of all added robots.
    """    
    def __init__(self, server=pyb.GUI, dt=0.001):
        """Initializes the PyBullet client.

        Args:
            server (int, optional): PyBullet server mode. pybullet.GUI creates a 
                graphical frontend using OpenGL while pybullet.DIRECT does not. 
                Defaults to pybullet.GUI.
            dt (float, optional): The length of the simulation integration step.
                Defaults to 0.001.
        """        
        self.dt = dt
        self.step_counter = 0
        self.data_rendering = False
        self.objects = []
        self.robots = []
        self.robot_tracking = False
        self.robot_to_track = 0

        self.physics_client = pyb.connect(server)
        pyb.setGravity(0, 0, -9.81)
        pyb.setPhysicsEngineParameter(fixedTimeStep=dt, numSubSteps=1)
        self.configure_gui()

    def add_robot(self, robot):
        """Adds a :py:obj:`RobotWrapper` object.

        Args:
            robot (:py:obj:`RobotWrapper`): Instance of a robot wrapper.

        Returns:
            robot (:py:obj:`RobotWrapper`): Instance of a robot wrapper.
        """
        self.robots.append(robot)
        return robot

    def add_object_from_urdf(
        self, urdf_path, pos=[0, 0, 0], ori=[0, 0, 0, 1], use_fixed_base=True
    ):
        """Adds an object described by a URDF file.

        Args:
            urdf_path (str): The absolute path of the URDF file.
            pos (list, optional): The initial position of the object in the world 
                frame. Defaults to [0, 0, 0].
            ori (list, optional): The initial orientation of the object in the 
                world frame, expressed in quaternions. Defaults to [0, 0, 0, 1].
            use_fixed_base (bool, optional): Determines if the robot base is fixed 
                or not. Defaults to True.

        Returns:
            [int]: The PyBullet id of the object if added successfully.
        """    
        # Load the object.
        object_id = pyb.loadURDF(
            urdf_path, 
            flags=pyb.URDF_USE_SELF_COLLISION, 
            useFixedBase=use_fixed_base
        )
        pyb.resetBasePositionAndOrientation(object_id, pos, ori)
        self.objects.append(object_id)
        return object_id
    
    def configure_gui(self, data_rendering=False, camera_distance=1.5, 
        camera_yaw=45, camera_pitch=-30, camera_target_position=[0., 0., 0.5]
    ):
        """Configures GUI.

        Args:
            data_rendering (bool, optional): Whether to render camera data.
            camera_distance (float, optional): Distance from eye to camera 
                target position.
            camera_yaw (float, optional): Camera yaw angle (in degrees) left/right.
            camera_pitch (float, optional): Camera pitch angle (in degrees) up/down.
            camera_target_position (list, optional): Camera focus point.
        """
        self.data_rendering = data_rendering
        if self.data_rendering:
            pyb.configureDebugVisualizer(pyb.COV_ENABLE_GUI, 1)
            pyb.configureDebugVisualizer(pyb.COV_ENABLE_RENDERING, 1)
        else:
            pyb.configureDebugVisualizer(pyb.COV_ENABLE_GUI, 0)

        pyb.resetDebugVisualizerCamera(
            cameraDistance=camera_distance,
            cameraYaw=camera_yaw,
            cameraPitch=camera_pitch,
            cameraTargetPosition=camera_target_position
        )

    def start_video_recording(self, file_name):
        """Starts video recording and save as a mp4 file.

        Args:
            file_name (str): The absolute path of the file to be saved.
        """        
        self.file_name = file_name
        self.log_id = pyb.startStateLogging(
            pyb.STATE_LOGGING_VIDEO_MP4, self.file_name
        )

    def stop_video_recording(self):
        """Stops video recording if any.
        """        
        if hasattr(self, "file_name") and hasattr(self, "log_id"):
            pyb.stopStateLogging(self.log_id)

    def debug(self):
        """Receives space key event to pause the simulation or start it again.
        """
        self.keys = pyb.getKeyboardEvents()
        # Pause simulation by space key event
        space_key = ord(' ')
        if space_key in self.keys and self.keys[space_key] & pyb.KEY_WAS_TRIGGERED:
            print("Simulation Paused!")
            print("Press Space key to start again!")
            while True:
                keys = pyb.getKeyboardEvents()
                if space_key in keys and keys[space_key] & pyb.KEY_WAS_TRIGGERED:
                    break

    def step(self, sleep=False):
        """Integrates the simulation one step forward.

        Args:
            sleep (bool, optional): Determines if the simulation sleeps for 
                :py:attr:`~dt` seconds at each step. Defaults to False.
        """        
        if sleep:
            time.sleep(self.dt)
        pyb.stepSimulation()
        self.step_counter += 1

        for robot in self.robots:
            robot.compute_numerical_quantities(self.dt)

        if self.data_rendering:
            pyb.getCameraImage(320,200)
            
        if self.robot_tracking == True:
            robot_pos = self.robots[self.robot_to_track-1].get_base_position()
            lookat = [robot_pos[0], robot_pos[1], robot_pos[2]]
            cam_info = pyb.getDebugVisualizerCamera()
            distance = cam_info[10]
            pitch = cam_info[9]
            yaw = cam_info[8]
            pyb.resetDebugVisualizerCamera(distance, yaw, pitch, lookat)

    def print_physics_engine_params(self):
        """Prints the parameters of the physics engine.
        """        
        params = pyb.getPhysicsEngineParameters(self.physicsClient)
        print("physics_engine_params:")
        for key in params:
            print("    - ", key, ": ", params[key])

    def get_time_since_start(self):
        """Gets the time passed (in seconds) since the simulation starts.
        """
        return self.step_counter * self.dt
        
        
    def track_robot(self, robot_id, enable_tracking=True):
        """Camera tracks a robot of a given id.
        
        Args:
            robot_id (int): the id of the robot to be tracked (only one robot
                            can be tracked at a time).
                            
            enable_tracking (bool, optional): turns tracking either on or off. 
        """
        self.robot_tracking = enable_tracking
        self.robot_to_track = robot_id


class WorldWithGround(World):
    """This class provides a shortcut to construct a PyBullet simulation 
       environment with a flat ground.
    """    
    def __init__(self, server=pyb.GUI, dt=0.001):     
        super().__init__(server, dt)
        with importlib_resources.path(__package__, "world.py") as p:
            package_dir = p.parent.parent.parent.parent.parent.absolute()
        plane_urdf = str(
            package_dir / "share" / "loma_bullet" / "resource" / "plane_with_restitution.urdf"
        )
        print("plane_urdf: ", plane_urdf)
        self.add_object_from_urdf(plane_urdf)