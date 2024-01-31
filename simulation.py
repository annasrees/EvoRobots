from world import WORLD
from robot import ROBOT

import pybullet as p
import pybullet_data
import pyrosim.pyrosim as pyrosim
import numpy as np
import constants as c

class SIMULATION:
    def __init__(self):
        '''
        27. Cut the statements from simulate.py that connect to pybullet, 
        set the additional search path, set gravity, and 
        Prepare_To_Simulate(), and paste them into SIMULATION's 
        constructor. For statements that create a variable, 
        add the self. prefix to the variable's name so that 
        it becomes an instance attribute.
        '''
        self.physicsClient = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0,0,-9.8)
        self.world = WORLD()
        self.robot = ROBOT()
        pyrosim.Prepare_To_Simulate(robot.robotId)

        '''
        29. cut the statements from simulate.py that load
        world.sdf and plane.urdf and paste them into WORLD's 
        constructor. Remember to add the self. prefix where 
        needed.
        '''
        p.loadSDF("world.sdf")
        self.planeId = p.loadURDF("plane.urdf")

