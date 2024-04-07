from world import WORLD
from robot import ROBOT

import pybullet as p
import pybullet_data
import time
import pyrosim.pyrosim as pyrosim
import numpy as np
import math
import random
import sys

class SIMULATION:
    def __init__(self, directOrGUI, solutionID):
        self.directOrGUI = directOrGUI
        if(self.directOrGUI == "DIRECT"):
            self.physicsClient = p.connect(p.DIRECT)
        else:
            self.physicsClient = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0,0,-9.8)

        p.loadSDF("world.sdf")
        self.planeId = p.loadURDF("plane.urdf")
        self.world = WORLD()
        self.robot = ROBOT(2, 2, solutionID)

        self.blockId = p.loadURDF("block.urdf")
        self.robot.set_block_id(self.blockId)        
        # pyrosim.Send_Cube(name="Block", pos=[1, 0, 1.5] , size=[0.5, 0.5, 3]) 

        self.fitnesses = []

# this is a test :-)

    def Run(self):
        for t in range(1000):
            p.stepSimulation()
            self.robot.Sense(t)
            self.robot.Think()
            self.robot.Act(t)
            if(self.directOrGUI == "GUI"):
                time.sleep(1/60) 
        # self.Get_Fitness()

    def Get_Fitness(self):
        self.robot.Get_Fitness()

    def __del__(self):
        p.disconnect()