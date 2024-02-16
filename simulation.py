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
    def __init__(self, directOrGUI):
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
        self.robot = ROBOT(2, 2)



    def Run(self):
        for t in range(100):
            p.stepSimulation()
            self.robot.Sense(t)
            self.robot.Think()
            self.robot.Act(t)
            if(self.directOrGUI == "GUI"):
                time.sleep(1/60) 

    def Get_Fitness(self):
        self.robot.Get_Fitness()


    def __del__(self):
        p.disconnect()