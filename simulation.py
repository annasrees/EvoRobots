from world import WORLD
from robot import ROBOT

import pybullet as p
import pybullet_data
import time
import pyrosim.pyrosim as pyrosim
import numpy as np
import math
import random

class SIMULATION:
    def __init__(self):

        self.physicsClient = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0,0,-9.8)
        p.loadSDF("world.sdf")
        self.world = WORLD()
        # seeing if I need to call world vals
        # self.world.planeId

        self.robot = ROBOT(2, 2)
 #this is supposed to be robotId
        self.planeId = p.loadURDF("plane.urdf")


    def Run(self):
        backAmplitude = (1)
        backFrequency = 10
        backPhaseOffset = np.pi / 4 #changed - step 48

        frontAmplitude = (1)
        frontFrequency = 10
        frontPhaseOffset = 0


        # backLegSensorValues = np.zeros(1000)
        # frontLegSensorValues = np.zeros(1000)

        sinVals = np.linspace(0, (2 * np.pi), 1000)
        backTargetAngles = np.zeros(len(sinVals))
        frontTargetAngles = np.zeros(len(sinVals))

        for i in range(len(sinVals)):
            backTargetAngles[i] = backAmplitude * np.sin(backFrequency * sinVals[i] + backPhaseOffset)
            frontTargetAngles[i] = frontAmplitude * np.sin(frontFrequency * sinVals[i] + frontPhaseOffset)

        for i in range(1000):
            p.stepSimulation()
            # #adjusting targetAngles - assignment 5 step 40
            
            # backLegSensorValues[i] = pyrosim.Get_Touch_Sensor_Value_For_Link("BackLeg")
            # frontLegSensorValues[i] = pyrosim.Get_Touch_Sensor_Value_For_Link("FrontLeg")
            # pyrosim.Set_Motor_For_Joint(bodyIndex = robotId,
            #                             jointName = "Torso_BackLeg",
            #                             controlMode = p.POSITION_CONTROL,
            #                             targetPosition = backTargetAngles[i],
            #                             maxForce = 500)
            # pyrosim.Set_Motor_For_Joint(bodyIndex = robotId,
            #                             jointName = "Torso_FrontLeg",
            #                             controlMode = p.POSITION_CONTROL,
            #                             targetPosition = frontTargetAngles[i],
            #                             maxForce = 500)
            time.sleep(1/60)

        def __del__(self):
            p.disconnect()