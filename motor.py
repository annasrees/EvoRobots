from constants import CONSTANTS as c
import pybullet as p
import pybullet_data
import time
import pyrosim.pyrosim as pyrosim
import numpy as np
import math
import random

class MOTOR:
    def __init__(self, jointName):
        self.jointName = jointName
        # self.Prepare_To_Act()
        # self.motorValues = []


          
    def Set_Value(self, robot, t):
        # self.motorValues[i] = robot.motors[i]
        pyrosim.Set_Motor_For_Joint(bodyIndex = robot.robotId,
                                    jointName = self.jointName,
                                    controlMode = p.POSITION_CONTROL,
                                    targetPosition = robot.motorValues[t],
                                    maxForce = 500)
