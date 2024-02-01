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
        self.Prepare_To_Act()
        self.motorValues = []

    def Prepare_To_Act(self):
        self.amplitude = c.amplitude
        self.frequency = c.frequency
        self.offset = c.phaseOffset

        for i in range(1000):
          self.motorValues[i] = self.amplitude * np.sin(self.frequency * c.sinVals[i] + self.offset)

          
    def Set_Value(self, robot, t):
        pyrosim.Set_Motor_For_Joint(bodyIndex = robot.robotId,
                                    jointName = self.jointName,
                                    controlMode = p.POSITION_CONTROL,
                                    targetPosition = self.motorValues[t],
                                    maxForce = 500)
