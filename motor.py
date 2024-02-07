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
        # acts as a copy of motorValues
        self.motorValues = np.zeros(1000)
        self.sinVals = np.linspace(0, (2 * np.pi), 1000)

        #making motorValues as part of the motor initialization (trying it)
        if jointName == "Torso_FrontLeg":
            # print("I found it!")
            self.amplitude = 1
            self.frequency = 10
            self.offset = 0
        else:
            # print("I also found this one too")
            self.amplitude = 1
            self.frequency = 5
            self.offset = 0

        for i in range(len(self.sinVals)): 
            self.motorValues[i] = self.amplitude * np.sin(self.frequency * self.sinVals[i] + self.offset)       

    def Set_Value(self, robot, desiredAngle):
        # self.motorValues[t] = robot.motorValues[t]
        # for i in range(len(self.sinVals)):
        #     self.motorValues = robot.amplitude * np.sin(robot.frequency * self.sinVals[i] + robot.offset)
        pyrosim.Set_Motor_For_Joint(bodyIndex = robot.robotId,
                                    jointName = self.jointName,
                                    controlMode = p.POSITION_CONTROL,
                                    targetPosition = desiredAngle,
                                    maxForce = 500)
        
    def Save_Values(self, robot):
        '''
        handles the saving of motor values
        '''
        outFile = open("../EvoRobots/data/" + self.jointName + "Data.npy", "wb")
        np.save(outFile, self.motorValues)
        outFile.close()
