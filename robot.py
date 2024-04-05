from sensor import SENSOR
from motor import MOTOR
import pybullet as p
import pybullet_data
import pyrosim.pyrosim as pyrosim
import numpy as np
from constants import CONSTANTS as c
from pyrosim.neuralNetwork import NEURAL_NETWORK
import os
import time

class ROBOT:
    def __init__(self, sensors, motors, solutionID):
        self.solutionID = solutionID
        # empty dictionaries
        self.blockId = None  # Initialize block ID as None
        self.robotId = p.loadURDF("body.urdf")
        pyrosim.Prepare_To_Simulate(self.robotId)
        self.Prepare_To_Sense()
        self.Prepare_To_Act()
        self.nn = NEURAL_NETWORK("brain" + solutionID + ".nndf")
        os.system("del brain" + str(solutionID) + ".nndf") #this step isn't working

    def Prepare_To_Sense(self):
        self.sensors = {} #FILLs THESE WITH INSTANCES OF SENSORS
        for linkName in pyrosim.linkNamesToIndices:
            # creates an instance of SENSOR class, one for each link
            self.sensors[linkName] = SENSOR(linkName)

    def set_block_id(self, blockId):
        self.blockId = blockId 

    def Sense(self, t):
        for i in self.sensors:
            # originally just self.sensors[i].Get_Value(t)
            self.sensors[i].Get_Value(t)
        # frontLegSensorValues[i] = pyrosim.Get_Touch_Sensor_Value_For_Link("FrontLeg")

    def Prepare_To_Act(self):
        '''
        amplitude = 1
        frequency = 10
        phaseOffset = 0
        sinVals = np.linspace(0, (2 * np.pi), 1000)
        '''

        self.motors = {} #FILLs THESE WITH INSTANCES OF MOTORS
        
        for jointName in pyrosim.jointNamesToIndices:
            self.sinVals = np.linspace(0, (2 * np.pi), 1000) #initializing inside the for loop?
            self.motors[jointName] = MOTOR(jointName)
            self.motorValues = np.zeros(1000)

    def Act(self, t):
        motorJointRange = 0.2
        for neuronName in self.nn.Get_Neuron_Names():
            if self.nn.Is_Motor_Neuron(neuronName):

                jointName = self.nn.Get_Motor_Neurons_Joint(neuronName)
                desiredAngle = (self.nn.Get_Value_Of(neuronName)) * motorJointRange

                motor_to_set = self.motors.get(jointName)
                motor_to_set.Set_Value(self, desiredAngle)



    def Think(self):
        self.nn.Update()
        self.nn.Print()

    def Get_Fitness(self):
        '''
        block_bot_distance = 0 # distance between block and bot
        arm_pos = 0 #block pos - arm pos position of arms relative to block
        body_pos = 0 #block pos - body pos; position of body relative to block
        fitness = (some weight) * (block_bot_distance) + (another weight) * (arm_pos) + (another weight) * (body pos)
        '''

        basePositionAndOrientation = p.getBasePositionAndOrientation(self.robotId) #body position
        basePosition = basePositionAndOrientation[0]
        xPosition = basePosition[0]
        # basePositionAndOrientation = p.getBasePositionAndOrientation(self.robotId)
        # basePosition = basePositionAndOrientation[2]
        # xPosition = basePosition[2] #actually the z position
        file = open("tmp" + str(self.solutionID) + ".txt", "w")
        file.write(str(xPosition))
        file.close()
        print(xPosition)
        os.rename("tmp"+str(self.solutionID)+".txt" , "Fitness"+str(self.solutionID)+".txt")
        # file.close()
        # print(xCoordinateOfLinkZero)
        # exit()

