from sensor import SENSOR
from motor import MOTOR
import pybullet as p
import pybullet_data
import pyrosim.pyrosim as pyrosim
import numpy as np
from constants import CONSTANTS as c
from pyrosim.neuralNetwork import NEURAL_NETWORK

class ROBOT:
    def __init__(self, sensors, motors):

        # empty dictionaries
        self.robotId = p.loadURDF("body.urdf")
        pyrosim.Prepare_To_Simulate(self.robotId)
        self.Prepare_To_Sense()
        self.Prepare_To_Act()
        self.nn = NEURAL_NETWORK("brain.nndf")

    def Prepare_To_Sense(self):
        self.sensors = {} #FILLs THESE WITH INSTANCES OF SENSORS
        for linkName in pyrosim.linkNamesToIndices:
            # creates an instance of SENSOR class, one for each link
            self.sensors[linkName] = SENSOR(linkName)

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


        # print(len(self.motors))
    def Act(self, t):
        for neuronName in self.nn.Get_Neuron_Names():
            if self.nn.Is_Motor_Neuron(neuronName):
                # print(self.motors)
                jointName = self.nn.Get_Motor_Neurons_Joint(neuronName)
                desiredAngle = self.nn.Get_Value_Of(neuronName)
                # print("desired angle:")
                # print(desiredAngle)
                # got rid of if statement
                motor_to_set = self.motors.get(jointName)
                motor_to_set.Set_Value(self, desiredAngle)
                # print(neuronName + ", " + jointName)
                # print(desiredAngle)
        # for i in self.motors:
        #     self.motors[i].Set_Value(self, t)


    def Think(self):
        self.nn.Update()
        self.nn.Print()

    def Get_Fitness(self):
        stateOfLinkZero = p.getLinkState(self.robotId,0)
        positionOfLinkZero = stateOfLinkZero[0]
        xCoordinateOfLinkZero = positionOfLinkZero[0]
        file = open("fitness.txt", "w")
        file.write(str(xCoordinateOfLinkZero))
        file.close()
        # print(xCoordinateOfLinkZero)
        # exit()

