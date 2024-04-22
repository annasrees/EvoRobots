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
        self.blockId = None
        self.robotId = p.loadURDF("body.urdf")
        pyrosim.Prepare_To_Simulate(self.robotId)
        self.Prepare_To_Sense()
        self.Prepare_To_Act()
        self.nn = NEURAL_NETWORK("brain" + solutionID + ".nndf")
        os.system("del brain" + str(solutionID) + ".nndf")

        self.jointIndices = {}

    def Prepare_To_Sense(self):
        self.sensors = {}
        for linkName in pyrosim.linkNamesToIndices:
            self.sensors[linkName] = SENSOR(linkName)

    def set_block_id(self, blockId):
        self.blockId = blockId      

    def Sense(self, t):
        for i in self.sensors:
            # originally just self.sensors[i].Get_Value(t)
            self.sensors[i].Get_Value(t)
        # frontLegSensorValues[i] = pyrosim.Get_Touch_Sensor_Value_For_Link("FrontLeg")

    def Prepare_To_Act(self):

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
        block_position = p.getBasePositionAndOrientation(self.blockId)
        basePosition = block_position[0]
        blockXPosition = basePosition[0]

        armJointNames = ["LeftUpperArm_LeftLowerArm", "RightUpperArm_RightLowerArm"]
        self.armJointIndices = []


        # Iterate over all joints
        for i in range(p.getNumJoints(self.robotId)):

            joint_info = p.getJointInfo(self.robotId, i)
            # Extract joint name
            joint_name = joint_info[1].decode('UTF-8')  # Convert byte string to regular string
            # Check if the joint name is in the list of arm joint names
            if joint_name in armJointNames:
                # If yes, append the joint index to the list
                self.armJointIndices.append(i)

        armJointPositions = []
        for joint in armJointNames:
            # Find the index of the current joint in armJointNames
            joint_index = armJointNames.index(joint)
            armJointPositions.append(p.getJointState(self.robotId, self.armJointIndices[joint_index])[0])
            self.jointIndices[joint] = joint

        avgArmPosition = 0
        relArmPosition = 0
        fitnessFunction = 0
        for i in range(len(armJointPositions)):
            relArmPosition += armJointPositions[i] - blockXPosition #want arm as far away from body as possible
        avgArmPosition = relArmPosition / (len(armJointPositions)) 

        botPositionAndOrientation = p.getBasePositionAndOrientation(self.robotId) #body position
        basePosition = botPositionAndOrientation[0]
        botXPosition = basePosition[0]
        botYPosition = basePosition[1]
        botZPosition = basePosition[2]

        relativeBotPos = botXPosition - blockXPosition

        fitnessFunction = botZPosition - relativeBotPos + relArmPosition

        file = open("tmp" + str(self.solutionID) + ".txt", "w")
        file.write(str(fitnessFunction)) #TEMPORARY
        file.close()
        print((fitnessFunction))
        os.rename("tmp"+str(self.solutionID)+".txt" , "Fitness"+str(self.solutionID)+".txt")
        file.close()

