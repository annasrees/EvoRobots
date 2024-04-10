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

        self.jointIndices = {}

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
        interested in...
        - the difference btwn the body pos. and the arm pos. 
            - if body_block > 0, arm_block < 0
            - if body_block < 0, arm_block > 0
        fitness = (some weight) * (wrap_factor) - (weight * distance btwn body and block)
            - wrap factor:
                body_block_diff * arm_block_diff < 0: (good)
                    wrap_factor = +1
                body_block_diff * arm_block_diff > 0: (bad)
                    wrap_factor = -1
        '''

        block_position = p.getBasePositionAndOrientation(self.blockId)
        basePosition = block_position[0]
        blockXPosition = basePosition[0]
        # Not sure i need this
        #blockYPosition = basePosition[1] #STAY UPRIGHT
        # blockZPosition = basePosition[2]

        armJointNames = ["LeftUpperArm_LeftLowerArm", "RightUpperArm_RightLowerArm"]
        self.armJointIndices = []

        # Iterate over all joints
        for i in range(p.getNumJoints(self.robotId)):

            # Get joint information
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
        for i in range(len(armJointPositions)):
            avgArmPosition += armJointPositions[i]
        avgArmPosition = avgArmPosition / (len(armJointPositions))

        botPositionAndOrientation = p.getBasePositionAndOrientation(self.robotId) #body position
        basePosition = botPositionAndOrientation[0]
        botXPosition = basePosition[0]
        botYPosition = basePosition[1]

        # WRAP FACTOR
        wrapFactor = 0

        # bodyBlockDifference = botXPosition - blockXPosition
        # armsBlockDifference = avgArmPosition - blockXPosition #might need to make 2 separate things for the 2 elbows tbd

        if (botXPosition < blockXPosition and avgArmPosition > blockXPosition):
            wrapFactor = 1
        elif(botXPosition > blockXPosition and avgArmPosition < blockXPosition):
            wrapFactor = -1
        else: 
            wrapFactor = 1

        # positive enforcement for bot's ability to wrap around the block AND stay upright
        fitnessFunction = wrapFactor + botYPosition

        file = open("tmp" + str(self.solutionID) + ".txt", "w")
        # file.write(str(blockXPosition) + "," + str(botXPosition) + "," + str(avgArmPosition))
        # file.close()
        # print(str(blockXPosition) + "," + str(botXPosition) + "," + str(avgArmPosition))
        # os.rename("tmp"+str(self.solutionID)+".txt" , "Fitness"+str(self.solutionID)+".txt")
        file.write(str(fitnessFunction)) #TEMPORARY
        file.close()
        print((fitnessFunction))
        os.rename("tmp"+str(self.solutionID)+".txt" , "Fitness"+str(self.solutionID)+".txt")
        file.close()
        # print(xCoordinateOfLinkZero)
        # exit()

