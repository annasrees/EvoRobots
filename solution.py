import numpy as np
import pyrosim.pyrosim as pyrosim
import os
import random
import time
import subprocess
import pybullet as p
class SOLUTION:
    def __init__(self, nextAvailableID, AorB):
        self.numSensorNeurons = 14
        self.numMotorNeurons = 13
        self.myID = nextAvailableID
        self.weights = np.random.rand(self.numSensorNeurons, self.numMotorNeurons) * 2 - 1

        # initializing AB testing
        self.A_or_B = AorB
        

    def Evaluate(self, directOrGUI):
        pass

    def Start_Simulation(self, directOrGUI):
        self.Create_world()
        self.Create_Body()
        self.Create_Brain()
        if self.A_or_B =="A":
            self.Create_Block_A()
        else:
            self.Create_Block_B()

        if(directOrGUI == "DIRECT"):
            os.system("start /B py simulate.py " + directOrGUI + " " + str(self.myID))
        else:
            os.system("py simulate.py " + directOrGUI + " " + str(self.myID))
        # subprocess.run(["python", "simulate.py", directOrGUI, str(self.myID)])
         

    def Wait_For_Simulation_To_End(self):
        fitnessFileName = "Fitness" + str(self.myID) + ".txt"
        while not os.path.exists(fitnessFileName):
            time.sleep(0.01)

        while True:
            try:
                with open(fitnessFileName, "r") as fitnessFile:
                    fitnessString = fitnessFile.read().strip()
                    self.fitness = float(fitnessString)
                break
            except PermissionError:
                time.sleep(0.01)
        fitnessFile.close()
        os.remove(fitnessFileName)


    def Create_world(self):
        pyrosim.Start_SDF("world.sdf") 

        pyrosim.End()

    def Create_Body(self):
        pyrosim.Start_URDF("body.urdf")

        '''QUADRUPED'''
        # pyrosim.Send_Cube(name="Torso", pos=[0, 0, 1] , size=[1, 1, 1]) #torso L0
        # pyrosim.Send_Joint( name = "Torso_BackLeg" , parent= "Torso" , child = "BackLeg" , type = "revolute", position = [0, -0.5, 1], jointAxis = "1 0 0") #root link
        # pyrosim.Send_Cube(name="BackLeg", pos=[0, -0.5, 0] , size=[0.2,1,0.2])
        # pyrosim.Send_Joint( name = "Torso_FrontLeg" , parent= "Torso" , child = "FrontLeg" , type = "revolute", position = [0, 0.5, 1], jointAxis = "1 0 0")
        # pyrosim.Send_Cube(name="FrontLeg", pos=[0,0.5,0] , size=[0.2,1,0.2])
        # pyrosim.Send_Joint( name = "Torso_LeftLeg" , parent= "Torso" , child = "LeftLeg" , type = "revolute", position = [-0.5, 0, 1], jointAxis = "0 1 0")
        # pyrosim.Send_Cube(name="LeftLeg", pos=[-0.5,0,0] , size=[1.0,0.2,0.2])
        # pyrosim.Send_Joint( name = "Torso_RightLeg" , parent= "Torso" , child = "RightLeg" , type = "revolute", position = [0.5, 0, 1], jointAxis = "0 1 0")
        # pyrosim.Send_Cube(name="RightLeg", pos=[0.5,0,0] , size=[1.0,0.2,0.2])
        # pyrosim.Send_Joint( name = "FrontLeg_FrontLowerLeg" , parent= "FrontLeg" , child = "FrontLowerLeg" , type = "revolute", position = [0, 1, 0], jointAxis = "1 0 0")
        # pyrosim.Send_Cube(name="FrontLowerLeg", pos=[0,0,-0.5] , size=[0.2,0.2,1.0]) 
        # pyrosim.Send_Joint( name = "BackLeg_BackLowerLeg" , parent= "BackLeg" , child = "BackLowerLeg" , type = "revolute", position = [0, -1.0, 0], jointAxis = "1 0 0")
        # pyrosim.Send_Cube(name="BackLowerLeg", pos=[0,0,-0.5] , size=[0.2,0.2,1.0])
        # pyrosim.Send_Joint( name = "LeftLeg_LeftLowerLeg" , parent= "LeftLeg" , child = "LeftLowerLeg" , type = "revolute", position = [-1.0, 0, 0], jointAxis = "1 0 0")
        # pyrosim.Send_Cube(name="LeftLowerLeg", pos=[0,0,-0.5] , size=[0.2,0.2,1.0])
        # pyrosim.Send_Joint( name = "RightLeg_RightLowerLeg" , parent= "RightLeg" , child = "RightLowerLeg" , type = "revolute", position = [1.0, 0, 0], jointAxis = "1 0 0")
        # pyrosim.Send_Cube(name="RightLowerLeg", pos=[0,0,-0.5] , size=[0.2,0.2,1.0])


        # #torso
        pyrosim.Send_Cube(name="Torso", pos=[0, 0, 2] , size=[1, 1, 1], mass=1.0)
        #T-LUA joint
        pyrosim.Send_Joint(name = "Torso_LeftShoulder", parent = "Torso", child = "LeftShoulder", type = "revolute", position = [0, -0.5, 2.5], jointAxis= "0 0 1")

        pyrosim.Send_Cube(name="LeftShoulder", pos=[0, -0.1, -0.1] , size=[0.2, 0.2, 0.2], mass=1.0)

        pyrosim.Send_Joint(name = "Torso_RightShoulder", parent = "Torso", child = "RightShoulder", type = "revolute", position = [0, 0.5, 2.5], jointAxis= "0 0 1")

        pyrosim.Send_Cube(name="RightShoulder", pos=[0, 0.1, -0.1] , size=[0.2, 0.2, 0.2], mass=1.0)

        pyrosim.Send_Joint( name = "LeftShoulder_LeftUpperArm" , parent= "LeftShoulder" , child = "LeftUpperArm" , type = "revolute", position = [0, -0.2, 0], jointAxis = "0 0 1") 
        #LUA
        pyrosim.Send_Cube(name="LeftUpperArm", pos=[0, -0.25, -0.1] , size=[0.2,1,0.2], mass=1.0)
        #T-RUA
        pyrosim.Send_Joint( name = "RightShoulder_RightUpperArm" , parent= "RightShoulder" , child = "RightUpperArm" , type = "revolute", position = [0, 0.2, 0], jointAxis = "0 0 1")
        #RUA
        pyrosim.Send_Cube(name="RightUpperArm", pos=[0,0.25,-0.1] , size=[0.2,1,0.2], mass=1.0) 
        #T-H
        pyrosim.Send_Joint( name = "Torso_Head" , parent= "Torso" , child = "Head" , type = "revolute", position = [0, 0, 2.5], jointAxis = "1 1 0") #change joint axis?
        #H
        pyrosim.Send_Cube(name="Head", pos=[0,0,0.1] , size=[0.2,0.2,0.2], mass=1.0)
        #T-LUL
        pyrosim.Send_Joint( name = "Torso_LeftUpperLeg" , parent= "Torso" , child = "LeftUpperLeg" , type = "revolute", position = [0, -0.5, 1.5], jointAxis = "0 1 0") #root link
        #LUL
        pyrosim.Send_Cube(name="LeftUpperLeg", pos=[0,-0.2,-0.25] , size=[0.4,0.4,0.75], mass=10.0)
                #T-RUL
        pyrosim.Send_Joint( name = "Torso_RightUpperLeg" , parent= "Torso" , child = "RightUpperLeg" , type = "revolute", position = [0, 0.5, 1.5], jointAxis = "0 1 0") #root link
        #RUL
        pyrosim.Send_Cube(name="RightUpperLeg", pos=[0,0.2,-0.25] , size=[0.4,0.4,0.75], mass=10.0)
        #LUL-LLL
        pyrosim.Send_Joint( name = "LeftUpperLeg_LeftLowerLeg" , parent= "LeftUpperLeg" , child = "LeftLowerLeg" , type = "revolute", position = [0, -0.2, -0.75], jointAxis = "0 1 0")
        #LLL
        pyrosim.Send_Cube(name="LeftLowerLeg", pos=[0,0,-0.25] , size=[0.4,0.4,0.75], mass= 15.0)
                #RUL-RLL
        pyrosim.Send_Joint( name = "RightUpperLeg_RightLowerLeg" , parent= "RightUpperLeg" , child = "RightLowerLeg" , type = "revolute", position = [0, 0.2, -0.75], jointAxis = "0 1 0")
        #RLL
        pyrosim.Send_Cube(name="RightLowerLeg", pos=[0,0,-0.25] , size=[0.4,0.4,0.75], mass= 15.0)
                #LUA-LLA
        pyrosim.Send_Joint( name = "LeftUpperArm_LeftLowerArm" , parent= "LeftUpperArm" , child = "LeftLowerArm" , type = "revolute", position = [0, 0.25, 0], jointAxis = "0 0 1")
        #LLA
        pyrosim.Send_Cube(name="LeftLowerArm", pos=[0.5,-1.1,-0.1] , size=[1.0,0.2,0.2], mass=1.0) #0.375,-1.1,-0.1
                #RUA-RLA
        pyrosim.Send_Joint( name = "RightUpperArm_RightLowerArm" , parent= "RightUpperArm" , child = "RightLowerArm" , type = "revolute", position = [0, 0.25, 0], jointAxis = "0 0 1")
        #RLA
        pyrosim.Send_Cube(name="RightLowerArm", pos=[0.5,0.6,-0.1] , size=[1.0,0.2,0.2], mass=1.0) #0.35,0.6,-0.1

        #giving him feet...
        pyrosim.Send_Joint(name = "LeftLowerLeg_LeftFoot", parent= "LeftLowerLeg" , child = "LeftFoot" , type = "revolute", position = [0, 0.2, -0.6], jointAxis = "0 1 0")
        pyrosim.Send_Cube(name = "LeftFoot", pos = [0.25,-0.2,0], size = [.75, 0.5, 0.05], mass=50.0)

        pyrosim.Send_Joint(name = "RightLowerLeg_RightFoot", parent= "RightLowerLeg" , child = "RightFoot" , type = "revolute", position = [0, 0.2, -0.6], jointAxis = "0 1 0")
        pyrosim.Send_Cube(name = "RightFoot", pos = [0.25,-0.2,0], size = [.75, 0.5, 0.05], mass=50.0)

        pyrosim.End()


    def Create_Brain(self):
        pyrosim.Start_NeuralNetwork("brain" + str(self.myID) + ".nndf")
        # sensor value comes back -1.0 -> causing issues!
        pyrosim.Send_Sensor_Neuron(name = 0 , linkName = "Torso") #0
        pyrosim.Send_Sensor_Neuron(name = 1 , linkName = "LeftUpperArm") #1
        pyrosim.Send_Sensor_Neuron(name = 2 , linkName = "RightUpperArm") #2
        pyrosim.Send_Sensor_Neuron(name = 3 , linkName = "RightLowerArm") #3
        pyrosim.Send_Sensor_Neuron(name = 4 , linkName = "LeftLowerArm") #4
        pyrosim.Send_Sensor_Neuron(name = 5 , linkName = "LeftShoulder") #5
        pyrosim.Send_Sensor_Neuron(name = 6 , linkName = "RightShoulder") #6
        pyrosim.Send_Sensor_Neuron(name = 7 , linkName = "RightLowerLeg") #7
        pyrosim.Send_Sensor_Neuron(name = 8 , linkName = "Head") #8
        pyrosim.Send_Sensor_Neuron(name = 9 , linkName = "LeftUpperLeg") #9
        pyrosim.Send_Sensor_Neuron(name = 10 , linkName = "RightUpperLeg") #9
        pyrosim.Send_Sensor_Neuron(name = 11 , linkName = "LeftLowerLeg") #9
        pyrosim.Send_Sensor_Neuron(name = 12 , linkName = "LeftFoot") #9
        pyrosim.Send_Sensor_Neuron(name = 13 , linkName = "RightFoot") #9

        pyrosim.Send_Motor_Neuron(name = 14 , jointName = "Torso_LeftShoulder") #0
        pyrosim.Send_Motor_Neuron(name = 15 , jointName = "Torso_RightShoulder") #1
        pyrosim.Send_Motor_Neuron(name = 16 , jointName = "LeftUpperArm_LeftLowerArm") #2
        pyrosim.Send_Motor_Neuron(name = 17 , jointName = "RightUpperArm_RightLowerArm") #3
        pyrosim.Send_Motor_Neuron(name = 18 , jointName = "LeftShoulder_LeftUpperArm") #4
        pyrosim.Send_Motor_Neuron(name = 19 , jointName = "RightShoulder_RightUpperArm") #5
        pyrosim.Send_Motor_Neuron(name = 20 , jointName = "RightUpperLeg_RightLowerLeg") #6
        pyrosim.Send_Motor_Neuron(name = 21 , jointName = "Torso_Head") #7
        pyrosim.Send_Motor_Neuron(name = 22, jointName = "Torso_LeftUpperLeg") #8
        pyrosim.Send_Motor_Neuron(name = 23, jointName = "Torso_RightUpperLeg") #8
        pyrosim.Send_Motor_Neuron(name = 24, jointName = "LeftUpperLeg_LeftLowerLeg") #8
        pyrosim.Send_Motor_Neuron(name = 25, jointName = "LeftLowerLeg_LeftFoot") #8
        pyrosim.Send_Motor_Neuron(name = 26, jointName = "RightLowerLeg_RightFoot") #8
        

        for currentRow in range(0,self.numSensorNeurons): #iterating over sensors
            # check on this it might be wrong
            for currentColumn in range(0,self.numMotorNeurons): #iterating over motors\
                pyrosim.Send_Synapse(sourceNeuronName= currentRow, targetNeuronName= currentColumn + self.numSensorNeurons, weight = self.weights[currentRow][currentColumn])

        pyrosim.End()

    def Create_Block_A(self):
        pyrosim.Start_URDF("block.urdf")
        pyrosim.Send_Cube(name="Block", pos=[1, 0, 1.5] , size=[0.5, 0.5, 3], mass=100.0)
        pyrosim.End()

    def Create_Block_B(self):
        pyrosim.Start_URDF("block.urdf")
        pyrosim.Send_Cube(name="Block", pos=[5, 0, 1.5] , size=[0.5, 0.5, 3], mass=100.0) #starting 5 units away
        pyrosim.End()


    def Mutate(self):
        randomRow = random.randint(0,self.numSensorNeurons - 1)
        randomCol = random.randint(0,self.numMotorNeurons - 1)
        self.weights[randomRow,randomCol] = random.random() * 2 - 1

        #forcibly mutating the arm weights <3
        randomARMRow = random.randint(1,6)
        randomARMCol = random.randint(0,5)
        self.weights[randomARMRow,randomARMCol] = random.random() * 2 - 1

    def Set_ID(self, nextAvailableID):
        self.myID = nextAvailableID




