import numpy as np
import pyrosim.pyrosim as pyrosim
import os
import random
import time
import subprocess
class SOLUTION:
    def __init__(self, nextAvailableID):
        self.numSensorNeurons = 10
        self.numMotorNeurons = 9
        self.myID = nextAvailableID
        self.weights = np.random.rand(self.numSensorNeurons, self.numMotorNeurons) * 2 - 1

    def Evaluate(self, directOrGUI):
        pass
        # self.Create_world()
        # self.Create_Body()
        # self.Create_Brain()
        # if(directOrGUI == "GUI"):
        #     os.system("py simulate.py GUI")
        # else:
        #     os.system("py simulate.py DIRECT")
        # # read in string stored in fitness.txt
        # print("py simulate.py " + directOrGUI + str(self.myID))
        # os.system("start /B py simulate.py " + directOrGUI + " " + str(self.myID)) 
        # fitnessFileName = "fitness" + str(self.myID) + ".txt"
        # while not os.path.exists(fitnessFileName):
        #     time.sleep(0.01)
        # fitnessFile = open("fitness" + str(self.myID) + ".txt", "r")
        # fitnessString = fitnessFile.read().strip()
        # self.fitness = float(fitnessString)
        # print(self.fitness)
        # fitnessFile.close()

    def Start_Simulation(self, directOrGUI):
        self.Create_world()
        self.Create_Body()
        self.Create_Brain()
        self.Create_Block()

        if(directOrGUI == "DIRECT"):
            os.system("start /B py simulate.py " + directOrGUI + " " + str(self.myID))
        else:
            os.system("py simulate.py " + directOrGUI + " " + str(self.myID))
        # subprocess.run(["python", "simulate.py", directOrGUI, str(self.myID)])
         

    def Wait_For_Simulation_To_End(self):
        fitnessFileName = "Fitness" + str(self.myID) + ".txt"
        while not os.path.exists(fitnessFileName):
            time.sleep(0.01)
        # fitnessFile = open(fitnessFileName, "r")
        # fitnessString = fitnessFile.read().strip()
        # self.fitness = float(fitnessString)

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

        # # QUADRUPED
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


                #torso
        pyrosim.Send_Cube(name="Torso", pos=[0, 0, 2] , size=[1, 1, 1])
        #T-LUA joint
        pyrosim.Send_Joint( name = "Torso_LeftUpperArm" , parent= "Torso" , child = "LeftUpperArm" , type = "revolute", position = [0, -0.75, 2.5], jointAxis = "1 0 1") #root link
        #LUA
        pyrosim.Send_Cube(name="LeftUpperArm", pos=[0, -0.25, -0.15] , size=[0.2,1.0,0.2])
        #T-RUA
        pyrosim.Send_Joint( name = "Torso_RightUpperArm" , parent= "Torso" , child = "RightUpperArm" , type = "revolute", position = [0, 0.75, 2.5], jointAxis = "1 0 1")
        #RUA
        pyrosim.Send_Cube(name="RightUpperArm", pos=[0,0.25,-0.15] , size=[0.2,1.0,0.2])
        #T-H
        pyrosim.Send_Joint( name = "Torso_Head" , parent= "Torso" , child = "Head" , type = "revolute", position = [0, 0, 2.5], jointAxis = "1 1 0") #change joint axis?
        #H
        pyrosim.Send_Cube(name="Head", pos=[0,0,0.1] , size=[0.2,0.2,0.2])
        #T-LUL
        pyrosim.Send_Joint( name = "Torso_LeftUpperLeg" , parent= "Torso" , child = "LeftUpperLeg" , type = "revolute", position = [0, -0.5, 1.5], jointAxis = "0 1 0") #root link
        #LUL
        pyrosim.Send_Cube(name="LeftUpperLeg", pos=[0,-0.2,-0.25] , size=[0.4,0.4,1.0])
                #T-RUL
        pyrosim.Send_Joint( name = "Torso_RightUpperLeg" , parent= "Torso" , child = "RightUpperLeg" , type = "revolute", position = [0, 0.5, 1.5], jointAxis = "0 1 0") #root link
        #RUL
        pyrosim.Send_Cube(name="RightUpperLeg", pos=[0,0.2,-0.25] , size=[0.4,0.4,1.0])
        #LUL-LLL
        pyrosim.Send_Joint( name = "LeftUpperLeg_LeftLowerLeg" , parent= "LeftUpperLeg" , child = "LeftLowerLeg" , type = "revolute", position = [0, -0.2, -1], jointAxis = "0 1 0")
        #LLL
        pyrosim.Send_Cube(name="LeftLowerLeg", pos=[0,0,-0.1] , size=[0.4,0.4,0.75])
                #RUL-RLL
        pyrosim.Send_Joint( name = "RightUpperLeg_RightLowerLeg" , parent= "RightUpperLeg" , child = "RightLowerLeg" , type = "revolute", position = [0, 0.2, -1], jointAxis = "0 1 0")
        #RLL
        pyrosim.Send_Cube(name="RightLowerLeg", pos=[0,0,-0.1] , size=[0.4,0.4,0.75])
                #LUA-LLA
        pyrosim.Send_Joint( name = "LeftUpperArm_LeftLowerArm" , parent= "LeftUpperArm" , child = "LeftLowerArm" , type = "revolute", position = [0, -1, 0], jointAxis = "1 0 1")
        #LLA
        pyrosim.Send_Cube(name="LeftLowerArm", pos=[0,-0.1,-0.15] , size=[0.2,0.75,0.2])
                #RUA-RLA
        pyrosim.Send_Joint( name = "RightUpperArm_RightLowerArm" , parent= "RightUpperArm" , child = "RightLowerArm" , type = "revolute", position = [0, 1, 0], jointAxis = "1 0 1")
        #RLA
        pyrosim.Send_Cube(name="RightLowerArm", pos=[0,0.1,-0.15] , size=[0.2,0.75,0.2])


        pyrosim.End()


    def Create_Brain(self):
        pyrosim.Start_NeuralNetwork("brain" + str(self.myID) + ".nndf")
        # sensor value comes back -1.0 -> causing issues!
        pyrosim.Send_Sensor_Neuron(name = 0 , linkName = "Torso") #0
        pyrosim.Send_Sensor_Neuron(name = 1 , linkName = "LeftUpperArm") #1
        pyrosim.Send_Sensor_Neuron(name = 2 , linkName = "RightUpperArm") #2
        pyrosim.Send_Sensor_Neuron(name = 3 , linkName = "RightLowerArm") #3
        pyrosim.Send_Sensor_Neuron(name = 4 , linkName = "LeftLowerArm") #4
        pyrosim.Send_Sensor_Neuron(name = 5 , linkName = "RightUpperLeg") #5
        pyrosim.Send_Sensor_Neuron(name = 6 , linkName = "LeftLowerLeg") #6
        pyrosim.Send_Sensor_Neuron(name = 7 , linkName = "RightLowerLeg") #7
        pyrosim.Send_Sensor_Neuron(name = 8 , linkName = "Head") #8
        pyrosim.Send_Sensor_Neuron(name = 9 , linkName = "LeftUpperLeg") #9

        pyrosim.Send_Motor_Neuron(name = 10 , jointName = "Torso_LeftUpperArm") #0
        pyrosim.Send_Motor_Neuron(name = 11 , jointName = "Torso_RightUpperArm") #1
        pyrosim.Send_Motor_Neuron(name = 12 , jointName = "LeftUpperArm_LeftLowerArm") #2
        pyrosim.Send_Motor_Neuron(name = 13 , jointName = "RightUpperArm_RightLowerArm") #3
        pyrosim.Send_Motor_Neuron(name = 14 , jointName = "Torso_RightUpperLeg") #4
        pyrosim.Send_Motor_Neuron(name = 15 , jointName = "LeftUpperLeg_LeftLowerLeg") #5
        pyrosim.Send_Motor_Neuron(name = 16 , jointName = "RightUpperLeg_RightLowerLeg") #6
        pyrosim.Send_Motor_Neuron(name = 17 , jointName = "Torso_Head") #7
        pyrosim.Send_Motor_Neuron(name = 18, jointName = "Torso_LeftUpperLeg") #8
        

        for currentRow in range(0,self.numSensorNeurons): #iterating over sensors
            # check on this it might be wrong
            for currentColumn in range(0,self.numMotorNeurons): #iterating over motors\
                pyrosim.Send_Synapse(sourceNeuronName= currentRow, targetNeuronName= currentColumn + self.numSensorNeurons, weight = self.weights[currentRow][currentColumn])

        pyrosim.End()

    def Create_Block(self):
        pyrosim.Start_URDF("block.urdf")
        pyrosim.Send_Cube(name="Block", pos=[1, 0, 1.5] , size=[0.5, 0.5, 3])
        pyrosim.End()



    def Mutate(self):
        randomRow = random.randint(0,self.numSensorNeurons - 1)
        randomCol = random.randint(0,self.numMotorNeurons - 1)
        self.weights[randomRow,randomCol] = random.random() * 2 - 1

        #forcibly mutating the arm weights <3
        randomARMRow = random.randint(1,4)
        randomARMCol = random.randint(0,3)
        self.weights[randomARMRow,randomARMCol] = random.random() * 2 - 1

    def Set_ID(self, nextAvailableID):
        self.myID = nextAvailableID




