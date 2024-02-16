import numpy as np
import pyrosim.pyrosim as pyrosim
import os
import random
class SOLUTION:
    def __init__(self, nextAvailableID):
        self.myID = nextAvailableID
        self.weights = np.random.rand(3,2)
        self.weights = self.weights * 2 - 1

    def Evaluate(self, directOrGUI):
        self.Create_world()
        self.Create_Body()
        self.Create_Brain()
        # if(directOrGUI == "GUI"):
        #     os.system("py simulate.py GUI")
        # else:
        #     os.system("py simulate.py DIRECT")
        # # read in string stored in fitness.txt
        # print("py simulate.py " + directOrGUI + str(self.myID))
        os.system("start /B py simulate.py " + directOrGUI + " " + str(self.myID)) #need to add 3rd arg here   
        fitnessFile = open("fitness.txt", "r")
        fitnessString = fitnessFile.read().strip()
        self.fitness = float(fitnessString)
        fitnessFile.close()

    def Create_world(self):
        pyrosim.Start_SDF("world.sdf") 
        pyrosim.Send_Cube(name="Box", pos=[4, 4, 0.5] , size=[1, 1, 1]) #position changed so out of way; module E step 20   
        pyrosim.End()

    def Create_Body(self):
        pyrosim.Start_URDF("body.urdf")

        pyrosim.Send_Cube(name="Torso", pos=[1.5,0,1.5] , size=[1, 1, 1]) #torso L0
        pyrosim.Send_Joint( name = "Torso_BackLeg" , parent= "Torso" , child = "BackLeg" , type = "revolute", position = [1, 0, 1]) #root link
        pyrosim.Send_Cube(name="BackLeg", pos=[-0.5,0,-0.5] , size=[1, 1, 1])
        pyrosim.Send_Joint( name = "Torso_FrontLeg" , parent= "Torso" , child = "FrontLeg" , type = "revolute", position = [2, 0, 1])
        pyrosim.Send_Cube(name="FrontLeg", pos=[0.5,0,-0.5] , size=[1, 1, 1])

        pyrosim.End()

    def Create_Brain(self):
        pyrosim.Start_NeuralNetwork("brain" + str(self.myID) + ".nndf")
        # sensor value comes back -1.0 -> causing issues!
        pyrosim.Send_Sensor_Neuron(name = 0 , linkName = "Torso")
        pyrosim.Send_Sensor_Neuron(name = 1 , linkName = "BackLeg")
        pyrosim.Send_Sensor_Neuron(name = 2 , linkName = "FrontLeg")

        pyrosim.Send_Motor_Neuron( name = 3 , jointName = "Torso_BackLeg")
        pyrosim.Send_Motor_Neuron( name = 4 , jointName = "Torso_FrontLeg")

        for currentRow in range(0,3): #iterating over sensors
            for currentColumn in range(0,2): #iterating over motors\
                pyrosim.Send_Synapse(sourceNeuronName= currentRow, targetNeuronName= currentColumn + 3, weight = self.weights[currentRow][currentColumn])

        pyrosim.End()

    def Mutate(self):
        randomRow = random.randint(0,2)
        randomCol = random.randint(0,1)
        self.weights[randomRow,randomCol] = random.random() * 2 - 1

    def Set_ID(self, nextAvailableID):
        self.myID = nextAvailableID




