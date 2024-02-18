import numpy as np
import pyrosim.pyrosim as pyrosim
import os
import random
import time
import subprocess
class SOLUTION:
    def __init__(self, nextAvailableID):
        self.numSensorNeurons = 3
        self.numMotorNeurons = 2
        self.myID = nextAvailableID
        self.weights = np.random.rand(self.numSensorNeurons, self.numMotorNeurons)
        self.weights = self.weights * 2 - 1

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

        for currentRow in range(0,self.numSensorNeurons): #iterating over sensors
            # check on this it might be wrong
            for currentColumn in range(0,1): #iterating over motors\
                pyrosim.Send_Synapse(sourceNeuronName= currentRow, targetNeuronName= currentColumn + self.numMotorNeurons, weight = self.weights[currentRow][currentColumn])

        pyrosim.End()
        

    def Mutate(self):
        randomRow = random.randint(0,self.numSensorNeurons - 1)
        randomCol = random.randint(0,2 - 1)
        self.weights[randomRow,randomCol] = random.random() * 2 - 1

    def Set_ID(self, nextAvailableID):
        self.myID = nextAvailableID




