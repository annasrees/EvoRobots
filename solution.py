import numpy as np
import pyrosim.pyrosim as pyrosim
import os
class SOLUTION:
    def __init__(self):
        self.weights = np.random.rand(3,2)
        self.weights = self.weights * 2 - 1

    def Create_World(self):
        pyrosim.Start_SDF("world.sdf") 
        pyrosim.End()
    def Create_Body(self):
        length = 1
        width = 1
        height = 1
        x = 0
        y = 0
        z = .5
        pyrosim.Start_URDF("body.urdf")

        pyrosim.Send_Cube(name="Torso", pos=[1.5,0,1.5] , size=[length, width, height]) #torso L0
        pyrosim.Send_Joint( name = "Torso_BackLeg" , parent= "Torso" , child = "BackLeg" , type = "revolute", position = [1, 0, 1]) #root link
        pyrosim.Send_Cube(name="BackLeg", pos=[-0.5,0,-0.5] , size=[length, width, height])
        pyrosim.Send_Joint( name = "Torso_FrontLeg" , parent= "Torso" , child = "FrontLeg" , type = "revolute", position = [2, 0, 1])
        pyrosim.Send_Cube(name="FrontLeg", pos=[0.5,0,-0.5] , size=[length, width, height])

        pyrosim.End()
    def Create_Brain(self):
        pyrosim.Start_NeuralNetwork("brain.nndf")
        # sensor value comes back -1.0 -> causing issues!
        pyrosim.Send_Sensor_Neuron(name = 0 , linkName = "Torso")
        pyrosim.Send_Sensor_Neuron(name = 1 , linkName = "BackLeg")
        pyrosim.Send_Sensor_Neuron(name = 2 , linkName = "FrontLeg")

        pyrosim.Send_Motor_Neuron( name = 3 , jointName = "Torso_BackLeg")
        pyrosim.Send_Motor_Neuron( name = 4 , jointName = "Torso_FrontLeg")

        for currentRow in range(0,3): #iterating over sensors
            for currentColumn in range(0,2): #iterating over motors\
                pyrosim.Send_Synapse(sourceNeuronName= currentRow, targetNeuronName= currentColumn + 3, weight = self.weights[currentRow][currentColumn])

    def Evaluate(self):
        self.Create_World()
        self.Create_Body()
        self.Create_Brain()
        # os.system("py simulate.py")
