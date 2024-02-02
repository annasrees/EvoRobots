from sensor import SENSOR
from motor import MOTOR
import pybullet as p
import pybullet_data
import pyrosim.pyrosim as pyrosim
import numpy as np
from constants import CONSTANTS as c

class ROBOT:
    def __init__(self, sensors, motors):

        # empty dictionaries
        self.robotId = p.loadURDF("body.urdf")
        # TODO: put Prepare_To_Simulate() in here?
        pyrosim.Prepare_To_Simulate(self.robotId)
        self.Prepare_To_Sense()
        self.Prepare_To_Act()



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
        self.amplitude = 1
        self.frequency = 10
        self.offset = 0
        self.sinVals = np.linspace(0, (2 * np.pi), 1000)
        self.motors = {} #FILLs THESE WITH INSTANCES OF MOTORS
        self.motorValues = np.zeros(1000)
   
        for jointName in pyrosim.jointNamesToIndices:
            self.motors[jointName] = MOTOR(jointName)
     
        for i in range(len(self.sinVals)):
            self.motorValues[i] = self.amplitude * np.sin(self.frequency * self.sinVals[i] + self.offset)

    def Act(self, t):
        for i in self.motors:
            self.motors[i].Set_Value(self, t)